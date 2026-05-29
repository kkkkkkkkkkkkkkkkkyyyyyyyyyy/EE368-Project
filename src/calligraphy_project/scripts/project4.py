import os
import argparse
import warnings
from collections import deque

import cv2
import numpy as np
from skimage.morphology import skeletonize

warnings.filterwarnings("ignore")


class ImageToRobotPath:
    """
    project4.py 改进版：
    1. 保留原来的图片预处理、边缘检测、轮廓显示功能；
    2. 新增“输入文字 -> 生成文字图片 -> 骨架路径 -> robot_path.txt”；
    3. image_to_path 会返回 normalized_paths，方便后续和机械臂控制程序对接。

    输出路径格式：
        x_norm y_norm
        x_norm y_norm
        BREAK
        x_norm y_norm

    其中 x_norm, y_norm 都在 [0, 1]，是图像归一化坐标。
    机械臂端需要再把它映射到真实纸面坐标。
    """

    def __init__(self):
        self.canny_low = 50
        self.canny_high = 150
        self.trackbars_created = False

        # 路径处理参数
        self.border_size = 5
        self.merge_dist = 6.0          # 路径端点距离小于该值时尝试合并，单位：像素
        self.simplify_epsilon = 2.0    # 路径简化参数，越大点越少
        self.min_path_len = 2

    # =========================================================
    # 任务 0：文字生成图片
    # =========================================================
    def _find_default_font(self):
        """自动寻找常见中文字体。"""
        candidates = [
            # Windows
            "C:/Windows/Fonts/simhei.ttf",
            "C:/Windows/Fonts/msyh.ttc",
            "C:/Windows/Fonts/simsun.ttc",
            "C:/Windows/Fonts/simkai.ttf",
            # Linux / Ubuntu 常见中文字体
            "/usr/share/fonts/truetype/wqy/wqy-microhei.ttc",
            "/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc",
            "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
            "/usr/share/fonts/opentype/noto/NotoSerifCJK-Regular.ttc",
            # macOS
            "/System/Library/Fonts/PingFang.ttc",
            "/System/Library/Fonts/STHeiti Light.ttc",
        ]

        for path in candidates:
            if os.path.exists(path):
                return path
        return None

    def text_to_gray_image(
        self,
        text,
        font_path=None,
        canvas_w=900,
        canvas_h=300,
        font_size=180,
        save_path="text_generated.png"
    ):
        """
        将输入文字渲染为 OpenCV 灰度图。
        黑字白底，方便后续 cv2.THRESH_BINARY_INV 二值化。
        """
        try:
            from PIL import Image, ImageDraw, ImageFont
        except ImportError:
            raise ImportError(
                "缺少 Pillow，请先安装：pip install pillow"
            )

        if not text or not text.strip():
            raise ValueError("输入文字为空，无法生成文字图片。")

        if font_path is None:
            font_path = self._find_default_font()

        if font_path is None or not os.path.exists(font_path):
            raise FileNotFoundError(
                "没有找到可用中文字体。请用 --font 指定字体路径，例如：\n"
                "Windows: C:/Windows/Fonts/simhei.ttf\n"
                "Ubuntu: /usr/share/fonts/truetype/wqy/wqy-microhei.ttc"
            )

        # 创建白色背景灰度图
        img = Image.new("L", (canvas_w, canvas_h), 255)
        draw = ImageDraw.Draw(img)
        font = ImageFont.truetype(font_path, font_size)

        # 支持多行文字
        lines = text.split("\\n")
        line_bboxes = [draw.textbbox((0, 0), line, font=font) for line in lines]
        line_widths = [bbox[2] - bbox[0] for bbox in line_bboxes]
        line_heights = [bbox[3] - bbox[1] for bbox in line_bboxes]

        line_gap = int(font_size * 0.20)
        total_h = sum(line_heights) + line_gap * (len(lines) - 1)
        start_y = (canvas_h - total_h) // 2

        y = start_y
        for line, bbox, tw, th in zip(lines, line_bboxes, line_widths, line_heights):
            x = (canvas_w - tw) // 2 - bbox[0]
            draw.text((x, y - bbox[1]), line, font=font, fill=0)
            y += th + line_gap

        img_gray = np.array(img)

        if save_path:
            cv2.imwrite(save_path, img_gray)
            print(f"文字图片已保存到: {save_path}")

        return img_gray

    def text_to_paths(
        self,
        text,
        output_file="robot_path.txt",
        font_path=None,
        canvas_w=900,
        canvas_h=300,
        font_size=180,
        save_debug=True
    ):
        """
        输入文字，输出：
        1. 文字灰度图；
        2. 骨架图；
        3. 像素坐标路径；
        4. 归一化坐标路径。
        """
        img_gray = self.text_to_gray_image(
            text=text,
            font_path=font_path,
            canvas_w=canvas_w,
            canvas_h=canvas_h,
            font_size=font_size,
            save_path="text_generated.png" if save_debug else None
        )

        skeleton_img, pixel_paths, normalized_paths = self.image_to_path(
            img_gray=img_gray,
            output_file=output_file,
            save_debug=save_debug
        )

        return img_gray, skeleton_img, pixel_paths, normalized_paths

    # =========================================================
    # 任务一：图像预处理
    # =========================================================
    def preprocess_image(self, img):
        """图像预处理流程：灰度化、高斯降噪、Canny 边缘检测、形态学增强。"""
        img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        img_blur = cv2.GaussianBlur(img_gray, (5, 5), 1)

        def update_canny(val):
            self.canny_low = cv2.getTrackbarPos("Canny Low", "Preprocessing")
            self.canny_high = cv2.getTrackbarPos("Canny High", "Preprocessing")

        if not self.trackbars_created:
            cv2.namedWindow("Preprocessing")
            cv2.createTrackbar("Canny Low", "Preprocessing", self.canny_low, 200, update_canny)
            cv2.createTrackbar("Canny High", "Preprocessing", self.canny_high, 300, update_canny)
            self.trackbars_created = True

        edges = cv2.Canny(img_blur, self.canny_low, self.canny_high)

        kernel = np.ones((5, 5), np.uint8)
        dilated = cv2.dilate(edges, kernel, iterations=1)
        enhanced = cv2.erode(dilated, kernel, iterations=1)

        return img, img_gray, edges, enhanced

    def extract_and_visualize_contours(self, img, processed_img):
        """轮廓提取与可视化。"""
        contours, _ = cv2.findContours(
            processed_img,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        contour_img = img.copy()
        cv2.drawContours(contour_img, contours, -1, (0, 255, 0), 2)

        return contour_img, contours

    def stack_images(self, img_list, scale=0.7):
        """拼接显示图像。"""
        resized_rows = []
        for row in img_list:
            row_imgs = []
            for img in row:
                if len(img.shape) == 2:
                    img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
                h, w = img.shape[:2]
                new_w, new_h = int(w * scale), int(h * scale)
                resized = cv2.resize(img, (new_w, new_h))
                row_imgs.append(resized)
            resized_rows.append(np.hstack(row_imgs))

        return np.vstack(resized_rows)

    # =========================================================
    # 任务二：轨迹规划
    # =========================================================
    def image_to_path(self, img_gray, output_file="robot_path.txt", save_debug=True):
        """
        从灰度图到机械臂路径的转换。

        返回：
            skeleton_img: 骨架图
            optimized_paths: 像素坐标路径 [(x, y), ...]
            normalized_paths: 归一化路径 [(x_norm, y_norm), ...]
        """
        if img_gray is None:
            raise ValueError("img_gray 为空，无法生成路径。")

        if len(img_gray.shape) == 3:
            img_gray = cv2.cvtColor(img_gray, cv2.COLOR_BGR2GRAY)

        # 1. 图像二值化
        # 黑字白底时，反向二值化后：文字为 255，背景为 0
        _, binary = cv2.threshold(img_gray, 127, 255, cv2.THRESH_BINARY_INV)

        # 2. 边界补充
        # 注意：这里必须补 0，即背景；如果补 255，会把图像外圈也当成路径。
        b = self.border_size
        binary = cv2.copyMakeBorder(
            binary,
            b, b, b, b,
            cv2.BORDER_CONSTANT,
            value=0
        )

        # 3. 骨架提取：粗笔画 -> 单像素中心线
        skeleton = skeletonize(binary // 255)
        skeleton_img = (skeleton * 255).astype(np.uint8)

        # 4. 简单断点修复
        skeleton_img = self._repair_breakpoints(skeleton_img)

        # 5. 路径提取与优化
        paths = self._extract_paths(skeleton_img)
        optimized_paths = self._optimize_paths(paths)

        # 6. 坐标归一化与文件输出
        normalized_paths = self._normalize_coordinates(optimized_paths, skeleton_img.shape)
        self._save_path_file(normalized_paths, output_file)

        if save_debug:
            cv2.imwrite("text_skeleton.png", skeleton_img)
            path_viz = self.visualize_paths(skeleton_img, optimized_paths)
            cv2.imwrite("text_path_preview.png", path_viz)
            print("骨架图已保存到: text_skeleton.png")
            print("路径预览图已保存到: text_path_preview.png")

        return skeleton_img, optimized_paths, normalized_paths

    def _repair_breakpoints(self, skeleton_img):
        """
        简单修复 1 像素级断点。
        说明：这个函数只做轻量修复，不保证复杂断裂全部修好。
        """
        height, width = skeleton_img.shape
        repaired = skeleton_img.copy()

        directions = [
            (-1, -1), (-1, 0), (-1, 1),
            (0, -1),           (0, 1),
            (1, -1),  (1, 0),  (1, 1)
        ]

        # 遍历背景点，如果它两侧有骨架点，则补上
        for y in range(1, height - 1):
            for x in range(1, width - 1):
                if skeleton_img[y, x] > 0:
                    continue

                for dy, dx in directions:
                    y1, x1 = y + dy, x + dx
                    y2, x2 = y - dy, x - dx
                    if skeleton_img[y1, x1] > 0 and skeleton_img[y2, x2] > 0:
                        repaired[y, x] = 255
                        break

        return repaired

    def _get_neighbors(self, skeleton_img, x, y):
        """获取某个骨架像素的 8 邻域骨架点，返回 [(nx, ny), ...]。"""
        height, width = skeleton_img.shape
        neighbors = []
        for dy in [-1, 0, 1]:
            for dx in [-1, 0, 1]:
                if dx == 0 and dy == 0:
                    continue
                nx, ny = x + dx, y + dy
                if 0 <= nx < width and 0 <= ny < height:
                    if skeleton_img[ny, nx] > 0:
                        neighbors.append((nx, ny))
        return neighbors

    def _count_neighbors(self, img, x, y):
        """计算 8 邻域骨架邻居数量。"""
        return len(self._get_neighbors(img, x, y))

    def _edge_key(self, p1, p2):
        """无向边 key，用于记录路径边是否访问过。"""
        return tuple(sorted([p1, p2]))

    def _extract_paths(self, skeleton_img):
        """
        提取有序路径。

        相比简单 BFS，这里按骨架图的图结构追踪路径：
        - 邻居数 != 2 的点视为节点，包括端点、分叉点；
        - 从节点出发沿骨架追踪到下一个节点；
        - 对纯闭环路径单独处理。
        """
        height, width = skeleton_img.shape
        pixels = set()
        degree = {}

        for y in range(height):
            for x in range(width):
                if skeleton_img[y, x] > 0:
                    p = (x, y)
                    pixels.add(p)

        if not pixels:
            return []

        for p in pixels:
            degree[p] = len(self._get_neighbors(skeleton_img, p[0], p[1]))

        # 节点：端点或分叉点；普通中间点 degree == 2
        nodes = {p for p in pixels if degree[p] != 2}

        visited_edges = set()
        paths = []

        # 从节点出发追踪路径段
        for node in nodes:
            for nb in self._get_neighbors(skeleton_img, node[0], node[1]):
                e = self._edge_key(node, nb)
                if e in visited_edges:
                    continue

                path = [node]
                prev = node
                cur = nb
                visited_edges.add(e)

                while True:
                    path.append(cur)

                    # 到达下一个节点，结束当前路径段
                    if cur in nodes and cur != node:
                        break

                    nbs = self._get_neighbors(skeleton_img, cur[0], cur[1])
                    next_candidates = [p for p in nbs if p != prev]

                    if not next_candidates:
                        break

                    # 正常骨架中间点一般只有一个 next
                    nxt = next_candidates[0]
                    e = self._edge_key(cur, nxt)
                    if e in visited_edges:
                        break

                    visited_edges.add(e)
                    prev, cur = cur, nxt

                if len(path) >= self.min_path_len:
                    paths.append(path)

        # 处理纯闭环：所有点 degree == 2，没有端点/分叉点
        for p in pixels:
            nbs = self._get_neighbors(skeleton_img, p[0], p[1])
            for nb in nbs:
                e = self._edge_key(p, nb)
                if e in visited_edges:
                    continue

                path = [p]
                start = p
                prev = p
                cur = nb
                visited_edges.add(e)

                while True:
                    path.append(cur)
                    nbs_cur = self._get_neighbors(skeleton_img, cur[0], cur[1])
                    next_candidates = [q for q in nbs_cur if q != prev]

                    if not next_candidates:
                        break

                    nxt = next_candidates[0]
                    e = self._edge_key(cur, nxt)
                    if e in visited_edges:
                        break

                    visited_edges.add(e)
                    prev, cur = cur, nxt

                    if cur == start:
                        path.append(cur)
                        break

                if len(path) >= self.min_path_len:
                    paths.append(path)

        return paths

    def _optimize_paths(self, paths):
        """路径合并与简化。"""
        if not paths:
            return []

        # 1. 尝试合并相邻路径
        merged_paths = []
        used = [False] * len(paths)

        for i in range(len(paths)):
            if used[i]:
                continue

            current_path = list(paths[i])
            used[i] = True

            merged = True
            while merged:
                merged = False

                for j in range(len(paths)):
                    if used[j]:
                        continue

                    start_i, end_i = current_path[0], current_path[-1]
                    start_j, end_j = paths[j][0], paths[j][-1]

                    distances = [
                        np.linalg.norm(np.array(end_i) - np.array(start_j)),
                        np.linalg.norm(np.array(end_i) - np.array(end_j)),
                        np.linalg.norm(np.array(start_i) - np.array(start_j)),
                        np.linalg.norm(np.array(start_i) - np.array(end_j))
                    ]

                    min_dist = min(distances)
                    if min_dist < self.merge_dist:
                        idx = distances.index(min_dist)
                        if idx == 0:      # end_i -> start_j
                            current_path.extend(paths[j])
                        elif idx == 1:    # end_i -> end_j
                            current_path.extend(reversed(paths[j]))
                        elif idx == 2:    # start_i -> start_j
                            current_path = list(reversed(paths[j])) + current_path
                        else:             # start_i -> end_j
                            current_path = list(paths[j]) + current_path

                        used[j] = True
                        merged = True
                        break

            merged_paths.append(current_path)

        # 2. 路径简化，减少机械臂规划点数
        simplified_paths = []
        for path in merged_paths:
            if len(path) < 3:
                simplified_paths.append(path)
                continue

            path_array = np.array(path, dtype=np.float32).reshape((-1, 1, 2))
            approx = cv2.approxPolyDP(path_array, self.simplify_epsilon, False)

            simplified = [tuple(point[0]) for point in approx]
            if len(simplified) >= self.min_path_len:
                simplified_paths.append(simplified)

        return simplified_paths

    def _normalize_coordinates(self, paths, img_shape):
        """
        将像素坐标归一化到 [0, 1]。

        注意：
        - x_norm 左 -> 右；
        - y_norm 上 -> 下；
        - 后续机械臂端需要根据纸面坐标系再转换。
        """
        height, width = img_shape
        b = self.border_size
        original_w = max(1, width - 2 * b)
        original_h = max(1, height - 2 * b)

        normalized_paths = []
        for path in paths:
            normalized_path = []
            for x, y in path:
                x_norm = (float(x) - b) / float(original_w - 1)
                y_norm = (float(y) - b) / float(original_h - 1)

                x_norm = max(0.0, min(1.0, x_norm))
                y_norm = max(0.0, min(1.0, y_norm))

                normalized_path.append((x_norm, y_norm))

            if len(normalized_path) >= self.min_path_len:
                normalized_paths.append(normalized_path)

        return normalized_paths

    def _save_path_file(self, paths, filename):
        """保存路径文件。"""
        with open(filename, "w", encoding="utf-8") as f:
            for i, path in enumerate(paths):
                for x, y in path:
                    f.write(f"{x:.6f} {y:.6f}\n")
                if i < len(paths) - 1:
                    f.write("BREAK\n")

        print(f"路径已保存到: {filename}")
        print(f"总路径数: {len(paths)}")
        print(f"总点数: {sum(len(p) for p in paths)}")

    def load_path_file(self, filename="robot_path.txt"):
        """
        读取 robot_path.txt，返回 normalized_paths。
        后续机械臂控制程序可以直接复用这个函数。
        """
        paths = []
        current = []

        with open(filename, "r", encoding="utf-8") as f:
            for line in f:
                line = line.strip()
                if not line:
                    continue

                if line.upper() == "BREAK":
                    if len(current) >= self.min_path_len:
                        paths.append(current)
                    current = []
                    continue

                parts = line.split()
                if len(parts) != 2:
                    continue

                x, y = float(parts[0]), float(parts[1])
                current.append((x, y))

        if len(current) >= self.min_path_len:
            paths.append(current)

        return paths

    def visualize_paths(self, skeleton_img, paths):
        """可视化提取的路径。"""
        if len(skeleton_img.shape) == 2:
            display_img = cv2.cvtColor(skeleton_img, cv2.COLOR_GRAY2BGR)
        else:
            display_img = skeleton_img.copy()

        colors = [
            (255, 0, 0),
            (0, 255, 0),
            (0, 0, 255),
            (255, 255, 0),
            (255, 0, 255),
            (0, 255, 255),
            (128, 128, 255),
            (255, 128, 128),
        ]

        for i, path in enumerate(paths):
            color = colors[i % len(colors)]

            for j in range(len(path) - 1):
                x1, y1 = int(path[j][0]), int(path[j][1])
                x2, y2 = int(path[j + 1][0]), int(path[j + 1][1])
                cv2.line(display_img, (x1, y1), (x2, y2), color, 2)

            # 起点绿色，终点红色
            if path:
                x, y = int(path[0][0]), int(path[0][1])
                cv2.circle(display_img, (x, y), 5, (0, 255, 0), -1)
                x, y = int(path[-1][0]), int(path[-1][1])
                cv2.circle(display_img, (x, y), 5, (0, 0, 255), -1)

        return display_img

    # =========================================================
    # 原来的交互式图片模式
    # =========================================================
    def run_interactive_image_mode(self, image_path="image2.png"):
        """
        保留原来的交互式模式：
        - 读取图片；
        - 显示预处理结果；
        - 按空格生成路径；
        - q 退出。
        """
        img = cv2.imread(image_path)
        if img is None:
            print(f"错误: 无法读取图像，请检查路径: {image_path}")
            return

        print("按空格键处理当前帧并生成路径，按 q 键退出。")

        while True:
            original, gray, edges, enhanced = self.preprocess_image(img.copy())
            contour_img, contours = self.extract_and_visualize_contours(original, enhanced)

            display_array = [
                [original, cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)],
                [cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR), contour_img]
            ]

            stacked_img = self.stack_images(display_array, scale=0.6)
            cv2.imshow("Preprocessing Results", stacked_img)
            cv2.imshow("Enhanced Edges", enhanced)

            key = cv2.waitKey(1) & 0xFF
            if key == ord(" "):
                print("开始轨迹规划...")
                skeleton_img, pixel_paths, normalized_paths = self.image_to_path(gray)
                path_viz = self.visualize_paths(skeleton_img, pixel_paths)
                cv2.imshow("Extracted Paths", path_viz)
                print("轨迹规划完成！")

            elif key == ord("q"):
                break

        cv2.destroyAllWindows()


# =========================================================
# 主程序
# =========================================================
def main():
    parser = argparse.ArgumentParser(description="文字/图片转机械臂书写路径 project4.py")

    parser.add_argument("--text", type=str, default=None, help="输入想让机械臂书写的文字，例如：--text 雷灿")
    parser.add_argument("--image", type=str, default=None, help="输入图片路径，例如：--image image2.png")
    parser.add_argument("--output", type=str, default="robot_path.txt", help="输出路径文件名")

    parser.add_argument("--font", type=str, default=None, help="中文字体路径，例如 C:/Windows/Fonts/simhei.ttf")
    parser.add_argument("--canvas-w", type=int, default=900, help="文字图片画布宽度")
    parser.add_argument("--canvas-h", type=int, default=300, help="文字图片画布高度")
    parser.add_argument("--font-size", type=int, default=180, help="文字字号")

    parser.add_argument("--show", action="store_true", help="生成后显示文字、骨架和路径预览窗口")
    parser.add_argument("--no-debug", action="store_true", help="不保存 text_generated/text_skeleton/text_path_preview 调试图")
    parser.add_argument("--interactive", action="store_true", help="使用原来的交互式图片处理模式")

    args = parser.parse_args()

    processor = ImageToRobotPath()
    save_debug = not args.no_debug

    # =========================
    # 新增：如果用户没有传任何文字/图片参数，
    # 就直接在终端里输入想写的文字
    # =========================
    if args.text is None and args.image is None and not args.interactive:
        user_text = input("请输入想让机械臂书写的文字：").strip()
        if user_text:
            args.text = user_text
        else:
            print("输入为空，程序退出。")
            return

    # 1. 输入文字模式
    if args.text is not None:
        print(f"输入文字: {args.text}")
        img_gray, skeleton_img, pixel_paths, normalized_paths = processor.text_to_paths(
            text=args.text,
            output_file=args.output,
            font_path=args.font,
            canvas_w=args.canvas_w,
            canvas_h=args.canvas_h,
            font_size=args.font_size,
            save_debug=save_debug
        )

        if args.show:
            path_viz = processor.visualize_paths(skeleton_img, pixel_paths)
            cv2.imshow("Text Image", img_gray)
            cv2.imshow("Skeleton", skeleton_img)
            cv2.imshow("Path Preview", path_viz)
            print("按任意键关闭窗口。")
            cv2.waitKey(0)
            cv2.destroyAllWindows()

        print("文字路径生成完成，可以把 robot_path.txt 交给机械臂端读取执行。")
        return

    # 2. 输入图片直接转路径模式
    if args.image is not None and not args.interactive:
        img = cv2.imread(args.image)
        if img is None:
            print(f"错误: 无法读取图像，请检查路径: {args.image}")
            return

        img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        skeleton_img, pixel_paths, normalized_paths = processor.image_to_path(
            img_gray=img_gray,
            output_file=args.output,
            save_debug=save_debug
        )

        if args.show:
            path_viz = processor.visualize_paths(skeleton_img, pixel_paths)
            cv2.imshow("Input Image Gray", img_gray)
            cv2.imshow("Skeleton", skeleton_img)
            cv2.imshow("Path Preview", path_viz)
            print("按任意键关闭窗口。")
            cv2.waitKey(0)
            cv2.destroyAllWindows()

        print("图片路径生成完成，可以把 robot_path.txt 交给机械臂端读取执行。")
        return

    # 3. 原来的交互式图片模式
    image_path = args.image if args.image is not None else "image2.png"
    processor.run_interactive_image_mode(image_path=image_path)
if __name__ == "__main__":
    main()