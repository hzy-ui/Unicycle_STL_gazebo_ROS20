#!/usr/bin/env python3

def print_c(text: str, color: str = "default", style: str = "default", 
            background: str = "default", rgb_color: tuple = None, rgb_bg: tuple = None) -> None:
    """
    彩色打印函数，支持更多颜色、样式和背景颜色。
    
    参数：
    - text (str): 要打印的文本内容。
    - color (str): 文本颜色（支持16色/256色/自定义RGB），可选值：
      - 基础颜色: "black", "red", "green", "yellow", "blue", "magenta", "cyan", "white"
      - 亮色: "bright_black", "bright_red", 等等
      - 256色: "color_<0-255>"，如 "color_123"
      - 默认: "default"
    - style (str): 文本样式，可选值：
      - 默认: "default"
      - 高亮: "bold"
      - 下划线: "underline"
      - 斜体: "italic"
      - 删除线: "strikethrough"
      - 闪烁: "blink"
      - 反显: "reverse"
      - 隐藏: "hidden"
    - background (str): 背景颜色（支持16色/256色/自定义RGB），格式同color。
    - rgb_color (tuple): 自定义文本颜色，格式为 (r, g, b)，0 <= r, g, b <= 255。
    - rgb_bg (tuple): 自定义背景颜色，格式为 (r, g, b)，0 <= r, g, b <= 255。
    """
    # ANSI颜色代码映射
    base_colors = {
        "default": 39,
        "black": 30, "red": 31, "green": 32, "yellow": 33,
        "blue": 34, "magenta": 35, "cyan": 36, "white": 37,
        "bright_black": 90, "bright_red": 91, "bright_green": 92, "bright_yellow": 93,
        "bright_blue": 94, "bright_magenta": 95, "bright_cyan": 96, "bright_white": 97,
    }

    # ANSI样式代码映射
    styles = {
        "default": 0, "bold": 1, "italic": 3, "underline": 4,
        "blink": 5, "reverse": 7, "hidden": 8, "strikethrough": 9,
    }

    # 获取颜色代码
    def get_color_code(color, is_background=False):
        if color.startswith("color_") and color[6:].isdigit():
            # 256色支持
            code = int(color[6:])
            return f"{48 if is_background else 38};5;{code}"
        elif rgb_color and not is_background:
            return f"38;2;{rgb_color[0]};{rgb_color[1]};{rgb_color[2]}"
        elif rgb_bg and is_background:
            return f"48;2;{rgb_bg[0]};{rgb_bg[1]};{rgb_bg[2]}"
        else:
            return base_colors.get(color.lower(), 39)

    # 获取样式代码
    style_code = styles.get(style.lower(), 0)

    # 构造 ANSI 转义序列
    color_code = get_color_code(color)
    bg_code = get_color_code(background, is_background=True)
    ansi_code = f"\033[{style_code};{color_code};{bg_code}m"

    # 打印彩色文本并重置样式
    print(f"{ansi_code}{text}\033[0m")


# 示例用法
# print_c("Hello, 16 colors!", color="bright_red", background="blue", style="bold")
# print_c("Hello, 256 colors!", color="color_123", background="color_200", style="underline")
# print_c("Hello, RGB!", rgb_color=(255, 165, 0), rgb_bg=(0, 0, 0), style="italic")

def format_color_log(
    text: str, 
    color: str = "default", 
    style: str = "default"
) -> str:
    """
    格式化彩色日志字符串，用于 self.get_logger().info(...)。

    参数：
    - text (str): 要输出的日志内容。
    - color (str): 文本颜色，可选值：
      - 基础颜色: "black", "red", "green", "yellow", "blue", "magenta", "cyan", "white"
      - 亮色: "bright_black", "bright_red", 等等
      - 默认: "default"
    - style (str): 文本样式，可选值：
      - 默认: "default"
      - 高亮: "bold"
      - 下划线: "underline"
      - 斜体: "italic"  (新增支持)
      - 闪烁: "blink"
      - 反显: "reverse"
      - 隐藏: "hidden"
    返回：
    - str: 带有ANSI转义序列的格式化字符串。
    """
    # ANSI颜色代码映射
    colors = {
        "default": 39,
        "black": 30,
        "red": 31,
        "green": 32,
        "yellow": 33,
        "blue": 34,
        "magenta": 35,
        "cyan": 36,
        "white": 37,
        "bright_black": 90,
        "bright_red": 91,
        "bright_green": 92,
        "bright_yellow": 93,
        "bright_blue": 94,
        "bright_magenta": 95,
        "bright_cyan": 96,
        "bright_white": 97,
    }
    # ANSI样式代码映射（新增 italic 支持）
    styles = {
        "default": 0,
        "bold": 1,
        "italic": 3,  # 斜体样式
        "underline": 4,
        "blink": 5,
        "reverse": 7,
        "hidden": 8,
    }

    # 获取颜色和样式代码
    color_code = colors.get(color.lower(), 39)
    style_code = styles.get(style.lower(), 0)

    # 返回带有ANSI转义序列的字符串
    return f"\033[{style_code};{color_code}m{text}\033[0m"

# 使用彩色日志
# usage
# log_red_bold = format_color_log("This is a bold red log!", color="red", style="bold")
# log_green_underline = format_color_log("This is an underlined green log!", color="green", style="underline")
# log_default = format_color_log("This is a default log.")

# node.get_logger().info(log_red_bold)
# node.get_logger().info(log_green_underline)
# node.get_logger().info(log_default)
