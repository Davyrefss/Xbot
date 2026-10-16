#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
把 autotune.py --csv 导出的曲线文件画出来(不重新动电机, 只看已存数据)。

用法:
    python autotune.py COM7 --csv curve.csv      # 先存数据
    python plot_curve.py curve.csv               # 再画
依赖: pip install matplotlib
"""
import sys, csv, argparse


def main():
    ap = argparse.ArgumentParser(description='画 autotune.py 导出的 CSV 曲线')
    ap.add_argument('csv_path', help='CSV 文件路径')
    ap.add_argument('--title', default=None, help='图标题')
    args = ap.parse_args()

    try:
        import matplotlib
        import matplotlib.pyplot as plt
    except ImportError:
        print('未安装 matplotlib, 请先: python -m pip install matplotlib')
        return

    plt.rcParams['font.sans-serif'] = ['Microsoft YaHei', 'SimHei', 'SimSun', 'DejaVu Sans']
    plt.rcParams['axes.unicode_minus'] = False

    steps = {}
    with open(args.csv_path, 'r', encoding='utf-8') as f:
        rd = csv.reader(f)
        header = next(rd, None)          # step,t,rpm
        for row in rd:
            if len(row) < 3:
                continue
            name = row[0]
            try:
                t, rpm = float(row[1]), float(row[2])
            except ValueError:
                continue
            steps.setdefault(name, []).append((t, rpm))

    if not steps:
        print('CSV 里没有数据: %s' % args.csv_path)
        return

    plt.figure(figsize=(10, 6))
    for name, samples in steps.items():
        plt.plot([t for t, _ in samples], [r for _, r in samples],
                 marker='.', markersize=2, label=name)
    plt.xlabel('t (s)')
    plt.ylabel('rpm')
    plt.title(args.title or args.csv_path)
    plt.legend()
    plt.grid(True)
    plt.show()


if __name__ == '__main__':
    main()
