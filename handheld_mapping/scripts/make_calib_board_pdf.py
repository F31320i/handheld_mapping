#!/usr/bin/env python3
"""Generate calib_boards_<paper>.pdf: print sheets for the handheld rig.

Two different targets, both needed:

  * ArUco markers for FAST-Calib's LiDAR-camera extrinsic target. The marker
    geometry is not free-form -- it is fixed by src/qr_detect.hpp upstream:
    DICT_6X6_250, and the four corner markers carry IDs 1, 2, 4, 3 in reading
    order (top-left, top-right, bottom-right, bottom-left). A marker with the
    wrong dictionary or the wrong ID simply fails to find a board pose, with no
    diagnostic beyond "not detected".

  * A 9x7-square checkerboard for the camera intrinsics, which FAST-Calib
    needs before it can do anything (qr_params.yaml takes the intrinsics).

Every size here comes from upstream: config/qr_params.yaml (marker_size, the
delta_* fields) and pics/calibration_target.jpg (the technical drawing).

Two papers, identical content and identical physical sizes:

  A4  the 200 mm marker spans 200 of the 210 mm short edge: a 5 mm margin,
      inside the unprintable border of plenty of inkjets.
  A3  48.5 mm on the same edge. Use it if an A4 print comes out clipped.

Both print the checkerboard at 20 mm squares, so `--square 0.020` is correct
for either sheet and the two cannot be mixed up.

Text is mixed: Droid Sans Fallback has CJK but not one Latin glyph, Helvetica
has Latin but no CJK, so every string is split into per-script runs.

    python3 make_calib_board_pdf.py --paper A4|A3 [-o out.pdf]
"""
import argparse
import os

import cv2
from fontTools.ttLib import TTFont as FTFont
from reportlab.lib.units import mm
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
from reportlab.pdfgen import canvas

# --- upstream geometry (do not "round" these; qr_params.yaml must match) ----
DICT = cv2.aruco.DICT_6X6_250
MARKER_SIZE = 200.0                     # marker_size 0.20
MARKER_IDS = [(1, '左上'), (2, '右上'), (4, '右下'), (3, '左下')]
CH_SQ, CH_COLS, CH_ROWS = 20.0, 9, 7    # 9x7 squares = 8x6 interior corners
                                        # 20 mm because 9 * 25 > 210 (portrait A4)

PAPERS = {'A4': (210.0, 297.0), 'A3': (297.0, 420.0)}
PAPER = 'A4'
W, H = PAPERS[PAPER]                    # rebound by main()
AW, AH = PAPERS['A4']                   # the layout the cover/checker pages
                                        # are drawn against; see sheet_offset

PT = 25.4 / 72.0                        # 1 pt in mm, since the canvas is in mm

CJK_PATH = '/usr/share/fonts/truetype/droid/DroidSansFallbackFull.ttf'
FONT_CJK = 'CJK'
FONT_LATIN = 'Helvetica'
FONT_MONO = 'Courier'
CJK_START = 0x2E80                      # below this the Latin font is used
WINANSI_EXTRA = u'°±·×Ø–—'
_cjk_cmap = set()


def _prepare_fonts():
    global _cjk_cmap
    pdfmetrics.registerFont(TTFont(FONT_CJK, CJK_PATH))
    _cjk_cmap = set(FTFont(CJK_PATH, fontNumber=0).getBestCmap())


def _covered(ch):
    if ord(ch) < 0x7F or ch in WINANSI_EXTRA:
        return True
    return ord(ch) in _cjk_cmap


def runs(s, latin=None):
    """Split into (font, text) runs so every glyph has a font that has it."""
    latin = latin or FONT_LATIN
    out = []
    for ch in s:
        if not _covered(ch):
            raise ValueError('no font covers %r (U+%04X) in %r' % (ch, ord(ch), s))
        f = latin if ord(ch) < CJK_START else FONT_CJK
        if out and out[-1][0] == f:
            out[-1][1].append(ch)
        else:
            out.append([f, [ch]])
    return [(f, ''.join(chs)) for f, chs in out]


def text_width(c, s, size, font=None):
    sz = size * PT
    return sum(c.stringWidth(t, f, sz) for f, t in runs(s, font))


def text(c, x, y, s, size=3.2, font=None, color=(0, 0, 0), align='l'):
    sz = size * PT
    rr = runs(s, font)
    if align != 'l':
        w = sum(c.stringWidth(t, f, sz) for f, t in rr)
        x -= w / 2.0 if align == 'c' else w
    c.setFillColorRGB(*color)
    for f, t in rr:
        c.setFont(f, sz)
        c.drawString(x, y, t)
        x += c.stringWidth(t, f, sz)
    return x


def para(c, x, y, lines, size=3.2, leading=4.6, color=(0, 0, 0), font=None):
    for i, line in enumerate(lines):
        text(c, x, y - i * leading, line, size, font, color)
    return y - len(lines) * leading


def rule(c, x1, y1, x2, y2, width=0.25, color=(0, 0, 0)):
    c.setLineWidth(width)
    c.setStrokeColorRGB(*color)
    c.line(x1, y1, x2, y2)


def box(c, x, y, w, h, width=0.3, color=(0, 0, 0)):
    c.setLineWidth(width)
    c.setStrokeColorRGB(*color)
    c.rect(x, y, w, h, stroke=1, fill=0)


def note_box(c, x, y, w, lines, title=None, size=3.0, leading=4.2,
             pad=3.0, accent=(0.80, 0.10, 0.10)):
    h = pad * 2 + leading * (len(lines) + (1.7 if title else 0)) - 1.5
    box(c, x, y - h, w, h, width=0.4, color=accent)
    yy = y - pad - size
    if title:
        text(c, x + pad, yy, title, size * 1.15, color=accent)
        yy -= leading * 1.7
    for line in lines:
        text(c, x + pad, yy, line, size)
        yy -= leading
    return y - h - 1.0


# ----------------------------------------------------------------- drawing --

def marker_bits(marker_id):
    """8x8 module grid of a DICT_6X6_250 marker: 6x6 payload + 1-module border.

    True means white. Built from OpenCV so the payload bits are by construction
    the ones the detector expects, rather than hand-copied from a picture.
    """
    d = cv2.aruco.getPredefinedDictionary(DICT)
    img = cv2.aruco.drawMarker(d, marker_id, 8 * 20)
    bits = [[bool(img[r * 20 + 10, c * 20 + 10] > 127) for c in range(8)]
            for r in range(8)]
    for i in range(8):                  # the border must be solid black
        assert not bits[0][i] and not bits[7][i]
        assert not bits[i][0] and not bits[i][7]
    return bits


class Sheet(object):
    """A page whose user space is millimetres, origin bottom-left."""

    def __init__(self, c):
        self.c = c

    def begin(self, w, h, dx=0.0, dy=0.0):
        self.c.setPageSize((w * mm, h * mm))
        self.c.saveState()
        self.c.scale(mm, mm)
        self.c.translate(dx, dy)
        self.w, self.h = w, h

    def end(self):
        self.c.restoreState()
        self.c.showPage()


def draw_marker(c, x, y, size, bits):
    n = len(bits)
    m = size / n
    c.setFillColorRGB(0, 0, 0)
    c.rect(x, y, size, size, stroke=0, fill=1)
    c.setFillColorRGB(1, 1, 1)
    for r in range(n):
        for col in range(n):
            if bits[r][col]:
                c.rect(x + col * m, y + (n - 1 - r) * m, m, m, stroke=0, fill=1)


def ruler_v(c, x, y0, length, step=10.0, major=50.0):
    """Vertical reference bar: the only way to catch a scaled print."""
    rule(c, x, y0, x, y0 + length, 0.5)
    for i in range(int(round(length / step)) + 1):
        y = y0 + i * step
        ln = 4.0 if i * step % major == 0 else 2.0
        rule(c, x - ln, y, x, y, 0.25)
        if i * step % major == 0:
            text(c, x - ln - 0.8, y - 1.0, '%d' % (i * step), 2.5, align='r')
    text(c, x + 1.5, y0 + length / 2 + 2, '200.0 mm', 3.4)
    text(c, x + 1.5, y0 + length / 2 - 2.5, '用钢尺量这一段', 2.6,
         color=(0.45, 0.45, 0.45))


# -------------------------------------------------------------------- pages --

def sheet_offset(w, h):
    """Where to put the A4-sized layout on a bigger sheet.

    The cover and checker pages are laid out once, at A4 coordinates, and then
    translated so that on A3 they sit centred rather than jammed into the top
    left with 90 mm of white on two sides. Zero on A4, so that file is bit-for-bit
    the layout it always had.
    """
    return (w - AW) / 2.0, (h - AH) / 2.0


def page_intro(c):
    s = Sheet(c)
    s.begin(W, H, *sheet_offset(W, H))
    L, R = 15.0, 178.0                    # text column; the ruler sits right of R

    text(c, L, AH - 25, 'FAST-Calib 靶标 + 相机内参标定板', 7.0)
    text(c, L, AH - 33, 'Livox Mid-360  /  Logitech C310  ·  %s  ·  必须 1:1 打印'
         % PAPER, 3.4, color=(0.4, 0.4, 0.4))

    y = AH - 45
    y = note_box(c, L, y, R - L, [
        '打印对话框里把缩放改成「实际大小 / 100%」，关掉「适应页面」',
        '「缩放以适合可打印区域」「每张纸打印多页」。任何一个都会让尺寸',
        '失真，整份标定作废。打完先量本页右侧那根竖线：必须是 200.0 mm。',
    ], title='打印设置（最重要的一步，先做这个）')

    y -= 8
    text(c, L, y, '纸张上的内容', 4.2)
    y -= 7
    for a, b in [
        ('第 2–5 页', '四个 ArUco 二维码，黑框边长 200.0 mm（1:1）'),
        ('第 6 页', '相机内参棋盘格：9×7 格，格边长 20.0 mm'),
    ]:
        text(c, L + 2, y, a, 3.2, color=(0.4, 0.4, 0.4))
        text(c, L + 30, y, b, 3.2)
        y -= 6.2

    y -= 4
    text(c, L, y, '四个码的 ID 不是 1 / 2 / 3 / 4', 4.2)
    y -= 7
    y = para(c, L, y, [
        '顺序是 左上 = 1、右上 = 2、右下 = 4、左下 = 3（源码 qr_detect.hpp:111-114 写死）。',
        '贴反了检测不到板子位姿，日志只说没检测到，不会说 ID 错了。',
        '字典是 DICT_6X6_250，换一个字典等于换了一套码，检测直接失败。',
    ], 3.2, 4.5)

    y -= 6
    text(c, L, y, '靶板参数（贴到板子上之后填进 qr_params.yaml）', 4.2)
    y -= 5
    y = para(c, L, y, [
        '靶板是 1400 × 1000 mm 的刚性板，四角各一个 200 mm 二维码，中间四个',
        'Ø240 mm 通孔，二维码中心距 1100 × 700 mm。A4 只有 210 mm 宽，装不下，',
        '所以这里只按真实尺寸印码，板子和孔另外做。',
    ], 3.2, 4.5)
    y -= 3
    mon = [
        'marker_size: 0.20              # 实测黑框边长，不是 0.20 就按实测填',
        'delta_width_qr_center: 0.55    # 码心横向半距 = 1100/2',
        'delta_height_qr_center: 0.35   # 码心纵向半距 = 700/2',
        'delta_width_circles: 0.50      # 孔心横向距 = 500',
        'delta_height_circles: 0.40     # 孔心纵向距 = 400',
        'circle_radius: 0.12            # 孔半径',
    ]
    step = 4.0
    top = y - 1.5
    bot = top - (len(mon) - 1) * step - 2.6
    c.setFillColorRGB(0.955, 0.955, 0.955)
    c.rect(L + 2, bot - 1.8, (R - L) - 4, top - bot + 6.2, stroke=0, fill=1)
    for i, line in enumerate(mon):
        text(c, L + 4, top - i * step, line, 2.6, font=FONT_MONO)

    ruler_v(c, 190.0, 40, 200.0)

    text(c, L, 16, 'handheld_mapping / scripts/make_calib_board_pdf.py  ·  参数取自 '
                   'hku-mars/FAST-Calib 的 qr_params.yaml 与技术图纸',
         2.4, color=(0.55, 0.55, 0.55))
    s.end()


def page_marker(c, marker_id, where, index, total):
    s = Sheet(c)
    s.begin(W, H)
    # Bias the block upwards: 30 mm of the spare height above the marker, the
    # rest below, where the labels want it. A4 -> marker at y = 78.5, 19 mm of
    # headroom; A3 -> 140, i.e. 80 mm, and 48.5 mm to either side.
    my = (H - MARKER_SIZE + 60.0) / 2.0
    draw_marker(c, (W - MARKER_SIZE) / 2.0, my, MARKER_SIZE, marker_bits(marker_id))

    text(c, W / 2, my - 14, 'ArUco  ID = %d' % marker_id, 7.0, align='c')
    text(c, W / 2, my - 22, '位置：%s（第 %d / %d 个）' % (where, index, total), 4.0,
         align='c', color=(0.35, 0.35, 0.35))
    text(c, W / 2, my - 30, '黑框边长 200.0 mm  ·  DICT_6X6_250', 3.2,
         align='c', color=(0.35, 0.35, 0.35))

    # Cap the box at the A4 width and centre it: on A3 a full-width box would be
    # 267 mm around two short lines.
    nw = min(W - 30.0, 180.0)
    note_box(c, (W - nw) / 2.0, my - 38, nw, [
        '沿黑框外沿裁下（宁可多留 1 mm 白边，不要裁进黑框）。贴完量一下黑框，',
        '不是 200.0 mm 就按实测值改 qr_params.yaml 的 marker_size。',
    ])

    text(c, W / 2, my - 67, '打印请选「实际大小 / 100%」', 2.8, align='c',
         color=(0.6, 0.6, 0.6))
    s.end()


def page_checker(c):
    """Portrait, like every other page: 9 * 20 mm = 180 mm fits the short edge.

    At 25 mm the pattern would be 225 mm wide and would need landscape on A4,
    i.e. flipping the paper in the middle of a print job. The square size is
    deliberately the same on A3 -- the calibration command then does not depend
    on which sheet was printed.
    """
    s = Sheet(c)
    s.begin(W, H, *sheet_offset(W, H))
    bw = CH_COLS * CH_SQ                  # 180
    bh = CH_ROWS * CH_SQ                  # 140
    x0 = (AW - bw) / 2.0
    y0 = AH - 52.0 - bh

    c.setFillColorRGB(0, 0, 0)
    for r in range(CH_ROWS):
        for col in range(CH_COLS):
            if (r + col) % 2 == 0:
                c.rect(x0 + col * CH_SQ, y0 + (CH_ROWS - 1 - r) * CH_SQ,
                       CH_SQ, CH_SQ, stroke=0, fill=1)

    text(c, AW / 2, AH - 25, '相机内参标定板', 6.0, align='c')
    text(c, AW / 2, AH - 37, '9 × 7 格  ·  格边长 20.0 mm  ·  内角点 8 × 6', 3.4,
         align='c', color=(0.35, 0.35, 0.35))
    text(c, AW / 2, y0 - 12, 'cameracalibrator --size 8x6 --square 0.020', 3.2,
         align='c', font=FONT_MONO, color=(0.35, 0.35, 0.35))

    ybar = y0 - 35
    rule(c, 45, ybar, 145, ybar, 0.5)
    for i in range(11):
        x = 45 + i * 10
        ln = 4.0 if i % 5 == 0 else 2.0
        rule(c, x, ybar - ln, x, ybar, 0.25)
    text(c, 148, ybar - 1.2, '100.0 mm', 3.2)
    text(c, AW / 2, ybar - 15, '用钢尺量这一段的长度，必须正好 100.0 mm', 2.8,
         align='c', color=(0.5, 0.5, 0.5))

    text(c, AW / 2, 20, '打印请选「实际大小 / 100%」', 2.8, align='c',
         color=(0.6, 0.6, 0.6))
    s.end()


def main():
    global PAPER, W, H
    ap = argparse.ArgumentParser()
    ap.add_argument('--paper', default='A4', choices=sorted(PAPERS),
                    help='A3 gives the 200 mm markers a 48.5 mm edge margin '
                         'instead of 5 mm')
    # Generated documents live in the project's docs/, not inside the ROS
    # package: the package is source, docs/ is where the output is collected.
    ap.add_argument('-o', '--out', default=None)
    args = ap.parse_args()

    PAPER = args.paper
    W, H = PAPERS[PAPER]
    out = args.out or os.path.expanduser(
        '~/ClaudeCode_ws/post_VMD/docs/calib_boards_%s.pdf' % PAPER)
    out = os.path.abspath(out)
    os.makedirs(os.path.dirname(out), exist_ok=True)

    _prepare_fonts()
    c = canvas.Canvas(out, pagesize=(W * mm, H * mm))
    c.setTitle('FAST-Calib ArUco markers + camera checkerboard (%s, 1:1)' % PAPER)
    c.setAuthor('handheld_mapping')

    page_intro(c)
    for i, (mid, where) in enumerate(MARKER_IDS, 1):
        page_marker(c, mid, where, i, len(MARKER_IDS))
    page_checker(c)
    c.save()

    margin = (W - MARKER_SIZE) / 2.0
    print('wrote %s  (%s %.0f x %.0f mm, marker edge margin %.1f mm)'
          % (out, PAPER, W, H, margin))


if __name__ == '__main__':
    main()
