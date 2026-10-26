#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-2-Clause
# Copyright (c) 2026 Johns Hopkins University, LCSR

"""
Generate printable ChArUco calibration boards in PDF, SVG, or PNG formats
with physical millimeter accuracy and calibration scale verification rulers.
"""

import argparse
import sys
from pathlib import Path

try:
    import cv2
    import cv2.aruco as aruco
    import numpy as np
except ImportError:
    print("Error: OpenCV (cv2) and numpy are required. Install with: sudo apt install python3-opencv python3-numpy")
    sys.exit(1)

try:
    import cairo
except ImportError:
    cairo = None

DICTIONARIES = {
    "DICT_4X4_50": aruco.DICT_4X4_50,
    "DICT_4X4_100": aruco.DICT_4X4_100,
    "DICT_4X4_250": aruco.DICT_4X4_250,
    "DICT_5X5_50": aruco.DICT_5X5_50,
    "DICT_5X5_100": aruco.DICT_5X5_100,
    "DICT_5X5_250": aruco.DICT_5X5_250,
    "DICT_6X6_50": aruco.DICT_6X6_50,
    "DICT_6X6_100": aruco.DICT_6X6_100,
    "DICT_6X6_250": aruco.DICT_6X6_250,
}

MM_TO_PTS = 72.0 / 25.4  # Cairo uses PostScript points (1/72 inch)

PAGE_SIZES_MM = {
    "a4": (210.0, 297.0),
    "letter": (215.9, 279.4),
}


def draw_ruler(cr, x_mm, y_mm, length_mm, label="Verification Ruler (mm)"):
    """Draw a millimeter ruler with 1mm tick marks and 5mm/10mm callouts."""
    cr.save()
    cr.set_source_rgb(0, 0, 0)
    cr.set_line_width(0.15 * MM_TO_PTS)

    # Main baseline
    cr.move_to(x_mm * MM_TO_PTS, y_mm * MM_TO_PTS)
    cr.line_to((x_mm + length_mm) * MM_TO_PTS, y_mm * MM_TO_PTS)
    cr.stroke()

    # Ticks
    for mm in range(int(length_mm) + 1):
        x = (x_mm + mm) * MM_TO_PTS
        y0 = y_mm * MM_TO_PTS

        if mm % 10 == 0:
            tick_h = 4.0 * MM_TO_PTS
            cr.move_to(x, y0)
            cr.line_to(x, y0 - tick_h)
            cr.stroke()

            # Number label
            cr.set_font_size(2.5 * MM_TO_PTS)
            cr.select_font_face("Sans", cairo.FONT_SLANT_NORMAL, cairo.FONT_WEIGHT_NORMAL)
            text = str(mm)
            extents = cr.text_extents(text)
            cr.move_to(x - extents.width / 2.0, y0 - tick_h - 1.0 * MM_TO_PTS)
            cr.show_text(text)
        elif mm % 5 == 0:
            tick_h = 2.5 * MM_TO_PTS
            cr.move_to(x, y0)
            cr.line_to(x, y0 - tick_h)
            cr.stroke()
        else:
            tick_h = 1.5 * MM_TO_PTS
            cr.move_to(x, y0)
            cr.line_to(x, y0 - tick_h)
            cr.stroke()

    # Caption
    cr.set_font_size(2.2 * MM_TO_PTS)
    cr.move_to(x_mm * MM_TO_PTS, (y_mm + 3.0) * MM_TO_PTS)
    cr.show_text(f"{label} - Verify scale with calipers before use")
    cr.restore()


def generate_board_cairo(output_path: Path, squares_x: int, squares_y: int,
                         square_mm: float, marker_mm: float, dict_name: str,
                         page_format: str, margin_mm: float, dpi: int):
    """Generate high-precision PDF or SVG board using Cairo."""
    board_w_mm = squares_x * square_mm
    board_h_mm = squares_y * square_mm

    if page_format in PAGE_SIZES_MM:
        page_w_mm, page_h_mm = PAGE_SIZES_MM[page_format]
    else:  # 'fit'
        page_w_mm = board_w_mm + 2 * margin_mm
        page_h_mm = board_h_mm + 2 * margin_mm + 25.0  # extra for header & ruler

    page_w_pts = page_w_mm * MM_TO_PTS
    page_h_pts = page_h_mm * MM_TO_PTS

    # Create Cairo Surface
    ext = output_path.suffix.lower()
    if ext == ".pdf":
        surface = cairo.PDFSurface(str(output_path), page_w_pts, page_h_pts)
    elif ext == ".svg":
        surface = cairo.SVGSurface(str(output_path), page_w_pts, page_h_pts)
    elif ext == ".png":
        px_w = int(page_w_mm / 25.4 * dpi)
        px_h = int(page_h_mm / 25.4 * dpi)
        surface = cairo.ImageSurface(cairo.FORMAT_ARGB32, px_w, px_h)
    else:
        raise ValueError(f"Unsupported format: {ext}")

    cr = cairo.Context(surface)
    if ext == ".png":
        # Scale context for high-DPI rasterization
        scale = (dpi / 25.4) / MM_TO_PTS
        cr.scale(scale, scale)

    # White page background
    cr.set_source_rgb(1.0, 1.0, 1.0)
    cr.paint()

    # Center board on page
    board_x_mm = (page_w_mm - board_w_mm) / 2.0
    board_y_mm = (page_h_mm - board_h_mm) / 2.0 - 5.0

    # Render ChArUco image at high resolution (e.g. 1200 DPI equivalent)
    dpm = 50  # 50 pixels per mm = 1270 DPI
    render_w = int(board_w_mm * dpm)
    render_h = int(board_h_mm * dpm)

    dict_id = DICTIONARIES[dict_name]
    aruco_dict = aruco.getPredefinedDictionary(dict_id)
    board = aruco.CharucoBoard_create(squares_x, squares_y, square_mm / 1000.0, marker_mm / 1000.0, aruco_dict)
    board_img = board.draw((render_w, render_h), marginSize=0, borderBits=1)

    # Convert board grayscale image to Cairo RGB surface
    argb = np.zeros((render_h, render_w, 4), dtype=np.uint8)
    argb[:, :, 0] = board_img  # Blue
    argb[:, :, 1] = board_img  # Green
    argb[:, :, 2] = board_img  # Red
    argb[:, :, 3] = 255        # Alpha

    img_surface = cairo.ImageSurface.create_for_data(
        argb, cairo.FORMAT_ARGB32, render_w, render_h, render_w * 4
    )

    # Draw ChArUco board onto page
    cr.save()
    cr.translate(board_x_mm * MM_TO_PTS, board_y_mm * MM_TO_PTS)
    cr.scale((board_w_mm * MM_TO_PTS) / render_w, (board_h_mm * MM_TO_PTS) / render_h)
    cr.set_source_surface(img_surface, 0, 0)
    cr.paint()
    cr.restore()

    # Draw a thin black border around the entire board
    cr.save()
    cr.set_source_rgb(0, 0, 0)
    cr.set_line_width(0.2 * MM_TO_PTS)
    cr.rectangle(board_x_mm * MM_TO_PTS, board_y_mm * MM_TO_PTS,
                 board_w_mm * MM_TO_PTS, board_h_mm * MM_TO_PTS)
    cr.stroke()
    cr.restore()

    # Draw Header text
    cr.save()
    cr.set_source_rgb(0, 0, 0)
    cr.select_font_face("Sans", cairo.FONT_SLANT_NORMAL, cairo.FONT_WEIGHT_BOLD)
    cr.set_font_size(3.5 * MM_TO_PTS)
    cr.move_to(board_x_mm * MM_TO_PTS, (board_y_mm - 10.0) * MM_TO_PTS)
    cr.show_text("dVRK ChArUco Calibration Target")

    cr.select_font_face("Sans", cairo.FONT_SLANT_NORMAL, cairo.FONT_WEIGHT_NORMAL)
    cr.set_font_size(2.5 * MM_TO_PTS)
    info_text = (
        f"Grid: {squares_x}x{squares_y} squares | Square: {square_mm:.2f} mm | "
        f"Marker: {marker_mm:.2f} mm | Dict: {dict_name} | Board: {board_w_mm:.1f}x{board_h_mm:.1f} mm"
    )
    cr.move_to(board_x_mm * MM_TO_PTS, (board_y_mm - 5.0) * MM_TO_PTS)
    cr.show_text(info_text)

    # Warning instructions
    cr.set_font_size(2.2 * MM_TO_PTS)
    cr.set_source_rgb(0.3, 0.3, 0.3)
    cr.move_to(board_x_mm * MM_TO_PTS, (board_y_mm - 1.5) * MM_TO_PTS)
    cr.show_text("IMPORTANT: Print at 100% scale (Do not fit/scale to page). Mount flat on rigid matte plate.")
    cr.restore()

    # Draw Millimeter Verification Ruler
    ruler_len = min(board_w_mm, 50.0)
    if ruler_len < 30.0:
        ruler_len = board_w_mm
    ruler_y = board_y_mm + board_h_mm + 12.0
    draw_ruler(cr, board_x_mm, ruler_y, ruler_len)

    if ext == ".png":
        surface.write_to_png(str(output_path))
    else:
        surface.finish()

    print(f"Generated {output_path} successfully ({board_w_mm:.1f}x{board_h_mm:.1f} mm target).")


def main():
    parser = argparse.ArgumentParser(allow_abbrev=False, description="Generate millimeter-accurate ChArUco calibration board.")
    parser.add_argument("-x", "--squares-x", type=int, default=8, help="Number of squares along X (default: 8)")
    parser.add_argument("-y", "--squares-y", type=int, default=6, help="Number of squares along Y (default: 6)")
    parser.add_argument("-s", "--square-length", type=float, default=5.0, help="Square side length in mm (default: 5.0)")
    parser.add_argument("-m", "--marker-length", type=float, default=3.5, help="Marker side length in mm (default: 3.5)")
    parser.add_argument("-d", "--dict", choices=list(DICTIONARIES.keys()), default="DICT_4X4_50",
                        help="ArUco dictionary (default: DICT_4X4_50)")
    parser.add_argument("-p", "--page-size", choices=["a4", "letter", "fit"], default="a4",
                        help="Page format (default: a4)")
    parser.add_argument("--margin", type=float, default=15.0, help="Page margins in mm for fit mode (default: 15.0)")
    parser.add_argument("--dpi", type=int, default=300, help="DPI for raster PNG output (default: 300)")
    parser.add_argument("-o", "--output", type=str, default="charuco_board.pdf",
                        help="Output file path (.pdf, .svg, or .png). Default: charuco_board.pdf")

    args = parser.parse_args()

    if args.marker_length >= args.square_length:
        parser.error(f"Marker length ({args.marker_length} mm) must be strictly less than square length ({args.square_length} mm)")

    if cairo is None:
        print("Error: pycairo is required. Install with: sudo apt install python3-cairo")
        sys.exit(1)

    out_path = Path(args.output).resolve()
    out_path.parent.mkdir(parents=True, exist_ok=True)

    generate_board_cairo(
        output_path=out_path,
        squares_x=args.squares_x,
        squares_y=args.squares_y,
        square_mm=args.square_length,
        marker_mm=args.marker_length,
        dict_name=args.dict,
        page_format=args.page_size,
        margin_mm=args.margin,
        dpi=args.dpi,
    )

    # Save matching JSON configuration file
    json_path = out_path.with_suffix(".json")
    save_board_json(
        json_path=json_path,
        squares_x=args.squares_x,
        squares_y=args.squares_y,
        square_mm=args.square_length,
        marker_mm=args.marker_length,
        dict_name=args.dict,
    )


def save_board_json(json_path: Path, squares_x: int, squares_y: int, square_mm: float, marker_mm: float, dict_name: str):
    import json
    data = {
        "type": "dvrk_calibration:charuco_board@1.0.0",
        "squares_x": squares_x,
        "squares_y": squares_y,
        "square_length_mm": square_mm,
        "marker_length_mm": marker_mm,
        "dictionary": dict_name,
        "description": f"ChArUco {squares_x}x{squares_y} calibration board ({square_mm}mm squares, {marker_mm}mm markers, {dict_name})"
    }
    with open(json_path, "w") as f:
        json.dump(data, f, indent=2)
    print(f"Generated board config JSON: {json_path}")


if __name__ == "__main__":
    main()

