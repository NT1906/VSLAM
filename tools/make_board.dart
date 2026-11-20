// Writes the camera-calibration chessboard as an SVG sized in millimetres,
// so it prints at true scale (print at 100 % / "actual size", no fit-to-page).
//
//   dart run tools/make_board.dart [out.svg]
//
// Board: 10 × 7 squares of 25 mm → 9 × 6 inner corners (what OpenCV's
// findChessboardCorners looks for), 250 × 175 mm, fits A4 / Letter landscape.
// A 100 mm ruler line is printed below the board: measure it after printing,
// and measure one square; enter the square size in the calibration screen.
import 'dart:io';

import 'package:vslam/calibration/board.dart';

void main(List<String> args) {
  final out = File(args.isEmpty ? 'calibration_board.svg' : args.first);
  out.writeAsStringSync(boardSvg(CalibrationBoard.standard));
  stdout.writeln('Wrote ${out.path}');
}
