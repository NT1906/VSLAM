import 'package:vector_math/vector_math_64.dart';

/// A chessboard for camera calibration.
///
/// [cols] × [rows] are the *inner* corners (OpenCV's `patternSize`), so the
/// board has (cols + 1) × (rows + 1) squares of side [squareM] metres.
class CalibrationBoard {
  const CalibrationBoard({required this.cols, required this.rows, required this.squareM});

  /// 9 × 6 inner corners, 25 mm squares: 250 × 175 mm, A4 / Letter landscape.
  static const standard = CalibrationBoard(cols: 9, rows: 6, squareM: 0.025);

  final int cols;
  final int rows;
  final double squareM;

  /// Board-frame corner positions (z = 0), row by row, in the order OpenCV
  /// returns detected corners for an unrotated board.
  List<Vector3> objectPoints() => [
        for (var r = 0; r < rows; r++)
          for (var c = 0; c < cols; c++) Vector3(c * squareM, r * squareM, 0),
      ];

  CalibrationBoard withSquare(double squareM) => CalibrationBoard(cols: cols, rows: rows, squareM: squareM);

  Map<String, dynamic> toJson() => {'cols': cols, 'rows': rows, 'square_m': squareM};

  factory CalibrationBoard.fromJson(Map<String, dynamic> j) => CalibrationBoard(
        cols: (j['cols'] as num).toInt(),
        rows: (j['rows'] as num).toInt(),
        squareM: (j['square_m'] as num).toDouble(),
      );
}

/// SVG of [board] in millimetres with a 100 mm ruler line, for printing at
/// actual size.
String boardSvg(CalibrationBoard board) {
  final s = board.squareM * 1000;
  final nx = board.cols + 1;
  final ny = board.rows + 1;
  const margin = 15.0;
  final w = nx * s + 2 * margin;
  final h = ny * s + 2 * margin + 20;
  final b = StringBuffer()
    ..writeln('<svg xmlns="http://www.w3.org/2000/svg" width="${w}mm" height="${h}mm" viewBox="0 0 $w $h">')
    ..writeln('<rect width="$w" height="$h" fill="white"/>');
  for (var y = 0; y < ny; y++) {
    for (var x = 0; x < nx; x++) {
      if ((x + y).isEven) {
        b.writeln('<rect x="${margin + x * s}" y="${margin + y * s}" width="$s" height="$s" fill="black"/>');
      }
    }
  }
  final ry = margin + ny * s + 10;
  b
    ..writeln('<line x1="$margin" y1="$ry" x2="${margin + 100}" y2="$ry" stroke="black" stroke-width="0.3"/>')
    ..writeln('<line x1="$margin" y1="${ry - 2}" x2="$margin" y2="${ry + 2}" stroke="black" stroke-width="0.3"/>')
    ..writeln('<line x1="${margin + 100}" y1="${ry - 2}" x2="${margin + 100}" y2="${ry + 2}" stroke="black" stroke-width="0.3"/>')
    ..writeln('<text x="${margin + 104}" y="${ry + 1.2}" font-size="3.5" font-family="Helvetica">'
        '100 mm. Chessboard ${board.cols}×${board.rows} inner corners, ${s.toStringAsFixed(1)} mm squares. '
        'Print at actual size.</text>')
    ..writeln('</svg>');
  return b.toString();
}
