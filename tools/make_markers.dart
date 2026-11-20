// Writes the ArUco marker sheet: DICT_4X4_50, IDs 0-9, one per A4 page,
// 150 mm black square, with a 100 mm ruler line.
//
//   dart run tools/make_markers.dart [out.pdf] [side_mm]
//
// The marker cells come from OpenCV (cv::aruco::generateImageMarker), so the
// printed pattern is exactly what the detector decodes. Print at any
// scaling, then measure the printed black square and enter that side in the
// app's recording settings (the sheet says so on every page).
import 'dart:io';

import 'package:vslam/vision/marker_sheet.dart';

void main(List<String> args) {
  final out = File(args.isEmpty ? 'markers_4x4_50_ids0-9.pdf' : args.first);
  final side = args.length > 1 ? double.parse(args[1]) : 150.0;
  out.writeAsBytesSync(markerSheetPdf(ids: List.generate(10, (i) => i), sideMm: side));
  stdout.writeln('Wrote ${out.path} (IDs 0-9, ${side.toStringAsFixed(1)} mm)');
}
