import 'dart:convert';
import 'dart:typed_data';

import 'package:dartcv4/dartcv.dart' as cv;

/// Cells of ArUco marker [id] of `DICT_4X4_50` including the one-cell black
/// border: a 6 × 6 grid, row 0 at the top of the printed marker, `true` for
/// black. Taken from OpenCV (`cv::aruco::generateImageMarker` at one pixel
/// per cell), so the printed pattern is exactly what the detector decodes.
List<List<bool>> markerCells(int id) {
  const cells = 6; // 4 data bits + 1 border bit on each side
  final img = cv.arucoGenerateImageMarker(cv.PredefinedDictionaryType.DICT_4X4_50, id, cells, 1);
  try {
    return [
      for (var r = 0; r < cells; r++) [for (var c = 0; c < cells; c++) img.atU8(r, i1: c) < 128],
    ];
  } finally {
    img.dispose();
  }
}

/// A print-ready PDF (A4 portrait, one marker per page) of `DICT_4X4_50`
/// markers [ids] with a black square of [sideMm] millimetres, an ID label,
/// a "top" mark, and a 100 mm ruler line.
///
/// Printer scaling does not matter: the app needs the printed side, so the
/// sheet asks the person to measure the black square and enter that value.
/// The ruler shows the scale the printer applied (a 100 mm line that
/// measures 82 mm means the marker came out at 82 % too).
///
/// The PDF is written by hand (PDF 1.4, vector rectangles, the standard
/// Helvetica font, which needs no embedding) to avoid another dependency.
Uint8List markerSheetPdf({required List<int> ids, double sideMm = 150}) {
  const mm = 72 / 25.4; // PDF points per millimetre
  const pageW = 210.0; // A4, mm
  const pageH = 297.0;
  String f(double v) => v.toStringAsFixed(3);

  final pages = <String>[];
  for (final id in ids) {
    final cells = markerCells(id);
    final cell = sideMm / cells.length;
    final left = (pageW - sideMm) / 2;
    final top = 45.0; // mm from the top edge to the top of the black square
    // One solid black square, then the white cells on top: adjacent black
    // cells drawn separately can show hairline seams in viewers and prints.
    // PDF y runs up from the bottom of the page.
    final b = StringBuffer()
      ..writeln('0 g')
      ..writeln('${f(left * mm)} ${f((pageH - top - sideMm) * mm)} ${f(sideMm * mm)} ${f(sideMm * mm)} re f')
      ..writeln('1 g');
    for (var r = 0; r < cells.length; r++) {
      for (var c = 0; c < cells.length; c++) {
        if (cells[r][c]) continue;
        final x = left + c * cell;
        final y = pageH - top - (r + 1) * cell;
        b.writeln('${f(x * mm)} ${f(y * mm)} ${f(cell * mm)} ${f(cell * mm)} re f');
      }
    }
    b.writeln('0 g');
    void text(double xMm, double yFromTopMm, double size, String s) {
      final escaped = s.replaceAll(r'\', r'\\').replaceAll('(', r'\(').replaceAll(')', r'\)');
      b.writeln('BT /F1 ${f(size)} Tf ${f(xMm * mm)} ${f((pageH - yFromTopMm) * mm)} Td ($escaped) Tj ET');
    }

    text(left, 22, 28, 'ID $id');
    text(left + 45, 20, 10, 'ArUco DICT_4X4_50');
    text(left + 45, 25, 10, 'Designed side ${sideMm.toStringAsFixed(1)} mm (black square, outer edge)');
    text(pageW / 2 - 6, top - 3, 9, 'TOP');

    final below = top + sideMm;
    final lines = [
      'After printing, measure the black square from its left outer edge to its right outer edge',
      'and enter it as "Marker side (cm)" in the app\'s recording settings, e.g. 12.3 cm.',
      'The printed size is what counts, so printer scaling does not matter. Keep the white margin.',
      'Mount the page flat (glue or tape it to card): a curled page bends the corners.',
    ];
    for (var i = 0; i < lines.length; i++) {
      text(left - 10, below + 14 + i * 5.5, 9, lines[i]);
    }

    // 100 mm ruler with 10 mm ticks.
    final rulerY = pageH - (below + 48);
    final x0 = (pageW - 100) / 2;
    b
      ..writeln('0.3 w')
      ..writeln('${f(x0 * mm)} ${f(rulerY * mm)} m ${f((x0 + 100) * mm)} ${f(rulerY * mm)} l S');
    for (var i = 0; i <= 10; i++) {
      final x = x0 + i * 10;
      final h = i == 0 || i == 10 ? 4.0 : (i == 5 ? 3.0 : 2.0);
      b.writeln('${f(x * mm)} ${f(rulerY * mm)} m ${f(x * mm)} ${f((rulerY + h) * mm)} l S');
    }
    text(x0, below + 48 + 6, 9, 'This line is 100 mm at true scale. If it measures less, the printer scaled the page;');
    text(x0, below + 48 + 11, 9, 'the marker shrank by the same factor. Enter the measured marker side either way.');
    pages.add(b.toString());
  }

  // Objects: 1 catalog, 2 page tree, 3 font, then (page, content) pairs.
  final objects = <String>[
    '<< /Type /Catalog /Pages 2 0 R >>',
    '<< /Type /Pages /Kids [${[for (var i = 0; i < pages.length; i++) '${4 + 2 * i} 0 R'].join(' ')}] '
        '/Count ${pages.length} >>',
    '<< /Type /Font /Subtype /Type1 /BaseFont /Helvetica /Encoding /WinAnsiEncoding >>',
  ];
  for (var i = 0; i < pages.length; i++) {
    objects.add('<< /Type /Page /Parent 2 0 R /MediaBox [0 0 ${f(pageW * mm)} ${f(pageH * mm)}] '
        '/Resources << /Font << /F1 3 0 R >> >> /Contents ${5 + 2 * i} 0 R >>');
    final content = pages[i];
    objects.add('<< /Length ${latin1.encode(content).length} >>\nstream\n${content}endstream');
  }

  final out = StringBuffer('%PDF-1.4\n');
  final offsets = <int>[];
  for (var i = 0; i < objects.length; i++) {
    offsets.add(latin1.encode(out.toString()).length);
    out.write('${i + 1} 0 obj\n${objects[i]}\nendobj\n');
  }
  final xref = latin1.encode(out.toString()).length;
  out
    ..write('xref\n0 ${objects.length + 1}\n')
    ..write('0000000000 65535 f \n');
  for (final o in offsets) {
    out.write('${o.toString().padLeft(10, '0')} 00000 n \n');
  }
  out.write('trailer\n<< /Size ${objects.length + 1} /Root 1 0 R >>\nstartxref\n$xref\n%%EOF\n');
  return latin1.encode(out.toString());
}
