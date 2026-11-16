import 'dart:io';
import 'dart:typed_data';

import 'package:flutter/material.dart';
import 'package:flutter_test/flutter_test.dart';
import 'package:vslam/dataset/session_files.dart';
import 'package:vslam/dataset/session_reader.dart';
import 'package:vslam/dataset/session_store.dart';
import 'package:vslam/estimation/session_baselines.dart';
import 'package:vslam/mapping/marker_map.dart';
import 'package:vslam/screens/session_detail_screen.dart';
import 'package:vslam/vision/aruco_detector.dart';
import 'package:vslam/vision/camera_model.dart';
import 'package:vslam/vision/session_detection.dart';

import 'sessions_ui_test.dart' show act, waitFor;
import 'support/simulator.dart';

/// Counts calls; finds nothing. The simulated frames carry no image, so
/// the detector should never be asked.
class CountingDetector implements MarkerDetector {
  int calls = 0;

  @override
  Future<FrameDetections> detect(Uint8List jpeg, CameraIntrinsics intrinsics, double sideM) async {
    calls++;
    return const FrameDetections([], 0);
  }
}

void main() {
  TestWidgetsFlutterBinding.ensureInitialized();
  late Directory tmp;
  setUp(() => tmp = Directory.systemTemp.createTempSync('vslam_wp4_ui_'));
  tearDown(() => tmp.deleteSync(recursive: true));

  testWidgets('session screen: Build map writes the results, Save map writes maps/<room>.json', (tester) async {
    final store = SessionStore(root: tmp, tempDir: tmp);
    final detector = CountingDetector();
    final dir = (await tester.runAsync(
      () => RoomSimulation(const SimConfig(laps: 1)).writeSession(store.ensureSessionsDir(), label: 'room walk'),
    ))!;
    await act(
      tester,
      () => tester.pumpWidget(MaterialApp(home: SessionDetailScreen(store: store, directory: dir, detector: detector))),
    );
    await waitFor(tester, find.text('Build map'));
    expect(find.text('Save map'), findsNothing);

    await tester.ensureVisible(find.text('Build map'));
    await tester.pumpAndSettle();
    await act(tester, () => tester.tap(find.text('Build map')));
    await waitFor(tester, find.text('Keyframes'));
    expect(detector.calls, 0); // detections.csv existed
    expect(find.text('IMU rest'), findsOneWidget);
    expect(find.textContaining('still: 0.0–2.0 s after the first keyframe'), findsOneWidget);
    expect(find.textContaining('ID 0: ('), findsOneWidget);
    for (final f in [trajArkitFile, trajImuFile, mapFile, baselinesSummaryFile]) {
      expect(File('${dir.path}/${SessionFiles.results}/$f').existsSync(), isTrue, reason: f);
    }

    // The result card pushes the buttons below the lazily built part of the
    // list; scroll until the button exists.
    await tester.scrollUntilVisible(find.text('Save map'), 200, scrollable: find.byType(Scrollable).first);
    await tester.pumpAndSettle();
    await tester.tap(find.text('Save map'));
    await tester.pumpAndSettle();
    await tester.enterText(find.byType(TextField), 'Lab 2');
    await act(tester, () => tester.tap(find.widgetWithText(FilledButton, 'Save')));
    await waitFor(tester, find.textContaining('maps/lab-2.json'));
    final saved = (await tester.runAsync(() => MapStore(tmp).load('lab-2')))!;
    expect(saved.ids, hasLength(8));
  });

  test('Build map runs marker detection first when it has not been run', () async {
    final dir = await RoomSimulation(const SimConfig(laps: 1)).writeSession(tmp);
    final csv = File('${dir.path}/${SessionFiles.results}/$detectionsFile');
    await csv.delete();
    final detector = CountingDetector();
    final run = await processSessionBaselines(await SessionReader.read(dir), detector: detector);
    expect(csv.existsSync(), isTrue);
    expect(File('${dir.path}/${SessionFiles.results}/$detectionsSummaryFile').existsSync(), isTrue);
    expect(detector.calls, 0); // no frame has an image
    expect(run.map.markers, isEmpty);
    expect(run.keyframes, isNotEmpty);
  });
}
