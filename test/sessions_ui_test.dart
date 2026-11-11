import 'dart:io';

import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:flutter_test/flutter_test.dart';
import 'package:vslam/dataset/session_reader.dart';
import 'package:vslam/dataset/session_store.dart';
import 'package:vslam/dataset/session_writer.dart';
import 'package:vslam/main.dart';
import 'package:vslam/screens/sessions_screen.dart';
import 'package:vslam/slam/slam_service.dart';

import 'session_test.dart' show FakeImu;
import 'support/synthetic_session.dart';

/// Widget tests run in fake time, and real file I/O started there never
/// completes. [act] runs a UI step in the real zone (so the I/O it starts,
/// e.g. in a new screen's initState, can finish), waits for that I/O, then
/// pumps the resulting frames.
Future<void> act(WidgetTester tester, Future<void> Function() step) async {
  await tester.runAsync(() async {
    await step();
    for (var i = 0; i < 6; i++) {
      await Future<void>.delayed(const Duration(milliseconds: 30));
      await tester.pump();
    }
  });
  await tester.pump();
}

/// Pumps (letting real I/O run in between) until [finder] matches, e.g.
/// while a screen shows a spinner for data it is reading from disk.
Future<void> waitFor(WidgetTester tester, Finder finder) async {
  for (var i = 0; i < 100 && finder.evaluate().isEmpty; i++) {
    await tester.runAsync(() => Future<void>.delayed(const Duration(milliseconds: 20)));
    await tester.pump(const Duration(milliseconds: 50));
  }
  expect(finder, findsWidgets);
}

void main() {
  TestWidgetsFlutterBinding.ensureInitialized();
  late Directory tmp;
  setUp(() {
    tmp = Directory.systemTemp.createTempSync('vslam_ui_');
    TestDefaultBinaryMessengerBinding.instance.defaultBinaryMessenger.setMockMethodCallHandler(
      const MethodChannel('vslam/ar'),
      (call) async => call.method == 'checkAvailability' ? 'unsupported' : null,
    );
  });
  tearDown(() {
    TestDefaultBinaryMessengerBinding.instance.defaultBinaryMessenger.setMockMethodCallHandler(
      const MethodChannel('vslam/ar'),
      null,
    );
    tmp.deleteSync(recursive: true);
  });

  testWidgets('record, then open, rename and delete the session', (tester) async {
    final store = SessionStore(root: tmp, tempDir: tmp);
    final imu = FakeImu();
    await act(tester, () => tester.pumpWidget(VslamApp(imu: imu.service(), slam: SlamService(), store: store)));

    // Settings for the next recording.
    await tester.tap(find.byIcon(Icons.tune));
    await tester.pumpAndSettle();
    await tester.enterText(find.widgetWithText(TextField, 'Label (folder name)'), 'Hall loop');
    await tester.enterText(find.widgetWithText(TextField, 'Marker side (cm)'), '15.2');
    await act(tester, () => tester.tap(find.text('Save settings')));
    await tester.pumpAndSettle();
    expect(find.textContaining('Hall loop · Simple path · marker 15.2 cm'), findsOneWidget);

    // Record.
    await act(tester, () async {
      await tester.tap(find.text('Start'));
      await tester.pump();
      imu.still(30);
    });
    expect(find.textContaining('Recording "Hall loop"'), findsOneWidget);
    await act(tester, () => tester.tap(find.text('Stop')));
    await waitFor(tester, find.textContaining('Saved '));
    expect(find.textContaining('Saved '), findsOneWidget);
    expect(find.textContaining('60 IMU samples'), findsOneWidget);
    ScaffoldMessenger.of(tester.element(find.text('Start'))).removeCurrentSnackBar();
    await tester.pumpAndSettle();

    // Sessions list → detail.
    await act(tester, () => tester.tap(find.byTooltip('Sessions')));
    await waitFor(tester, find.text('Hall loop'));
    await act(tester, () => tester.tap(find.text('Hall loop')));
    await waitFor(tester, find.text('Accelerometer'));
    expect(find.textContaining('30 at'), findsNWidgets(2)); // accelerometer and gyroscope counts
    expect(find.textContaining('side 15.2 cm'), findsOneWidget);

    // Rename.
    await tester.tap(find.text('Rename'));
    await tester.pumpAndSettle();
    await tester.enterText(find.byType(TextField), 'Hall loop 2');
    await act(tester, () => tester.tap(find.widgetWithText(FilledButton, 'Rename')));
    await waitFor(tester, find.textContaining('_hall-loop-2'));
    final entries = await tester.runAsync(store.list);
    expect(entries!.single.name, endsWith('_hall-loop-2'));
    expect(find.text('Hall loop 2'), findsOneWidget);

    // Delete (with confirmation) returns to an empty list.
    await tester.ensureVisible(find.text('Delete'));
    await tester.pumpAndSettle();
    // Opened in the real zone so the code after the dialog (file deletion,
    // then reloading the list) runs where file I/O can complete.
    await act(tester, () => tester.tap(find.text('Delete')));
    await tester.pumpAndSettle();
    expect(find.text('Delete session?'), findsOneWidget);
    await act(tester, () => tester.tap(find.widgetWithText(FilledButton, 'Delete')));
    await waitFor(tester, find.textContaining('No sessions yet'));
    expect(find.textContaining('No sessions yet'), findsOneWidget);
    expect((await tester.runAsync(store.list))!, isEmpty);
  });

  testWidgets('sessions list shows a synthetic session with its counts', (tester) async {
    final store = SessionStore(root: tmp, tempDir: tmp);
    final s = SyntheticSession();
    await tester.runAsync(() async {
      final writer = SessionWriter.create(store.ensureSessionsDir(), SyntheticSession.meta());
      var frame = 0;
      for (final e in s.frames) {
        writer.addFrame(e, null);
        frame++;
      }
      s.poses.forEach(writer.addPose);
      s.imu.forEach(writer.addImu);
      await writer.close();
      expect(frame, s.frames.length);
    });
    await act(tester, () => tester.pumpWidget(MaterialApp(home: SessionsScreen(store: store))));
    await waitFor(tester, find.text('Closed loop #1'));
    expect(
      find.textContaining('Closed loop · 0:02 · ${s.frames.length} frames · ${s.imu.length} IMU samples'),
      findsOneWidget,
    );

    await act(tester, () => tester.tap(find.text('Closed loop #1')));
    await waitFor(tester, find.text('Accelerometer'));
    final data = await tester.runAsync(
      () => SessionReader.read(Directory('${store.sessionsDir.path}/2026-11-11_14-03-22_closed-loop-1')),
    );
    expect(find.textContaining('${data!.frames.length} at 10.0 Hz'), findsOneWidget);
    expect(find.textContaining('iPhone15,2, iOS 26.1'), findsOneWidget);
  });
}
