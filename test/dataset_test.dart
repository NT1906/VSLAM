import 'dart:async';
import 'dart:convert';
import 'dart:io';

import 'package:archive/archive.dart';
import 'package:flutter/services.dart';
import 'package:flutter_test/flutter_test.dart';
import 'package:vector_math/vector_math_64.dart';
import 'package:vslam/dataset/replay_source.dart';
import 'package:vslam/dataset/sensor_event.dart';
import 'package:vslam/dataset/session_files.dart';
import 'package:vslam/dataset/session_meta.dart';
import 'package:vslam/dataset/session_reader.dart';
import 'package:vslam/dataset/session_recorder.dart';
import 'package:vslam/dataset/session_store.dart';
import 'package:vslam/dataset/session_writer.dart';
import 'package:vslam/services/imu_service.dart';
import 'package:vslam/services/session_controller.dart';
import 'package:vslam/slam/ar_frame.dart';
import 'package:vslam/slam/ar_pose.dart';
import 'package:vslam/slam/slam_service.dart';

import 'support/synthetic_session.dart';

/// Writes [s] through a [SessionWriter] in event order.
Future<SessionMeta> writeSession(SessionWriter writer, SyntheticSession s) async {
  var frame = 0;
  for (final e in s.events) {
    switch (e) {
      case FrameEvent():
        writer.addFrame(e, s.jpegs[frame++]);
      case PoseEvent():
        writer.addPose(e);
      case ImuEvent():
        writer.addImu(e);
    }
  }
  return writer.close();
}

void expectSameEvent(SensorEvent a, SensorEvent b) {
  expect(a.runtimeType, b.runtimeType);
  expect(a.tUs, b.tUs);
  switch ((a, b)) {
    case (ImuEvent a, ImuEvent b):
      expect(a.sensor, b.sensor);
      expect(a.value, b.value);
    case (PoseEvent a, PoseEvent b):
      expect(a.state, b.state);
      expect(a.translation, b.translation);
      expect(a.rotation?.storage, b.rotation?.storage);
    case (FrameEvent a, FrameEvent b):
      expect(a.state, b.state);
      expect(a.translation, b.translation);
      expect(a.rotation?.storage, b.rotation?.storage);
      expect(a.intrinsics.toJson(), b.intrinsics.toJson());
    default:
      fail('different event types');
  }
}

void main() {
  late Directory tmp;
  setUp(() => tmp = Directory.systemTemp.createTempSync('vslam_dataset_'));
  tearDown(() => tmp.deleteSync(recursive: true));

  group('SessionFiles', () {
    test('folder name is <date>_<time>_<slug>', () {
      expect(
        SessionFiles.folderName(DateTime(2026, 1, 2, 3, 4, 5), 'Closed loop #1'),
        '2026-01-02_03-04-05_closed-loop-1',
      );
      expect(SessionFiles.slug('  ///  '), 'run');
      expect(SessionFiles.folderPrefix('2026-01-02_03-04-05_closed-loop-1'), '2026-01-02_03-04-05');
      expect(SessionFiles.frameFile(7), 'frames/000007.jpg');
    });
  });

  group('write, read back, replay', () {
    test('every field survives the round trip', () async {
      final s = SyntheticSession();
      final writer = SessionWriter.create(tmp, SyntheticSession.meta());
      final written = await writeSession(writer, s);
      expect(writer.directory.path, endsWith('2026-11-11_14-03-22_closed-loop-1'));

      final data = await SessionReader.read(writer.directory);

      // meta.json
      final m = data.meta;
      expect(m.schemaVersion, SessionMeta.currentSchemaVersion);
      expect(m.settings.toJson(), SyntheticSession.meta().settings.toJson());
      expect(m.device.toJson(), SyntheticSession.meta().device.toJson());
      expect(m.created, DateTime(2026, 11, 11, 14, 3, 22));
      expect(m.requestedFrameHz, 10);
      expect(m.requestedImuHz, 100);
      expect((m.imageWidth, m.imageHeight, m.sourceWidth, m.sourceHeight), (960, 720, 1920, 1440));
      expect(m.jpegQuality, 0.95);
      expect(m.startTUs, s.events.first.tUs);
      expect(m.endTUs, s.events.map((e) => e.tUs).reduce((a, b) => a > b ? a : b));
      expect(m.counts.toJson(), written.counts.toJson());
      expect(m.counts.frames, s.frames.length);
      expect(m.counts.framesWithoutImage, s.jpegs.where((j) => j == null).length);
      expect(m.counts.frameGaps, 0);
      expect(m.counts.poses, s.poses.length);
      expect(m.counts.trackedPoses, s.poses.where((p) => p.tracking).length);
      expect(m.counts.accelerometer, s.imu.where((e) => e.sensor == ImuSensor.accelerometer).length);
      expect(m.counts.gyroscope, s.imu.where((e) => e.sensor == ImuSensor.gyroscope).length);

      // frames.csv and the JPEG files
      expect(data.frames.length, s.frames.length);
      for (final (i, f) in s.frames.indexed) {
        final r = data.frames[i];
        expect(r.index, i + 1);
        expect(r.file, s.jpegs[i] == null ? '' : SessionFiles.frameFile(i + 1));
        expectSameEvent(r.event, f);
        expect(await r.event.loadJpeg(), s.jpegs[i]);
      }
      // arkit_poses.csv
      expect(data.poses.length, s.poses.length);
      for (final (i, p) in s.poses.indexed) {
        expectSameEvent(data.poses[i], p);
      }
      // imu.csv
      expect(data.imu.length, s.imu.length);
      for (final (i, e) in s.imu.indexed) {
        expectSameEvent(data.imu[i], e);
      }
      expect(Directory('${writer.directory.path}/results').existsSync(), isTrue);
    });

    test('replay yields the original event order and counts', () async {
      final s = SyntheticSession(seed: 4);
      final writer = SessionWriter.create(tmp, SyntheticSession.meta());
      await writeSession(writer, s);
      final replayed = await ReplaySource(await SessionReader.read(writer.directory)).events.toList();
      expect(replayed.length, s.events.length);
      for (var i = 0; i < replayed.length; i++) {
        expectSameEvent(replayed[i], s.events[i]);
      }
    });

    test('measured rates match the scripted ones', () async {
      final s = SyntheticSession(seconds: 4);
      final writer = SessionWriter.create(tmp, SyntheticSession.meta());
      await writeSession(writer, s);
      final data = await SessionReader.read(writer.directory);
      expect(data.frameHz, closeTo(10, 1e-9));
      expect(data.poseHz, closeTo(30, 0.01));
      expect(data.imuHz(ImuSensor.accelerometer), closeTo(100, 1e-9));
      expect(data.imuHz(ImuSensor.gyroscope), closeTo(100, 1e-9));
    });

    test('a dropped frame is counted as a gap', () async {
      final s = SyntheticSession(dropFrameIndex: 12);
      final writer = SessionWriter.create(tmp, SyntheticSession.meta());
      final meta = await writeSession(writer, s);
      expect(meta.counts.frameGaps, 1);
    });

    test('a second session started in the same second gets its own folder', () {
      final a = SessionWriter.create(tmp, SyntheticSession.meta());
      final b = SessionWriter.create(tmp, SyntheticSession.meta());
      expect(b.directory.path, '${a.directory.path}_2');
    });

    test('reader rejects a file with the wrong columns', () async {
      final writer = SessionWriter.create(tmp, SyntheticSession.meta());
      await writer.close();
      File('${writer.directory.path}/imu.csv').writeAsStringSync('t_us,sensor,x,y\n1,acc,0,0\n');
      expect(SessionReader.read(writer.directory), throwsFormatException);
    });
  });

  group('SessionRecorder', () {
    test('records a source stream exactly as the writer would', () async {
      final s = SyntheticSession(seed: 9);
      final source = _ListSource();
      final recorder = SessionRecorder(source, SessionWriter.create(tmp, SyntheticSession.meta()));
      for (final e in s.events) {
        source.controller.add(e);
      }
      final meta = await recorder.stop();
      expect(meta.counts.frames, s.frames.length);
      expect(meta.counts.poses, s.poses.length);
      expect(meta.counts.accelerometer + meta.counts.gyroscope, s.imu.length);
      final replayed = await ReplaySource(await SessionReader.read(recorder.writer.directory)).events.toList();
      for (var i = 0; i < replayed.length; i++) {
        expectSameEvent(replayed[i], s.events[i]);
      }
    });
  });

  group('SessionController saves the live streams', () {
    TestWidgetsFlutterBinding.ensureInitialized();
    const channel = MethodChannel('vslam/ar');
    tearDown(
      () => TestDefaultBinaryMessengerBinding.instance.defaultBinaryMessenger.setMockMethodCallHandler(channel, null),
    );

    test('frames, poses and IMU samples reach the session folder', () async {
      TestDefaultBinaryMessengerBinding.instance.defaultBinaryMessenger.setMockMethodCallHandler(
        channel,
        (call) async => switch (call.method) {
          'checkAvailability' => 'supported',
          'hasCameraPermission' => true,
          'deviceInfo' => {'model': 'iPhone15,2', 'os': 'iOS 26.1', 'appVersion': '0.6.0', 'appBuild': '2'},
          _ => null,
        },
      );
      final acc = StreamController<ImuSample>.broadcast();
      final gyro = StreamController<ImuSample>.broadcast();
      final imu = ImuService(accelerometer: () => acc.stream, gyroscope: () => gyro.stream);
      final slam = SlamService();
      await slam.initialize();
      final store = SessionStore(root: tmp, tempDir: tmp);
      final session = SessionController(imu: imu, slam: slam, store: store);
      await session.load();
      expect(session.device.model, 'iPhone15,2');

      session.start();
      const t0 = 1762850000000000;
      for (var i = 0; i < 20; i++) {
        final t = DateTime.fromMicrosecondsSinceEpoch(t0 + i * 10000);
        acc.add(ImuSample(Vector3(0, 9.81, 0), t));
        gyro.add(ImuSample(Vector3(0.01, 0, 0), t));
      }
      for (var i = 0; i < 3; i++) {
        slam.handlePose(
          ArPose(
            state: ArTrackingState.tracking,
            translation: Vector3(i * 0.1, 1.4, 0),
            rotation: Quaternion.identity(),
            time: DateTime.fromMicrosecondsSinceEpoch(t0 + i * 33333),
          ),
        );
        slam.handleFrame(
          ArFrame.fromMap({
            'ts': t0 + i * 100000,
            'w': 4,
            'h': 3,
            'y': Uint8List(12),
            'k': [3.0, 3.0, 1.5, 1.0],
            'state': 'tracking',
            't': [i * 0.1, 1.4, 0],
            'q': [0, 0, 0, 1],
            'jpg': Uint8List.fromList([0xff, 0xd8, i, 0xff, 0xd9]),
            'jpgQ': 0.95,
            'srcW': 8,
            'srcH': 6,
          }, DateTime.now()),
        );
      }
      await pumpEventQueue();
      final meta = await session.stop();
      expect(meta, isNotNull);
      expect(session.lastSaved, isNotNull);

      final data = await SessionReader.read(session.lastSaved!.directory);
      expect(data.meta.device.model, 'iPhone15,2');
      expect(data.frames.length, 3);
      expect(data.meta.counts.framesWithoutImage, 0);
      expect(await data.frames[2].event.loadJpeg(), [0xff, 0xd8, 2, 0xff, 0xd9]);
      expect((data.meta.sourceWidth, data.meta.jpegQuality), (8, 0.95));
      expect(data.poses.length, 3);
      expect(data.poses[1].tUs, t0 + 33333);
      expect(data.count(ImuSensor.accelerometer), 20);
      expect(data.count(ImuSensor.gyroscope), 20);
      expect(data.imu.first.tUs, t0);
      session.dispose();
    });
  });

  group('SessionStore', () {
    test('lists, renames, exports and deletes sessions; keeps settings', () async {
      final store = SessionStore(root: tmp, tempDir: Directory('${tmp.path}/tmp')..createSync());
      final writer = SessionWriter.create(store.ensureSessionsDir(), SyntheticSession.meta());
      await writeSession(writer, SyntheticSession());
      final later = SessionWriter.create(store.sessionsDir, SyntheticSession.meta(created: DateTime(2026, 11, 12)));
      await later.close();

      var entries = await store.list();
      expect(entries.map((e) => e.name), ['2026-11-12_00-00-00_closed-loop-1', '2026-11-11_14-03-22_closed-loop-1']);

      final renamed = await store.rename(writer.directory, 'Loop A');
      expect(renamed.path, endsWith('2026-11-11_14-03-22_loop-a'));
      expect(writer.directory.existsSync(), isFalse);
      expect((await SessionReader.readMeta(renamed)).settings.label, 'Loop A');
      expect((await SessionReader.read(renamed)).frames.length, SyntheticSession().frames.length);

      final zip = await store.exportZip(renamed);
      final archive = ZipDecoder().decodeBytes(zip.readAsBytesSync());
      final names = archive.files.map((f) => f.name).toSet();
      expect(names, containsAll(['2026-11-11_14-03-22_loop-a/meta.json', '2026-11-11_14-03-22_loop-a/imu.csv']));
      final metaEntry = archive.files.firstWhere((f) => f.name.endsWith('meta.json'));
      expect((jsonDecode(utf8.decode(metaEntry.content as List<int>)) as Map)['label'], 'Loop A');

      await store.delete(renamed);
      entries = await store.list();
      expect(entries.length, 1);

      expect((await store.loadSettings()).toJson(), const RecordingSettings().toJson());
      const custom = RecordingSettings(
        label: 'x',
        experiment: ExperimentType.difficult,
        markerSideM: 0.1,
        notes: 'dim',
      );
      await store.saveSettings(custom);
      expect((await store.loadSettings()).toJson(), custom.toJson());
    });

    test('an unreadable session is listed with its error', () async {
      final store = SessionStore(root: tmp, tempDir: tmp);
      Directory('${store.ensureSessionsDir().path}/broken').createSync();
      final entries = await store.list();
      expect(entries.single.meta, isNull);
      expect(entries.single.error, isNotNull);
    });
  });
}

class _ListSource implements SensorSource {
  final controller = StreamController<SensorEvent>(sync: true);

  @override
  Stream<SensorEvent> get events => controller.stream;
}
