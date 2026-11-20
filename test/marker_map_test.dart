import 'dart:io';

import 'package:flutter_test/flutter_test.dart';
import 'package:vector_math/vector_math_64.dart';
import 'package:vslam/core/se3.dart';
import 'package:vslam/mapping/marker_map.dart';

MarkerMap sample() => MarkerMap(
      markers: [
        MapMarker(
          id: 3,
          wTm: Pose3(so3Exp(Vector3(0.1, -1.2, 0.4)), Vector3(2.513, -0.25, 1.47)),
          firstKeyframe: 12,
          firstTUs: 1763300001234567,
          firstDistanceM: 1.234,
          firstViewDeg: 37.5,
          acceptedObservations: 41,
          keyframeObservations: 9,
        ),
        MapMarker(id: 7, wTm: Pose3(so3Exp(Vector3(3.0, 0.2, -0.1)), Vector3(-1, 4.25, 1.2))),
      ],
      markerSideM: 0.15,
      method: 'first_sight_arkit',
      frame: mappingFrameDescription,
      sourceSession: '2026-11-16_15-00-00_room',
      created: DateTime(2026, 11, 16, 15, 1, 2),
    );

void expectSamePose(Pose3 a, Pose3 b) {
  expect((a.translation - b.translation).length, lessThan(1e-12));
  expect(rotationAngle(a.rotation.transposed().multiplied(b.rotation)), lessThan(1e-12));
}

void main() {
  late Directory tmp;
  setUp(() => tmp = Directory.systemTemp.createTempSync('vslam_map_'));
  tearDown(() => tmp.deleteSync(recursive: true));

  test('map JSON round trip keeps every field', () async {
    final map = sample();
    final file = File('${tmp.path}/a/map.json');
    await map.save(file);
    final back = await MarkerMap.load(file);
    expect(back.ids, [3, 7]);
    expect(back.markerSideM, 0.15);
    expect(back.method, 'first_sight_arkit');
    expect(back.frame, mappingFrameDescription);
    expect(back.dictionary, 'DICT_4X4_50');
    expect(back.sourceSession, '2026-11-16_15-00-00_room');
    expect(back.created, DateTime(2026, 11, 16, 15, 1, 2));
    for (final m in map.markers) {
      final b = back[m.id]!;
      expectSamePose(b.wTm, m.wTm);
      expect(b.firstKeyframe, m.firstKeyframe);
      expect(b.firstTUs, m.firstTUs);
      expect(b.firstDistanceM, m.firstDistanceM);
      expect(b.firstViewDeg, m.firstViewDeg);
      expect(b.acceptedObservations, m.acceptedObservations);
      expect(b.keyframeObservations, m.keyframeObservations);
    }
    // Saving what was loaded gives the same poses again. (Not the same
    // bytes: Pose3 keeps a rotation matrix, and quaternion → matrix →
    // quaternion changes the last printed digit, ~1e-16.)
    final again = MarkerMap.fromJson(back.toJson());
    for (final m in map.markers) {
      expectSamePose(again[m.id]!.wTm, m.wTm);
    }
  });

  test('foreign or newer files are refused', () {
    expect(() => MarkerMap.fromJson({'format': 'other', 'version': 1}), throwsFormatException);
    expect(() => MarkerMap.fromJson({'format': MarkerMap.format, 'version': 2}), throwsFormatException);
  });

  test('room maps live under maps/<room>.json', () async {
    final store = MapStore(tmp);
    expect(await store.list(), isEmpty);
    final file = await store.save('Lab 2 / IE415', sample());
    expect(file.path, '${tmp.path}/maps/lab-2-ie415.json');
    await store.save('hall', sample());
    expect(await store.list(), ['hall', 'lab-2-ie415']);
    final back = await store.load('lab-2-ie415');
    expectSamePose(back[3]!.wTm, sample()[3]!.wTm);
  });
}
