import 'dart:convert';
import 'dart:io';

import '../core/se3.dart';
import '../dataset/session_files.dart';
import '../estimation/keyframe_selector.dart';
import '../vision/marker_observation.dart';

/// One marker of a map: its pose `wTm` in the map frame W (marker frame of
/// spec 4.3: origin at the centre, x right, y up, z out of the face) and
/// where the estimate came from.
class MapMarker {
  const MapMarker({
    required this.id,
    required this.wTm,
    this.firstKeyframe,
    this.firstTUs,
    this.firstDistanceM,
    this.firstViewDeg,
    this.acceptedObservations = 0,
    this.keyframeObservations = 0,
  });

  final int id;
  final Pose3 wTm;

  /// Keyframe and time of the detection the marker was placed from.
  final int? firstKeyframe;
  final int? firstTUs;

  /// Camera–marker distance and viewing angle of that detection.
  final double? firstDistanceM;
  final double? firstViewDeg;

  /// Accepted detections of this ID over all frames of the session.
  final int acceptedObservations;

  /// Usable detections of this ID in keyframes (what WP5 turns into
  /// factors).
  final int keyframeObservations;

  Map<String, dynamic> toJson() => {
        'id': id,
        'pose': wTm.toJson(),
        'first_keyframe': ?firstKeyframe,
        'first_t_us': ?firstTUs,
        'first_distance_m': ?firstDistanceM,
        'first_view_deg': ?firstViewDeg,
        'accepted_observations': acceptedObservations,
        'keyframe_observations': keyframeObservations,
      };

  factory MapMarker.fromJson(Map<String, dynamic> json) => MapMarker(
        id: (json['id'] as num).toInt(),
        wTm: Pose3.fromJson(json['pose'] as Map<String, dynamic>),
        firstKeyframe: (json['first_keyframe'] as num?)?.toInt(),
        firstTUs: (json['first_t_us'] as num?)?.toInt(),
        firstDistanceM: (json['first_distance_m'] as num?)?.toDouble(),
        firstViewDeg: (json['first_view_deg'] as num?)?.toDouble(),
        acceptedObservations: (json['accepted_observations'] as num?)?.toInt() ?? 0,
        keyframeObservations: (json['keyframe_observations'] as num?)?.toInt() ?? 0,
      );
}

/// A map of ArUco markers in one frame W, as saved in `results/map.json`
/// and `maps/<room>.json`.
class MarkerMap {
  const MarkerMap({
    required this.markers,
    required this.markerSideM,
    required this.method,
    required this.frame,
    this.dictionary = 'DICT_4X4_50',
    this.sourceSession,
    this.created,
  });

  static const format = 'vslam-marker-map';
  static const version = 1;

  /// Markers sorted by ID.
  final List<MapMarker> markers;
  final double markerSideM;
  final String dictionary;

  /// How the marker poses were obtained, e.g. `first_sight_arkit` (WP4).
  final String method;

  /// What W is, in words (saved so a map file explains itself).
  final String frame;

  /// Folder name of the session the map was built from.
  final String? sourceSession;
  final DateTime? created;

  MapMarker? operator [](int id) {
    for (final m in markers) {
      if (m.id == id) return m;
    }
    return null;
  }

  List<int> get ids => [for (final m in markers) m.id];

  Map<String, dynamic> toJson() => {
        'format': format,
        'version': version,
        'method': method,
        'frame': frame,
        'dictionary': dictionary,
        'marker_side_m': markerSideM,
        'source_session': ?sourceSession,
        'created': ?created?.toIso8601String(),
        'markers': [for (final m in markers) m.toJson()],
      };

  factory MarkerMap.fromJson(Map<String, dynamic> json) {
    if (json['format'] != format) throw FormatException('Not a marker map (format "${json['format']}")');
    final v = (json['version'] as num?)?.toInt();
    if (v != version) throw FormatException('Unsupported marker map version $v');
    final markers = [for (final m in json['markers'] as List) MapMarker.fromJson(m as Map<String, dynamic>)]
      ..sort((a, b) => a.id.compareTo(b.id));
    return MarkerMap(
      markers: markers,
      markerSideM: (json['marker_side_m'] as num).toDouble(),
      dictionary: json['dictionary'] as String? ?? 'DICT_4X4_50',
      method: json['method'] as String? ?? '',
      frame: json['frame'] as String? ?? '',
      sourceSession: json['source_session'] as String?,
      created: json['created'] == null ? null : DateTime.parse(json['created'] as String),
    );
  }

  String encode() => const JsonEncoder.withIndent('  ').convert(toJson());

  Future<File> save(File file) async {
    await file.parent.create(recursive: true);
    return file.writeAsString(encode());
  }

  static Future<MarkerMap> load(File file) async =>
      MarkerMap.fromJson(jsonDecode(await file.readAsString()) as Map<String, dynamic>);
}

/// The text saved in [MarkerMap.frame] for maps built in mapping mode.
const mappingFrameDescription = 'W = gravity-aligned frame of the first keyframe: origin at its camera centre, '
    'z up, x along the horizontal direction the camera looked';

/// Builds the WP4 marker map: every marker is placed once, from its first
/// accepted detection in a keyframe, by composing the baseline pose of that
/// keyframe with the PnP pose of the detection (spec 4.4, marker
/// initialization):
///
///     wTm = wTc_k · cTm
///
/// [poses] are the keyframe poses `wTc_k` (baseline 1), [detections] the
/// usable detections per frame time. Detections that failed a rejection
/// rule never place a marker; they still count as keyframe observations
/// when their status is listed in [usableStatuses].
MarkerMap buildFirstSightMap({
  required List<Keyframe> keyframes,
  required List<Pose3> poses,
  required Map<int, List<MarkerObservation>> detections,
  required double markerSideM,
  required Set<String> usableStatuses,
  String? sourceSession,
  DateTime? created,
}) {
  final accepted = <int, int>{};
  for (final list in detections.values) {
    for (final m in list) {
      if (m.accepted) accepted[m.id] = (accepted[m.id] ?? 0) + 1;
    }
  }
  final placed = <int, MapMarker>{};
  final inKeyframes = <int, int>{};
  for (final kf in keyframes) {
    for (final m in detections[kf.tUs] ?? const <MarkerObservation>[]) {
      if (usableStatuses.contains(m.statusCode)) inKeyframes[m.id] = (inKeyframes[m.id] ?? 0) + 1;
      if (!m.accepted || placed.containsKey(m.id)) continue;
      placed[m.id] = MapMarker(
        id: m.id,
        wTm: poses[kf.index] * m.cTm!,
        firstKeyframe: kf.index,
        firstTUs: kf.tUs,
        firstDistanceM: m.distanceM,
        firstViewDeg: m.viewAngleDeg,
      );
    }
  }
  final markers = [
    for (final m in placed.values)
      MapMarker(
        id: m.id,
        wTm: m.wTm,
        firstKeyframe: m.firstKeyframe,
        firstTUs: m.firstTUs,
        firstDistanceM: m.firstDistanceM,
        firstViewDeg: m.firstViewDeg,
        acceptedObservations: accepted[m.id] ?? 0,
        keyframeObservations: inKeyframes[m.id] ?? 0,
      ),
  ]..sort((a, b) => a.id.compareTo(b.id));
  return MarkerMap(
    markers: markers,
    markerSideM: markerSideM,
    method: 'first_sight_arkit',
    frame: mappingFrameDescription,
    sourceSession: sourceSession,
    created: created,
  );
}

/// Room maps under `<root>/maps/<room>.json` (spec section 5 layout).
class MapStore {
  MapStore(this.root);

  final Directory root;

  Directory get mapsDir => Directory('${root.path}/maps');

  /// File of the map named [room] (slugged like session labels).
  File fileFor(String room) => File('${mapsDir.path}/${SessionFiles.slug(room)}.json');

  Future<File> save(String room, MarkerMap map) => map.save(fileFor(room));

  Future<MarkerMap> load(String room) => MarkerMap.load(fileFor(room));

  /// Room names with a saved map, sorted.
  Future<List<String>> list() async {
    if (!await mapsDir.exists()) return const [];
    final names = <String>[
      await for (final e in mapsDir.list())
        if (e is File && e.path.endsWith('.json')) e.uri.pathSegments.last.replaceAll(RegExp(r'\.json$'), ''),
    ]..sort();
    return names;
  }
}
