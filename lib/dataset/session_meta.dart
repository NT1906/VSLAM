/// The experiment a recording belongs to (handout section 8), plus the
/// support recordings later packages need.
enum ExperimentType {
  simplePath('Simple path'),
  markerLoss('Marker-loss path'),
  closedLoop('Closed loop'),
  repeatRun('Repeat run (relocalization)'),
  difficult('Difficult condition'),
  calibration('Calibration / alignment'),
  other('Other');

  const ExperimentType(this.label);

  final String label;

  static ExperimentType fromName(String? name) =>
      values.firstWhere((e) => e.name == name, orElse: () => ExperimentType.other);
}

/// What the person sets before pressing Start. Kept between recordings.
class RecordingSettings {
  const RecordingSettings({
    this.label = 'run',
    this.experiment = ExperimentType.simplePath,
    this.markerDictionary = 'DICT_4X4_50',
    this.markerSideM = 0.15,
    this.notes = '',
  });

  final String label;
  final ExperimentType experiment;
  final String markerDictionary;

  /// Printed side length of the black marker square, metres.
  final double markerSideM;
  final String notes;

  RecordingSettings copyWith({
    String? label,
    ExperimentType? experiment,
    String? markerDictionary,
    double? markerSideM,
    String? notes,
  }) => RecordingSettings(
    label: label ?? this.label,
    experiment: experiment ?? this.experiment,
    markerDictionary: markerDictionary ?? this.markerDictionary,
    markerSideM: markerSideM ?? this.markerSideM,
    notes: notes ?? this.notes,
  );

  Map<String, dynamic> toJson() => {
    'label': label,
    'experiment': experiment.name,
    'marker_dictionary': markerDictionary,
    'marker_side_m': markerSideM,
    'notes': notes,
  };

  factory RecordingSettings.fromJson(Map<String, dynamic> json) => RecordingSettings(
    label: json['label'] as String? ?? 'run',
    experiment: ExperimentType.fromName(json['experiment'] as String?),
    markerDictionary: json['marker_dictionary'] as String? ?? 'DICT_4X4_50',
    markerSideM: (json['marker_side_m'] as num?)?.toDouble() ?? 0.15,
    notes: json['notes'] as String? ?? '',
  );
}

/// Hardware and software the session was recorded with.
class DeviceInfo {
  const DeviceInfo({
    this.model = 'unknown',
    this.os = 'unknown',
    this.appVersion = 'unknown',
    this.appBuild = 'unknown',
  });

  /// Hardware identifier such as `iPhone15,2`.
  final String model;
  final String os;
  final String appVersion;
  final String appBuild;

  Map<String, dynamic> toJson() => {'model': model, 'os': os, 'app_version': appVersion, 'app_build': appBuild};

  factory DeviceInfo.fromJson(Map<String, dynamic> json) => DeviceInfo(
    model: json['model'] as String? ?? 'unknown',
    os: json['os'] as String? ?? 'unknown',
    appVersion: json['app_version'] as String? ?? 'unknown',
    appBuild: json['app_build'] as String? ?? 'unknown',
  );

  /// From the native `deviceInfo` method's map.
  factory DeviceInfo.fromPlatform(Map<Object?, Object?> map) => DeviceInfo(
    model: map['model'] as String? ?? 'unknown',
    os: map['os'] as String? ?? 'unknown',
    appVersion: map['appVersion'] as String? ?? 'unknown',
    appBuild: map['appBuild'] as String? ?? 'unknown',
  );
}

/// Counts written by the recorder. They let the person (and the device
/// checks) confirm nothing was lost without opening the CSV files.
class SessionCounts {
  const SessionCounts({
    this.frames = 0,
    this.framesWithoutImage = 0,
    this.frameGaps = 0,
    this.poses = 0,
    this.trackedPoses = 0,
    this.accelerometer = 0,
    this.gyroscope = 0,
  });

  final int frames;

  /// Frames whose JPEG was missing or could not be written.
  final int framesWithoutImage;

  /// Consecutive frames further apart than 1.5 × the requested interval:
  /// the number of places where at least one frame was dropped.
  final int frameGaps;
  final int poses;
  final int trackedPoses;
  final int accelerometer;
  final int gyroscope;

  Map<String, dynamic> toJson() => {
    'frames': frames,
    'frames_without_image': framesWithoutImage,
    'frame_gaps': frameGaps,
    'poses': poses,
    'tracked_poses': trackedPoses,
    'accelerometer': accelerometer,
    'gyroscope': gyroscope,
  };

  factory SessionCounts.fromJson(Map<String, dynamic> json) {
    int n(String key) => (json[key] as num?)?.toInt() ?? 0;
    return SessionCounts(
      frames: n('frames'),
      framesWithoutImage: n('frames_without_image'),
      frameGaps: n('frame_gaps'),
      poses: n('poses'),
      trackedPoses: n('tracked_poses'),
      accelerometer: n('accelerometer'),
      gyroscope: n('gyroscope'),
    );
  }
}

/// Contents of `meta.json`.
///
/// Conventions recorded with the data (so a saved session explains itself):
/// - time: integer microseconds since the Unix epoch on the boot-based clock
///   shared by ARKit and CoreMotion;
/// - images: 8-bit grayscale JPEG in sensor orientation (landscape);
/// - intrinsics: ARKit `camera.intrinsics` rescaled to the saved image size;
/// - poses: ARKit `camera.transform` (world +y up, camera looks along −z);
/// - IMU: see [ImuEvent] for units and axes.
class SessionMeta {
  const SessionMeta({
    required this.settings,
    required this.device,
    required this.created,
    this.schemaVersion = currentSchemaVersion,
    this.requestedFrameHz = 10,
    this.requestedImuHz = 100,
    this.imageWidth,
    this.imageHeight,
    this.sourceWidth,
    this.sourceHeight,
    this.jpegQuality,
    this.startTUs,
    this.endTUs,
    this.counts = const SessionCounts(),
  });

  static const currentSchemaVersion = 1;

  static const conventions = {
    'time': 'integer microseconds since the Unix epoch, boot-based clock shared by ARKit and CoreMotion',
    'images': '8-bit grayscale JPEG, sensor orientation (landscape), box-filtered from capturedImage',
    'intrinsics': 'ARKit camera.intrinsics rescaled to the saved image size; autofocus disabled',
    'arkit_poses': 'ARKit camera.transform: world +y up (gravity), camera x right, y up, looking along -z',
    'imu_accelerometer':
        'm/s^2, specific force (-CoreMotion acceleration x 9.80665), +9.81 on the axis pointing up at rest',
    'imu_gyroscope': 'rad/s, right-handed about device axes',
    'device_axes': 'x right, y toward the top of the screen (portrait), z out of the screen',
  };

  final int schemaVersion;
  final RecordingSettings settings;
  final DeviceInfo device;

  /// Local wall-clock time the recording started.
  final DateTime created;
  final double requestedFrameHz;
  final double requestedImuHz;

  /// Saved image size (null until the first frame arrives).
  final int? imageWidth;
  final int? imageHeight;

  /// Size of ARKit's `capturedImage` the saved image was reduced from.
  final int? sourceWidth;
  final int? sourceHeight;
  final double? jpegQuality;

  /// First and last sensor timestamps in the session.
  final int? startTUs;
  final int? endTUs;
  final SessionCounts counts;

  Duration get duration =>
      startTUs == null || endTUs == null ? Duration.zero : Duration(microseconds: endTUs! - startTUs!);

  SessionMeta copyWith({
    RecordingSettings? settings,
    DeviceInfo? device,
    int? imageWidth,
    int? imageHeight,
    int? sourceWidth,
    int? sourceHeight,
    double? jpegQuality,
    int? startTUs,
    int? endTUs,
    SessionCounts? counts,
  }) => SessionMeta(
    schemaVersion: schemaVersion,
    settings: settings ?? this.settings,
    device: device ?? this.device,
    created: created,
    requestedFrameHz: requestedFrameHz,
    requestedImuHz: requestedImuHz,
    imageWidth: imageWidth ?? this.imageWidth,
    imageHeight: imageHeight ?? this.imageHeight,
    sourceWidth: sourceWidth ?? this.sourceWidth,
    sourceHeight: sourceHeight ?? this.sourceHeight,
    jpegQuality: jpegQuality ?? this.jpegQuality,
    startTUs: startTUs ?? this.startTUs,
    endTUs: endTUs ?? this.endTUs,
    counts: counts ?? this.counts,
  );

  Map<String, dynamic> toJson() => {
    'schema_version': schemaVersion,
    'created': created.toIso8601String(),
    ...settings.toJson(),
    'device': device.toJson(),
    'requested_frame_hz': requestedFrameHz,
    'requested_imu_hz': requestedImuHz,
    'image_width': imageWidth,
    'image_height': imageHeight,
    'source_width': sourceWidth,
    'source_height': sourceHeight,
    'jpeg_quality': jpegQuality,
    'start_t_us': startTUs,
    'end_t_us': endTUs,
    'counts': counts.toJson(),
    'conventions': conventions,
  };

  factory SessionMeta.fromJson(Map<String, dynamic> json) => SessionMeta(
    schemaVersion: (json['schema_version'] as num?)?.toInt() ?? currentSchemaVersion,
    created: DateTime.parse(json['created'] as String),
    settings: RecordingSettings.fromJson(json),
    device: DeviceInfo.fromJson((json['device'] as Map?)?.cast<String, dynamic>() ?? const {}),
    requestedFrameHz: (json['requested_frame_hz'] as num?)?.toDouble() ?? 10,
    requestedImuHz: (json['requested_imu_hz'] as num?)?.toDouble() ?? 100,
    imageWidth: (json['image_width'] as num?)?.toInt(),
    imageHeight: (json['image_height'] as num?)?.toInt(),
    sourceWidth: (json['source_width'] as num?)?.toInt(),
    sourceHeight: (json['source_height'] as num?)?.toInt(),
    jpegQuality: (json['jpeg_quality'] as num?)?.toDouble(),
    startTUs: (json['start_t_us'] as num?)?.toInt(),
    endTUs: (json['end_t_us'] as num?)?.toInt(),
    counts: SessionCounts.fromJson((json['counts'] as Map?)?.cast<String, dynamic>() ?? const {}),
  );
}
