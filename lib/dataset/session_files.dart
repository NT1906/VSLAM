/// File names and CSV columns of a saved session.
///
/// ```
/// sessions/<date>_<time>_<label>/
///   meta.json
///   frames.csv        index, t_us, file, fx, fy, cx, cy, tracking_state,
///                     tx, ty, tz, qx, qy, qz, qw
///   frames/000001.jpg
///   arkit_poses.csv   t_us, tx, ty, tz, qx, qy, qz, qw, tracking_state
///   imu.csv           t_us, sensor, x, y, z
///   results/          written by processing (later packages)
/// ```
///
/// `frames.csv` carries each image's own ARKit pose (empty when not
/// tracking). The pose stream in `arkit_poses.csv` is throttled to about
/// 30 Hz and comes from other ARKit frames, so it cannot supply the exact
/// pose of a saved image.
abstract final class SessionFiles {
  static const meta = 'meta.json';
  static const frames = 'frames.csv';
  static const frameDir = 'frames';
  static const poses = 'arkit_poses.csv';
  static const imu = 'imu.csv';
  static const results = 'results';

  static const framesHeader = 'index,t_us,file,fx,fy,cx,cy,tracking_state,tx,ty,tz,qx,qy,qz,qw';
  static const posesHeader = 't_us,tx,ty,tz,qx,qy,qz,qw,tracking_state';
  static const imuHeader = 't_us,sensor,x,y,z';

  /// `frames/000001.jpg` for index 1.
  static String frameFile(int index) => '$frameDir/${index.toString().padLeft(6, '0')}.jpg';

  /// Folder-name-safe version of a label: lower case, `[a-z0-9-]` only.
  static String slug(String label) {
    final s = label.toLowerCase().replaceAll(RegExp(r'[^a-z0-9]+'), '-').replaceAll(RegExp(r'^-+|-+$'), '');
    return s.isEmpty ? 'run' : s;
  }

  /// `2026-11-11_14-03-22_<slug>` for a recording started at [t].
  static String folderName(DateTime t, String label) {
    String two(int v) => v.toString().padLeft(2, '0');
    return '${t.year}-${two(t.month)}-${two(t.day)}_${two(t.hour)}-${two(t.minute)}-${two(t.second)}_${slug(label)}';
  }

  /// The `<date>_<time>` prefix of a folder name made by [folderName].
  static String folderPrefix(String folderName) {
    final match = RegExp(r'^\d{4}-\d{2}-\d{2}_\d{2}-\d{2}-\d{2}').firstMatch(folderName);
    return match?.group(0) ?? folderName;
  }
}
