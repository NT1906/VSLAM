import 'dart:convert';
import 'dart:io';
import 'dart:isolate';

import 'package:archive/archive_io.dart';

import 'session_files.dart';
import 'session_meta.dart';
import 'session_reader.dart';

/// A session folder with its metadata, or the reason it could not be read.
class SessionEntry {
  const SessionEntry(this.directory, this.meta, [this.error]);

  final Directory directory;
  final SessionMeta? meta;
  final String? error;

  String get name => directory.uri.pathSegments.where((s) => s.isNotEmpty).last;
}

/// The app's saved data under one root folder (the app's Documents
/// directory on the phone, which `UIFileSharingEnabled` makes visible in
/// Finder and the Files app):
///
/// ```
/// <root>/sessions/<date>_<time>_<label>/   one recording each
/// <root>/recording_settings.json           last used recording settings
/// ```
class SessionStore {
  SessionStore({required this.root, required this.tempDir});

  final Directory root;

  /// Where export archives are built before the share sheet takes them.
  final Directory tempDir;

  Directory get sessionsDir => Directory('${root.path}/sessions');
  File get _settingsFile => File('${root.path}/recording_settings.json');

  Directory ensureSessionsDir() {
    final dir = sessionsDir;
    if (!dir.existsSync()) dir.createSync(recursive: true);
    return dir;
  }

  /// All sessions, newest first.
  Future<List<SessionEntry>> list() async {
    final dir = sessionsDir;
    if (!await dir.exists()) return const [];
    final entries = <SessionEntry>[];
    await for (final e in dir.list()) {
      if (e is! Directory) continue;
      try {
        entries.add(SessionEntry(e, await SessionReader.readMeta(e)));
      } catch (error) {
        entries.add(SessionEntry(e, null, '$error'));
      }
    }
    entries.sort((a, b) => b.name.compareTo(a.name));
    return entries;
  }

  /// Changes the session label in `meta.json` and the folder name (the
  /// date and time prefix is kept). Returns the renamed folder.
  Future<Directory> rename(Directory session, String label) async {
    final meta = await SessionReader.readMeta(session);
    final updated = meta.copyWith(settings: meta.settings.copyWith(label: label));
    await File('${session.path}/${SessionFiles.meta}')
        .writeAsString(const JsonEncoder.withIndent('  ').convert(updated.toJson()));
    final prefix = SessionFiles.folderPrefix(session.uri.pathSegments.where((s) => s.isNotEmpty).last);
    final base = '${prefix}_${SessionFiles.slug(label)}';
    var target = Directory('${session.parent.path}/$base');
    if (target.path == session.path) return session;
    for (var n = 2; await target.exists(); n++) {
      target = Directory('${session.parent.path}/${base}_$n');
    }
    return session.rename(target.path);
  }

  Future<void> delete(Directory session) => session.delete(recursive: true);

  /// Zips the session folder (stored, not recompressed: the images are
  /// already JPEG) on a background isolate. Returns the archive file.
  Future<File> exportZip(Directory session) async {
    final name = session.uri.pathSegments.where((s) => s.isNotEmpty).last;
    final zipPath = '${tempDir.path}/$name.zip';
    final sourcePath = session.path;
    await Isolate.run(() async {
      final encoder = ZipFileEncoder()..create(zipPath, level: ZipFileEncoder.store);
      await encoder.addDirectory(Directory(sourcePath), level: ZipFileEncoder.store);
      await encoder.close();
    });
    return File(zipPath);
  }

  Future<RecordingSettings> loadSettings() async {
    try {
      final text = await _settingsFile.readAsString();
      return RecordingSettings.fromJson(jsonDecode(text) as Map<String, dynamic>);
    } on FileSystemException {
      return const RecordingSettings();
    } on FormatException {
      return const RecordingSettings();
    }
  }

  Future<void> saveSettings(RecordingSettings settings) async {
    await root.create(recursive: true);
    await _settingsFile.writeAsString(jsonEncode(settings.toJson()));
  }
}
