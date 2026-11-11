import 'package:flutter/material.dart';

import '../dataset/session_store.dart';
import 'session_detail_screen.dart';

/// Saved recordings, newest first. Tap one to open it.
class SessionsScreen extends StatefulWidget {
  const SessionsScreen({super.key, required this.store});

  final SessionStore store;

  @override
  State<SessionsScreen> createState() => _SessionsScreenState();
}

class _SessionsScreenState extends State<SessionsScreen> {
  late Future<List<SessionEntry>> _entries = widget.store.list();

  void _reload() {
    final entries = widget.store.list();
    setState(() {
      _entries = entries;
    });
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        title: const Text('Sessions'),
        actions: [IconButton(icon: const Icon(Icons.refresh), tooltip: 'Refresh', onPressed: _reload)],
      ),
      body: FutureBuilder<List<SessionEntry>>(
        future: _entries,
        builder: (context, snapshot) {
          if (snapshot.hasError) return Center(child: Text('Could not list sessions: ${snapshot.error}'));
          if (!snapshot.hasData) return const Center(child: CircularProgressIndicator());
          final entries = snapshot.data!;
          if (entries.isEmpty) {
            return const Center(
              child: Padding(
                padding: EdgeInsets.all(24),
                child: Text(
                  'No sessions yet. Press Start on the main screen to record one.',
                  textAlign: TextAlign.center,
                ),
              ),
            );
          }
          return ListView.separated(
            itemCount: entries.length,
            separatorBuilder: (_, _) => const Divider(height: 1),
            itemBuilder: (context, i) => _SessionTile(
              entry: entries[i],
              onTap: () async {
                await Navigator.push<void>(
                  context,
                  MaterialPageRoute(
                    builder: (_) => SessionDetailScreen(store: widget.store, directory: entries[i].directory),
                  ),
                );
                _reload();
              },
            ),
          );
        },
      ),
    );
  }
}

class _SessionTile extends StatelessWidget {
  const _SessionTile({required this.entry, required this.onTap});

  final SessionEntry entry;
  final VoidCallback onTap;

  @override
  Widget build(BuildContext context) {
    final meta = entry.meta;
    if (meta == null) {
      return ListTile(
        leading: const Icon(Icons.error_outline),
        title: Text(entry.name),
        subtitle: Text('Unreadable: ${entry.error}', maxLines: 2, overflow: TextOverflow.ellipsis),
        onTap: onTap,
      );
    }
    final d = meta.duration;
    final length = '${d.inMinutes}:${(d.inSeconds % 60).toString().padLeft(2, '0')}';
    return ListTile(
      title: Text(meta.settings.label),
      subtitle: Text(
        '${entry.name}\n${meta.settings.experiment.label} · $length · ${meta.counts.frames} frames · '
        '${meta.counts.accelerometer + meta.counts.gyroscope} IMU samples',
      ),
      isThreeLine: true,
      trailing: const Icon(Icons.chevron_right),
      onTap: onTap,
    );
  }
}
