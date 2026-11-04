import 'dart:async';

import 'package:flutter/widgets.dart';

/// Rebuilds [builder] every [period] (for clocks and rates that change
/// without a notification).
class PeriodicRebuild extends StatefulWidget {
  const PeriodicRebuild({super.key, required this.builder, this.period = const Duration(milliseconds: 500)});

  final WidgetBuilder builder;
  final Duration period;

  @override
  State<PeriodicRebuild> createState() => _PeriodicRebuildState();
}

class _PeriodicRebuildState extends State<PeriodicRebuild> {
  late final Timer _timer;

  @override
  void initState() {
    super.initState();
    _timer = Timer.periodic(widget.period, (_) => setState(() {}));
  }

  @override
  void dispose() {
    _timer.cancel();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) => widget.builder(context);
}
