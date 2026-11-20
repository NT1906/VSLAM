import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:path_provider/path_provider.dart';
import 'package:provider/provider.dart';

import 'dataset/session_store.dart';
import 'screens/home_screen.dart';
import 'services/imu_service.dart';
import 'services/session_controller.dart';
import 'slam/slam_service.dart';

Future<void> main() async {
  WidgetsFlutterBinding.ensureInitialized();
  // The AR display geometry assumes portrait.
  SystemChrome.setPreferredOrientations([DeviceOrientation.portraitUp]);
  final slam = SlamService();
  await slam.setFrameRate(10);
  // Sessions live in the app's Documents directory, which Info.plist
  // (UIFileSharingEnabled) exposes in Finder and the Files app.
  final store = SessionStore(root: await getApplicationDocumentsDirectory(), tempDir: await getTemporaryDirectory());
  runApp(VslamApp(imu: ImuService(), slam: slam, store: store));
}

class VslamApp extends StatelessWidget {
  const VslamApp({super.key, required this.imu, required this.slam, this.store});

  final ImuService imu;
  final SlamService slam;

  /// Where recordings are saved; null disables saving (UI-only tests).
  final SessionStore? store;

  @override
  Widget build(BuildContext context) {
    return MultiProvider(
      providers: [
        ChangeNotifierProvider.value(value: imu),
        ChangeNotifierProvider.value(value: slam),
        Provider<SessionStore?>.value(value: store),
        ChangeNotifierProvider(
          create: (_) => SessionController(imu: imu, slam: slam, store: store)..load(),
        ),
      ],
      child: MaterialApp(
        title: 'VSLAM',
        debugShowCheckedModeBanner: false,
        themeMode: ThemeMode.system,
        theme: ThemeData(colorSchemeSeed: Colors.indigo, useMaterial3: true),
        darkTheme: ThemeData(colorSchemeSeed: Colors.indigo, brightness: Brightness.dark, useMaterial3: true),
        home: const HomeScreen(),
      ),
    );
  }
}
