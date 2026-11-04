import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:provider/provider.dart';

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
  runApp(VslamApp(imu: ImuService(), slam: slam));
}

class VslamApp extends StatelessWidget {
  const VslamApp({super.key, required this.imu, required this.slam});

  final ImuService imu;
  final SlamService slam;

  @override
  Widget build(BuildContext context) {
    return MultiProvider(
      providers: [
        ChangeNotifierProvider.value(value: imu),
        ChangeNotifierProvider.value(value: slam),
        ChangeNotifierProvider(
          create: (_) => SessionController(imu: imu, slam: slam),
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
