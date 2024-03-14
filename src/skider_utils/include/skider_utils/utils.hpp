
// enum ChassisState{

//     ChassisWeak = 0,
//     ChassisJoy = 1,
// };

// enum GimbalState{

//     GimbalWeak = 0,
//     GimbalJoy = 1,
//     GimbalRotor = 2
// };

enum class RobotState {

  ChassisWeakGimbalWeak = 0,
  ChassisWeakGimbalJoy = 1,
  ChassisWeakGimbalJoyRotor = 2,
  ChassisJoyGimbalWeak = 3,
  ChassisJoyGimbalJoy = 4,
  ChassisJoyGimbalJoyRotor = 5,
  AutoaimGimbalWeak = 6,
  AutoaimGimbalAutoaim = 7,
  AutoaimGimbalAutoaimRotor = 8,

};
