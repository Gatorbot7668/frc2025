Next tasks:
* calibrate NavX, https://yagsl.gitbook.io/yagsl/devices/gyroscope/navx
* name the robot and move deploy/swerve files into a subdirectory, to prep for
  the next year
* adjust constants in SwerveSubsystem constructor (`angleConversionFactor`, `driveConversionFactor`, `maximumSpeed`)
   * see if `angleConversionFactor` should be set to 360 since we are
     using absolute encoder, https://yagsl.gitbook.io/yagsl/configuring-yagsl/standard-conversion-factors
* learn about pathplanning with http://pathplanner.dev
* tune PIDs, https://yagsl.gitbook.io/yagsl/configuring-yagsl/how-to-tune-pidf
  * learn how to output PID constants to the dashboard for live editing
  * angle and drive for indiviual modules is in pidfproperties.json
  * robot angle is in controllerproperties.json
  * robot drive is in SwerveSubsystem.setupPathPlanner
* make sure zeroGyro works (assigned to A button)
* clean up commands in RobotContainer

Useful links:
* [YAGSL, the Swerve library we are using](https://yagsl.gitbook.io/yagsl)
* [Visualizing Swerve with FRC Web Components](https://yagsl.gitbook.io/yagsl/analytics-and-debugging/frc-web-components)
* [AdvantageScope is maybe useful too](https://github.com/Mechanical-Advantage/AdvantageScope/blob/main/docs/NAVIGATION.md)
* [How to get Logitech controller to work on a Mac](https://gist.github.com/jackblk/8138827afd986f30cf9d26647e8448e1)
* [How to fix "Unresolved dependency: org.junit.platform junit-platform-launcher" build error in wpilib 2024.1.1](https://www.chiefdelphi.com/t/wpilib-blog-2024-kickoff-release-of-wpilib/448056/9)

Notes:
* Zeroing cancoders via Phoenix Tuner did not work because could not figure
  out how to burn constants into EEPROM, so that they stick after power-cycle.
  Setting constants in software (via deploy/swerve JSON files) worked great.
* `driveConversionFactor` and `angleConversionFactor` are set in
  SwerveSubsystem.java rather than JSON config so that calculating them is done with code rather then by hand.