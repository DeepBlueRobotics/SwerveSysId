// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.stream.Collectors;

import org.carlmontrobotics.lib199.swerve.SwerveModule.ModuleType;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {

  public static Robot robot;

  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  private static double ackNum = 0;
  private static SendableChooser<ModuleType> m_chooser = new SendableChooser<>();

  @Override
  public void robotInit() {
    robot = this;
    m_robotContainer = new RobotContainer();
    setTelemetry("");
    ackNum = SmartDashboard.getNumber("SysIdAckNumber", 0); // Input
    SmartDashboard.putNumber("SysIdVoltageCommand", 0.0); // Input
    SmartDashboard.putString("SysIdTestType", ""); // Input
    SmartDashboard.putString("SysIdTest", ""); // Input
    SmartDashboard.putBoolean("SysIdRotate", false); // Input
    SmartDashboard.putBoolean("SysIdOverflow", false); // Output
    SmartDashboard.putBoolean("SysIdWrongMech", false); // Output

    for(ModuleType type: ModuleType.values()) m_chooser.addOption(type.toString(), type);

    SmartDashboard.putData("Module Type", m_chooser);
    SmartDashboard.putBoolean("Drive", false);
    SmartDashboard.putNumber("Direction", 0);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {
    clearWhenReceived();
  }

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  public void clearWhenReceived() {
    if(SmartDashboard.getNumber("SysIdAckNumber", 0) > ackNum) {
      setTelemetry("");
      ackNum = SmartDashboard.getNumber("SysIdAckNumber", 0);
    }
  }

  public static void setTelemetry(ArrayList<double[]> data) {
    StringBuilder builder = new StringBuilder();
    builder.append(getTestType() == TestType.DYNAMIC ? "fast" : "slow");
    builder.append("-");
    builder.append(getVoltageCommand() > 0 ? "forward" : "backward");
    builder.append(";");
    builder.append(data.stream().flatMapToDouble(Arrays::stream).mapToObj(Double::toString).collect(Collectors.joining(",")));
    setTelemetry(builder.toString());
    ackNum = SmartDashboard.getNumber("SysIdAckNumber", 0);
    SmartDashboard.putNumber("SysIdAckNumber", ++ackNum);
  }

  public static void setTelemetry(String telemetry) {
    SmartDashboard.putString("SysIdTelemetry", telemetry);
  }

  public static TestType getTestType() {
    String requestedType = SmartDashboard.getString("SysIdTestType", null).toLowerCase();
    for(TestType type : TestType.values()) {
      if(type.name().toLowerCase().equals(requestedType)) {
        return type;
      }
    }
    System.err.println("Invalid test type requested: " + requestedType);
    return null;
  }

  public static double getVoltageCommand() {
    return SmartDashboard.getNumber("SysIdVoltageCommand", 0);
  }

  public static boolean getRotate() {
    return SmartDashboard.getBoolean("SysIdRotate", false);
  }

  public static void setOverflow(boolean overflow) {
    SmartDashboard.putBoolean("SysIdOverflow", overflow);
  }

  public static void setWrongMech(boolean wrongMech) {
    SmartDashboard.putBoolean("SysIdWrongMech", wrongMech);
  }

  public static AnalysisType getAnalysisType() {
    String requestedType = SmartDashboard.getString("SysIdTest", null).toLowerCase();
    if(requestedType == "Drivetrain (Angular)".toLowerCase()) {
      return AnalysisType.DRIVETRAIN_ANGULAR;
    }
    for(AnalysisType type : AnalysisType.values()) {
      if(type.name().toLowerCase().equals(requestedType)) {
        return type;
      }
    }
    System.err.println("Invalid analysis type requested: " + requestedType);
    return null;
  }

  public static ModuleType getModuleType() {
    return m_chooser.getSelected();
  }

  public static boolean getDrive() {
    return SmartDashboard.getBoolean("Drive", false);
  }

  public static double getDirection() {
    return SmartDashboard.getNumber("Direction", 0);
  }

}
