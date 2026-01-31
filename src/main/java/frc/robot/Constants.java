// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  // ==================================================================
  // Hardware IO constants
  // ==================================================================

  /**
   * Analog IO contants
   */
  public static final class AnalogConstants {}

  /**
   * CAN bus IO contants
   */
  public static final class CANConstants {}
  
  /**
   * Digital IO constants
   */
  public static final class DIOConstants {}

  /**
   * PWM IO constants
   */
  public static class PWMConstants {
    public static final int LEDStringID = 0;
    public static final int IntakeID    = 1;
    public static final int ShooterID   = 2;
    public static final int ClimberID   = 3;
  }

  /**
   * Field constants
   */
  public static class FieldConstants {
    public static final AprilTagFieldLayout kFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
    public static final double kFieldLengthMeters = Units.inchesToMeters(651.25); // meters
    public static final double kFieldWidthMeters = Units.inchesToMeters(315.5); // meters
  }

  // ==================================================================
  // Subsystem constants
  // ==================================================================

  /**
   * Climber subsystem constants
   */
  public static class ClimberConstants {
    // motor speeds from -1.0 to 1.0 (-100% to 100% -> x 12 for voltage control)
    public static double UpVoltage   =  0.50 * 12; 
    public static double DownVoltage = -0.50 * 12; 
  }

  /**
   * Feedback subsystem constants
   */
  public static class FeedbackConstants { 
    /** Number of LEDs in the strip */
    public static final int LEDLength = 60;
      
    /** Color when robot is idle/ready (soft blue) */
    public static final Color IdleColor = new Color(0.0, 0.3, 1.0);
    
    /** Color for warnings (orange) */
    public static final Color WarningColor = new Color(1.0, 0.5, 0.0);
    
    /** Color for errors (red) */
    public static final Color ErrorColor = Color.kRed;
       
    /** Team color - Green */
    public static final Color TeamGreen = new Color(0.0, 0.8, 0.2);
    
    /** Team color - Copper */
    public static final Color TeamCopper = new Color(0.72, 0.45, 0.20);
       
    /**
     * Enum representing different LED display modes
     */
    public enum DisplayMode {
      /** All LEDs off */
      OFF,
      
      /** Robot idle/ready state */
      IDLE,
      
      /** Warning state */
      WARNING,
      
      /** Error state */
      ERROR,
      
      /** Team colors gradient chase */
      TEAM_COLORS,

      /** Scoring shift indication */
      SCORING_SHIFT
    }
  }

  /**
   * Intake subsystem constants
   */
  public static class IntakeConstants {
    // motor speeds from -1.0 to 1.0 (-100% to 100% -> x 12 for voltage control)
    public static double IntakeFuelVoltage = 0.20 * 12; 
  }

  /**
   * Swerve drive subsystem contants
   */
  public static class SwerveConstants {
    public static final double MaxSpeed       = Units.feetToMeters( 10 ); // 14.5;
    public static final double WheelLockTime  = 10; // seconds
    public static final double RobotMass      = Units.lbsToKilograms(134);
    public static final Matter Chassis        = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), RobotMass);
    public static final double LoopTime       = 0.13; //s, 20ms + 110ms spark max velocity lag 
    
    public static final double Deadband       = 0.1;
    public static final double LeftYDeadbad   = 0.1;
    public static final double RightXDeadband = 0.1;
    public static final double TurnConstant   = 6;

    public static final double DefaultScaleTranslation = 0.8;
    public static final double SlowModeScaleTranlastion = 0.3;

    public static final double DefaultScaleRotation = 0.7;
    public static final double SlowModeScaleRotation = 0.3;    
  }
}
