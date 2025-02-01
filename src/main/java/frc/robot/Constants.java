// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    /**
     * Constants that describe physical aspects of the entire robot.
     */
    public static final class RobotConstants {
        /** Mass of the robot in kilograms. */
        public static final double kRobotMassKG = 68;

        /** Robot's moment of inertia. */
        public static final double kRobotMOI = 20;
    }

    /**
     * Constants that describe how the robot should move and
     * how specific hardware is identified in software.
     */
    public static final class DriveConstants {
        // Driving Parameters - Note that these are not the maximum capable speeds of
        // the robot, rather the allowed maximum speeds

        /** The highest allowed speed of the drive train. */
        public static final double kMaxSpeedMetersPerSecond = 5;

        /** The highest allowed rotational speed of the drive train. */
        public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

        /** The highest acceleration allowed in the drive train */
        public static final double kMagnitudeSlewRate = 5 * kMaxSpeedMetersPerSecond; // meters per second^2

        /** The highest rotational acceleration allowed in the drive train */
        public static final double kRotationalSlewRate = 5 * kMaxAngularSpeed; // radians per second^2

        // Chassis configuration

        /** Distance between centers of right and left wheels on robot. */
        public static final double kTrackWidth = Units.inchesToMeters(21);

        /** Distance between front and back wheels on robot. */
        public static final double kWheelBase = Units.inchesToMeters(21);

        /** A kinematics object that calculates how the robot should move
         * using the positions of the modules on the robot.
         */
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
            new Translation2d(kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));
    

        // SPARK MAX CAN IDs

        /** CAN ID of the front left driving motor controller. */
        public static final int kFrontLeftDrivingCanId = 8; 

        /** CAN ID of the rear left driving motor controller. */
        public static final int kRearLeftDrivingCanId = 6;

        /** CAN ID of the front right driving motor controller. */
        public static final int kFrontRightDrivingCanId = 2;

        /** CAN ID of the rear lefrightt driving motor controller. */
        public static final int kRearRightDrivingCanId = 4;


        /** CAN ID of the front left turning motor controller. */
        public static final int kFrontLeftTurningCanId = 7;

        /** CAN ID of the rear left turning motor controller. */
        public static final int kRearLeftTurningCanId = 5;

        /** CAN ID of the front right turning motor controller. */
        public static final int kFrontRightTurningCanId = 1;

        /** CAN ID of the rear right turning motor controller. */
        public static final int kRearRightTurningCanId = 3;

        // Encoder CAN Ids
        public static final int KFrontLeftTurningEncoderId = 14;
        public static final int KFrontRightTurningEncoderId = 12;
        public static final int KRearLeftTurningEncoderId = 13;
        public static final int KRearRightTurningEncoderId = 11;
    }

    /**
     * Constants related to the individual swerve
     * modules and not to the drive subsystem itself.
     */
    public static final class ModuleConstants {
        // Calculations required for driving motor conversion factors and feed forward
        public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
        public static final double kWheelRadiusMeters = kWheelDiameterMeters / 2;
        public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;

        public static final DCMotor kMotorGearboxConfig = DCMotor.getNEO(1);

        // The coefficient of friction between the drive wheel and the carpet
        public static final double kWheelCOF = 1;

        // The L1 MK4 and MK4i modules have a gear ratio of 8.14:1 on the drive wheels.
        public static final double kDrivingMotorReduction = 6.75;
        public static final double kDriveWheelFreeSpeedMetersPerSecond = (NEOMotorConstants.kFreeSpeedRps * kWheelCircumferenceMeters)
                / kDrivingMotorReduction;

        public static final double kDrivingEncoderPositionFactor = kWheelCircumferenceMeters
                / kDrivingMotorReduction; // meters
        public static final double kDrivingEncoderVelocityFactor = (kWheelCircumferenceMeters
                / kDrivingMotorReduction) / 60.0; // meters per second

        // Position factor for the turning encoders on the NEOs 
        public static final double kTurningMotorReduction = 150.0 / 7;

        public static final double kTurningEncoderPositionFactor = (2 * Math.PI) / kTurningMotorReduction; // radians
        public static final double kTurningEncoderVelocityFactor = kTurningEncoderPositionFactor / 60.0; // radians per second

        public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
        public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor; // radians

        public static final double kDrivingP = 0.04;
        public static final double kDrivingI = 0;
        public static final double kDrivingD = 0;
        public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedMetersPerSecond; 
        public static final double kDrivingMinOutput = -1;
        public static final double kDrivingMaxOutput = 1;

        public static final double kTurningP = 0.75;
        public static final double kTurningI = 0;
        public static final double kTurningD = 0;
        public static final double kTurningFF = 0;
        public static final double kTurningMinOutput = -1;
        public static final double kTurningMaxOutput = 1;

        // Inversion of drive motors
        // This will vary depending on how your wheels are oriented when you zero them.
        public static final boolean kLeftFrontInverted = true;
        public static final boolean kLeftRearInverted = true;
        public static final boolean kRightFrontInverted = true;
        public static final boolean kRightRearInverted = true;

        // Inversion of turning motors
        // Unless oriented differently, all of your turning motors should spin in the same direction.
        public static final boolean kTurningMotorsInverted = true;

        // Inversion of turning ENCODERS (not motors).
        // Unless oriented differently, all of your turning encoders should spin in the same direction.
        public static final boolean kTurningEncoderInverted = false;

        public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
        public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

        public static final int kDrivingMotorCurrentLimit = 35; // amps
        public static final int kTurningMotorCurrentLimit = 35; // amps
    }

    public static final class HeadingConstants {
        // The gyro should be CCW positive
        public static final boolean kGyroReversed = true;

        // This is used for making the robot face a certain direction
        public static final double kHeadingP = 0.025;
        public static final double kHeadingI = 0;
        public static final double kHeadingD = 0.001;
        public static final double kHeadingMaxOutput = 0.8; // Percent
        public static final double kHeadingTolerance = 1; // Degrees

        public static final double kTranslationP = 5;
        public static final double kTranslationI = 0;
        public static final double kTranslationD = 0;
        public static final double kTranslationMaxOutput = 1; // Percent 
        public static final double kTranslationTolerance = Units.inchesToMeters(3); // Meters
    }

    public static final class OIConstants {
        public static final int kDriverControllerPort = 0;
        public static final int kCoDriverControllerPort = 1;
        
        public static final double kJoystickDeadband = 0.05;
        public static final double kTriggerDeadband = 0.5;
    }

    public static final class FieldConstants {
        /** X axis: long side */
        public static final double kFieldWidthMeters = 16.54175;
        /** Y axis: short side */
        public static final double kFieldHeightMeters = 8.2;
        
        // Translation2d can be used to store the coordinates of important positions on the field:
        public static final Translation2d kRandomPosition = new Translation2d(
            kFieldWidthMeters/2, kFieldHeightMeters/2
        );
    }

    public static final class AutoConstants {
        public static final double kAutoMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;

        public static final PIDConstants kAutoTranslationPID = new PIDConstants(
            HeadingConstants.kTranslationP, 
            HeadingConstants.kTranslationI, 
            HeadingConstants.kTranslationD
        );

        public static final PIDConstants kAutoAngularPID = new PIDConstants(
            5, 
            0, 
            0
        );

        public static final PPHolonomicDriveController kPathPlannerController = new PPHolonomicDriveController( 
            kAutoTranslationPID, // Translation PID constants
            kAutoAngularPID // Rotation PID constants
        );

        public static final RobotConfig kPathPlannerRobotConfig = new RobotConfig(
            RobotConstants.kRobotMassKG, // robot mass
            RobotConstants.kRobotMOI, // robot moment of inertia
            new ModuleConfig(
                ModuleConstants.kWheelRadiusMeters, // drive wheel radius
                ModuleConstants.kDriveWheelFreeSpeedMetersPerSecond, // drive motor maximum capable speed
                ModuleConstants.kWheelCOF, // wheel friction on carpet
                ModuleConstants.kMotorGearboxConfig, // config for NEO motor gearbox
                ModuleConstants.kDrivingMotorReduction, // gear ratio
                ModuleConstants.kDrivingMotorCurrentLimit, // drive motor current limit
                1 // number of drive motors per module
            ),
            DriveConstants.kDriveKinematics.getModules() // module offsets from center of chassis
        );
    }

    public static final class NEOMotorConstants {
        public static final double kFreeSpeedRpm = 5676;
        public static final double kFreeSpeedRps = kFreeSpeedRpm / 60;
    }

    public static final class VisionConstants {
        // Pipeline constants
        public static final int kAprilTagPipeline = 0;
        // public static final int kReflectiveTapePipeline = 3;
        public static final int kGamePiecePipeline = 2;

        // NOTE: the limelight starts with pipeline 0 by default, so we need to make sure we make that pipeline something 
        // that doesn't use the green lights so we don't blind everybody.
        public static final int kDefaultPipeline = kAprilTagPipeline;
    }
}
