package frc.robot.commands.vision;

import frc.robot.subsystems.DriveSubsystem;

//Import subsystem(s) this command interacts with below

import frc.robot.subsystems.VisionSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.utils.OdometryUtils;
import frc.utils.SwerveUtils;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

import java.util.function.DoubleSupplier;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.HeadingConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.VisionConstants;

//Import this so you can make this class a command
import edu.wpi.first.wpilibj2.command.Command;

public class AutoAim extends Command {

    //Import any instance variables that are passed into the file below here, such as the subsystem(s) your command interacts with.
    final VisionSubsystem m_visionSubsystem;
    final DriveSubsystem m_driveSubsystem;
    private final PIDController angleController = new PIDController(HeadingConstants.kHeadingP, 
                                                                  HeadingConstants.kHeadingI, 
                                                                  HeadingConstants.kHeadingD);

    //If you want to contoll whether or not the command has ended, you should store it in some sort of variable:
    private boolean m_complete = false;
    private final DoubleSupplier m_xSpeed;
    private final DoubleSupplier m_ySpeed;

    //Class Constructor
    public AutoAim(VisionSubsystem visionSubsystem, DriveSubsystem driveSubsystem, DoubleSupplier xSpeed, DoubleSupplier ySpeed){
        m_driveSubsystem = driveSubsystem;
        m_visionSubsystem = visionSubsystem;
        m_xSpeed = xSpeed;
        m_ySpeed = ySpeed;
        //If your command interacts with any subsystem(s), you should pass them into "addRequirements()"
        //This function makes it so your command will only run once these subsystem(s) are free from other commands.
        //This is really important as it will stop scenarios where two commands try to controll a motor at the same time.
        addRequirements(m_visionSubsystem, m_driveSubsystem);
    }



    /*Like Robot.java, there are a series of functions that you can override to give the command functionality. */
    

    /*This function is called once when the command is schedueled.
     * If you are overriding "isFinished()", you should probably use this to set m_complete to false in case a command object is 
     * called a second time.
     */
    //When not overridden, this function is blank.
    @Override
    public void initialize(){
        m_visionSubsystem.setPipeline(VisionConstants.kAprilTagPipeline);
        angleController.reset();
        m_complete = false;
    }

    /*This function is called repeatedly when the schedueler's "run()" function is called.
     * Once you want the function to end, you should set m_complete to true.
     */
   @Override
    public void execute(){
        double angle = m_driveSubsystem.getHeading(); //navx
        
        Translation2d pos1 = m_driveSubsystem.getPose().getTranslation(); // Position of robot on field
        Translation2d pos2 = new Translation2d(FieldConstants.kSpeakerX, FieldConstants.kSpeakerY); //speaker position 
        Rotation2d angleToTarget = OdometryUtils.anglePoseToPose(pos1, pos2); // Angle to make robot face speacker
        double distanceToTarget = OdometryUtils.getDistacnePosToPos(pos1, pos2); //distance in inches from limelight to speaker
        Shuffleboard.getTab("Vision").add("Angle to Goal", angleToTarget.getDegrees());
        Shuffleboard.getTab("Vision").add("Distance to Goal", distanceToTarget);


        angleController.setSetpoint(angleToTarget.getDegrees());

        /* 
            get shooter to be at the same angle
            1. Get absolute encoder value from the shooter 
            2. Move the shooter to meet that pot value, do this by using PID to go to the angleToTarget
            3. Shoot the note at the fastest possible speed, lets see how it works
            */
        double encoderValue = 0;
        double offset = 1; 
        double speedAngleChange = 0;
        double maxDistanceShot = 20;

        double rotation = angleController.calculate(angle); //speed needed to rotate robot to set point

        rotation = MathUtil.clamp(rotation, HeadingConstants.kHeadingMinOutput, HeadingConstants.kHeadingMaxOutput); // clamp value (speed limiter)
        
        m_driveSubsystem.drive(
            -MathUtil.applyDeadband(m_xSpeed.getAsDouble(), OIConstants.kDriveDeadband),
            -MathUtil.applyDeadband(m_ySpeed.getAsDouble(), OIConstants.kDriveDeadband),
            rotation,
            true, true
        );
            
    }

    /*This function is called once when the command ends.
     * A command ends either when you tell it to end with the "isFinished()" function below, or when it is interupted.
     * Whether a command is interrupted or not is determined by "boolean interrupted."
     * Things initialized in "initialize()" should be closed here.
     */
    @Override
    public void end(boolean interrupted){
    
    }

    @Override
    public boolean isFinished(){
        return m_complete;
    }
}