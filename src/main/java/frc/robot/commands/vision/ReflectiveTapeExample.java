package frc.robot.commands.vision;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import edu.wpi.first.math.controller.PIDController;

import edu.wpi.first.wpilibj2.command.Command;

public class ReflectiveTapeExample extends Command {

    final VisionSubsystem m_visionSubsystem;
    final DriveSubsystem m_driveSubsystem;
    final PIDController distanceController = new PIDController(.7, 0, 0.1);

    boolean isAlignDistacne = false;
    boolean isAlignRotation = false;

    final double kRotationTolerance = 2;
    final double kRotationSpeed = 0.2;

    /**
     * A rough outline of a command that orbits a position marked by reflective tape.
     * This command is no longer necessary since reflective tape isn't used anymore.
     * @param visionSubsystem
     * @param driveSubsystem
     */
    public ReflectiveTapeExample(VisionSubsystem visionSubsystem, DriveSubsystem driveSubsystem){
        m_driveSubsystem = driveSubsystem;
        m_visionSubsystem = visionSubsystem;
        
        //If your command interacts with any subsystem(s), you should pass them into "addRequirements()"
        //This function makes it so your command will only run once these subsystem(s) are free from other commands.
        //This is really important as it will stop scenarios where two commands try to controll a motor at the same time.
        addRequirements(m_visionSubsystem, m_driveSubsystem);
    }

    /*This function is called once when the command is schedueled.
     * If you are overriding "isFinished()", you should probably use this to set m_complete to false in case a command object is 
     * called a second time.
     */
    //When not overridden, this function is blank.
    @Override
    public void initialize(){
        m_visionSubsystem.setPipeline(3);
    }

    /*This function is called repeatedly when the schedueler's "run()" function is called.
     * Once you want the function to end, you should set m_complete to true.
     */
    @Override
    public void execute() {
        double x = m_visionSubsystem.getX();
        double y = m_visionSubsystem.getY();
        double targets = m_visionSubsystem.getTargets();
        double rotate = 0;
        double orbitSpeed = 0.2; // Adjust this value to control the orbit speed

        if (targets > 0) {
            if (x > kRotationTolerance){
                rotate = kRotationSpeed;
            }
            else if (x < -kRotationTolerance){
                rotate = -kRotationSpeed;
            }

            double forwardSpeed = distanceController.calculate(y, 10); //needs to be implemented

            // Drive the robot with orbit control
            m_driveSubsystem.drive(
                forwardSpeed*.2, 
                orbitSpeed, 
                rotate, 
                true, 
                true);

        } else {
            // No targets, stop the robot
            m_driveSubsystem.drive(0, 0, 0, false, true);
        }
    }    

    /*This function is called once when the command ends.
     * A command ends either when you tell it to end with the "isFinished()" function below, or when it is interupted.
     * Whether a command is interrupted or not is determined by "boolean interrupted."
     * Things initialized in "initialize()" should be closed here.
     */
    @Override
    public void end(boolean interrupted){
        m_driveSubsystem.drive(0, 0, 0, false, false);
    }
}