package frc.robot.auto;

import frc.robot.auto.TestAuto;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.RobotSubsystem;

import javax.tools.Diagnostic;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class AutoChooser {

    private final DriveSubsystem drive = new DriveSubsystem();
    private final RobotSubsystem robot = new RobotSubsystem();
    
    private final TestAuto testAuto = new TestAuto(drive);

    private final Command m_TestAuto = testAuto;
    private final SendableChooser<Command> m_chooser = new SendableChooser<>();

    public AutoChooser() {

        m_chooser.setDefaultOption("Test Auto", m_TestAuto);

        SmartDashboard.putData(m_chooser);

    }

}
