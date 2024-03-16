package frc.robot.auto;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.RobotSubsystem;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class AutoChooser extends Command{

    private final DriveSubsystem drive;
    private final RobotSubsystem robot;
    
    private final SendableChooser<Command> m_chooser = new SendableChooser<>();

    private Command m_TestAuto;
    private Command m_TwoNoteAuto;

    public AutoChooser(DriveSubsystem drive, RobotSubsystem robot) {

        this.drive = drive;
        this.robot = robot;

        SetAuto();

        m_chooser.setDefaultOption("Test Auto", m_TestAuto);
        m_chooser.addOption("2 Notes", m_TwoNoteAuto);

        SmartDashboard.putData(m_chooser);

    }

    private void SetAuto() {

        final TestAuto testAuto = new TestAuto(drive);
        final TwoNoteAuto twoNoteAuto = new TwoNoteAuto(robot);

        m_TestAuto = testAuto;
        m_TwoNoteAuto = twoNoteAuto;

    }

    public Command GetAuto() {

        return m_chooser.getSelected();

    }

}
