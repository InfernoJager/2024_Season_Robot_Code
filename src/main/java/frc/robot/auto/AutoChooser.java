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

    private Command m_LeaveAuto;
    private Command m_SideAuto;
    private Command m_MirrorSideAuto;
    private Command m_TwoNoteAuto;
    private Command m_ThreeNoteAuto;
    private Command m_FourNoteAuto;
    private Command m_TestAuto;


    public AutoChooser(DriveSubsystem drive, RobotSubsystem robot) {

        this.drive = drive;
        this.robot = robot;

        SetAuto();

        m_chooser.setDefaultOption("Choose Auto", null);
        m_chooser.addOption("Leave", m_LeaveAuto);
        m_chooser.addOption("Left Side Note", m_SideAuto);
        m_chooser.addOption("Right Side Note", m_MirrorSideAuto);
        m_chooser.addOption("2 Notes", m_TwoNoteAuto);
        m_chooser.addOption("3 Notes", m_ThreeNoteAuto);
        m_chooser.addOption("4 Notes", m_FourNoteAuto);
        m_chooser.addOption("Test Auto (PROGRAMMING ONLY)", m_TestAuto);

        SmartDashboard.putData(m_chooser);

    }

    private void SetAuto() {

        final LeaveAuto leaveAuto = new LeaveAuto(drive);
        final SideAuto sideAuto = new SideAuto(robot, drive);
        final MirrorSideAuto mirrorSideAuto = new MirrorSideAuto(robot, drive);
        final TwoNoteAuto twoNoteAuto = new TwoNoteAuto(robot, drive);
        final ThreeNoteAuto threeNoteAuto = new ThreeNoteAuto(drive, robot);
        final FourNoteAuto fourNoteAuto = new FourNoteAuto(drive, robot);
        final TestAuto testAuto = new TestAuto(drive, robot);

        m_LeaveAuto = leaveAuto;
        m_SideAuto = sideAuto;
        m_MirrorSideAuto = mirrorSideAuto;
        m_TwoNoteAuto = twoNoteAuto;
        m_ThreeNoteAuto = threeNoteAuto;
        m_FourNoteAuto = fourNoteAuto;
        m_TestAuto = testAuto;
        
    }

    public Command GetAuto() {

        return m_chooser.getSelected();

    }

}
