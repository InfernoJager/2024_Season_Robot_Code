package frc.robot.auto;

import frc.robot.auto.autoroutines.AmpSideAuto;
import frc.robot.auto.autoroutines.FourNoteAuto;
import frc.robot.auto.autoroutines.LeaveAuto;
import frc.robot.auto.autoroutines.SourceSideAuto;
import frc.robot.auto.autoroutines.TestAuto;
import frc.robot.auto.autoroutines.ThreeAmpNoteAuto;
import frc.robot.auto.autoroutines.TwoNoteAuto;
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
    private Command m_AmpSideAuto;
    private Command m_SourceSideAuto;
    private Command m_TwoNoteAuto;
    private Command m_ThreeAmpNoteAuto;
    private Command m_FourNoteAuto;
    private Command m_TestAuto;


    public AutoChooser(DriveSubsystem drive, RobotSubsystem robot) {

        this.drive = drive;
        this.robot = robot;

        SetAuto();

        m_chooser.setDefaultOption("CHOOSE AUTO", null);
        m_chooser.addOption("Center Autos:", null);
        m_chooser.addOption("  - 2 Notes", m_TwoNoteAuto);
        m_chooser.addOption("  - 3 Amp Notes", m_ThreeAmpNoteAuto);
        m_chooser.addOption("  - 3 Source Notes", m_ThreeAmpNoteAuto);
        m_chooser.addOption("  - 4 Notes", m_FourNoteAuto);
        m_chooser.addOption("Side Autos:", null);
        m_chooser.addOption("  - Amp Side", m_AmpSideAuto);
        m_chooser.addOption("  - Source Side (WORK IN PROGRESS)", m_SourceSideAuto);
        m_chooser.addOption("Alternate Autos:", null);
        m_chooser.addOption("  - Leave", m_LeaveAuto);
        m_chooser.addOption("  - Test Auto (PROGRAMMING ONLY)", m_TestAuto);

        SmartDashboard.putData(m_chooser);

    }

    private void SetAuto() {

        final LeaveAuto leaveAuto = new LeaveAuto(drive);
        final AmpSideAuto ampSideAuto = new AmpSideAuto(robot);
        final SourceSideAuto sourceSideAuto = new SourceSideAuto(robot, drive);
        final TwoNoteAuto twoNoteAuto = new TwoNoteAuto(robot, drive);
        final ThreeAmpNoteAuto threeNoteAuto = new ThreeAmpNoteAuto(drive, robot);
        final FourNoteAuto fourNoteAuto = new FourNoteAuto(drive, robot);
        final TestAuto testAuto = new TestAuto(drive, robot);

        m_LeaveAuto = leaveAuto;
        m_AmpSideAuto = ampSideAuto;
        m_SourceSideAuto = sourceSideAuto;
        m_TwoNoteAuto = twoNoteAuto;
        m_ThreeAmpNoteAuto = threeNoteAuto;
        m_FourNoteAuto = fourNoteAuto;
        m_TestAuto = testAuto;
        
    }

    public Command GetAuto() {

        return m_chooser.getSelected();

    }

}
