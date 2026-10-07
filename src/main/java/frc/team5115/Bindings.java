package frc.team5115;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.team5115.commands.DriveCommands;
import frc.team5115.subsystems.drive.Drivetrain;
import frc.team5115.subsystems.indexer.Indexer;
import frc.team5115.subsystems.intake.Intake;

public class Bindings {
    private final CommandXboxController driveJoy;
    private final CommandXboxController manipJoy;

    private final Drivetrain drivetrain;
    private final Intake intake;
    
    private final Indexer indexer;
   

    public Bindings(
            Drivetrain drivetrain, Intake intake, Indexer indexer) {
        this.drivetrain = drivetrain;
        this.intake = intake;
        this.indexer = indexer;


        // If in single mode, both Controller objects reference the controller on port 0
        driveJoy = new CommandXboxController(0);
        manipJoy = Constants.SINGLE_MODE ? driveJoy : new CommandXboxController(1);
    }

    public boolean joysticksConnected() {
        return driveJoy.isConnected() && manipJoy.isConnected();
    }

    private Command offsetGyro() {
        return Commands.runOnce(() -> drivetrain.zeroGyro(), drivetrain).ignoringDisable(true);
    }

    /**
     * Automation is enabled UNLESS one of the following conditions is true:
     *
     * <ol>
     *   <li>auto is enabled
     *   <li>the robot is disabled
     *   <li>{@code Constants.DISABLE_AUTOMATION} is true
     * </ol>
     *
     * @return a Trigger that rises when automation enables and falls when automation is disabled.
     */
    public Trigger automationEnabled() {
        // return (driveJoy.povCenter().negate())
        //         .or(manipJoy.povCenter().negate())
        return new Trigger(DriverStation::isAutonomousEnabled)
                .or(DriverStation::isDisabled)
                .or(() -> Constants.DISABLE_AUTOMATION)
                .negate();
    }



    public void configureButtonBindings() {
        final Trigger slowMode = driveJoy.rightBumper();

        drivetrain.setDefaultCommand(
                DriveCommands.joystickDrive(
                        drivetrain,
                        () -> false,
                        slowMode,
                        () -> -driveJoy.getLeftY(),
                        () -> -driveJoy.getLeftX(),
                        () -> -driveJoy.getRightX()));

        // Hold Y to enable intake mode
        // .toggleOnTrue(
        //         DriveCommands.fieldRelativeHeadingDrive(
        //                 drivetrain,
        //                 slowMode,
        //                 () -> -driveJoy.getLeftY(),
        //                 () -> -driveJoy.getLeftX(),
        //                 () -> -driveJoy.getLeftY(),
        //                 () -> -driveJoy.getLeftX()));

        // Hold left bumper to drive robot relative
        driveJoy
                .leftBumper()
                .whileTrue(
                        DriveCommands.joystickDrive(
                                drivetrain,
                                () -> true,
                                slowMode,
                                () -> -driveJoy.getLeftY(),
                                () -> -driveJoy.getLeftX(),
                                () -> -driveJoy.getRightX()));

        driveJoy.x().onTrue(Commands.runOnce(drivetrain::stopWithX, drivetrain));
        driveJoy.start().onTrue(offsetGyro());


        driveJoy.povUp().onTrue(Commands.runOnce(drivetrain::humanOverrideLimit));
        driveJoy.povDown().onTrue(Commands.runOnce(drivetrain::setTeleopCurrentLimits));

        if (Constants.ENABLE_DEFAULT_AGITATION) {
            indexer.setDefaultCommand(indexer.reject());
        }

       
        

    }

//     private Command rumble(double value) {
//         return Commands.runOnce(
//                 () -> {
//                     driveJoy.setRumble(GenericHID.RumbleType.kBothRumble, value);
//                     if (manipJoy != null) {
//                         manipJoy.setRumble(GenericHID.RumbleType.kBothRumble, value);
//                     }
//                 });
//     }
}
