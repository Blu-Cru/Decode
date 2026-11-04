package org.firstinspires.ftc.teamcode.blucru.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

import org.firstinspires.ftc.teamcode.blucru.common.commands.ParallelizeIntakeCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.elevator.ElevatorDownCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.elevator.ElevatorMiddleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.elevator.ElevatorUpCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.outtake.shooter.shooterCommands.SetHoodAngleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.outtake.shooter.shooterCommands.ShootWithVelocityCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.outtake.shooter.shooterCommands.TurnOffShooterCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.outtake.turret.turretCommands.CenterTurretCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.transfer.transferCommands.AllTransferDownCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.transfer.transferCommands.AllTransferMiddleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.transfer.transferCommands.LeftTransferUpCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.transfer.transferCommands.MiddleTransferUpCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.transfer.transferCommands.RightTransferUpCommand;
import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;

/**
 * One-controller indoor demo. Same drive and intake -> transfer -> shoot flow as Tele, but with
 * no auto-aim, a low fixed shooter velocity, and a manually aimed turret after transferring.
 *
 * No camera or Limelight is added here, so nothing on this path may use AutoAimCommand,
 * LockOnGoalCommand, TransferCommand or UnshootCommand: they all depend on the turret cam.
 */
@Config
@TeleOp(name = "Demo", group = "norm")
public class Demo extends BluLinearOpMode {

    public static double SHOOTER_VEL = 600;
    public static double HOOD_ANGLE = 26;
    public static double TURRET_POWER = 0.2;
    public static double TURRET_LIMIT_DEG = 90;
    // Set to -1 if dpad left/right turn the turret the wrong way.
    public static double TURRET_DIRECTION = 1;

    private static final int RUMBLE_MS = 200;
    private static final int READY_RUMBLE_MS = 500;

    public enum State {
        INTAKING,
        TRANSFERRING,
        AIMING,
        RESETTING
    }

    private StateMachine sm;
    // Set by the last step of each sequence, so buttons can't interrupt one partway through.
    private boolean sequenceDone = false;
    private boolean readyRumbled = false;

    @Override
    public void initialize() {
        reportTelemetry = true;
        manageTelemetry = true;
        robot.clear();
        addSixWheel();
        addTurret();
        addIntake();
        addElevator();
        addTransfer();
        addShooter();
        CommandScheduler.getInstance().reset();

        sm = new StateMachineBuilder()

                .state(State.INTAKING)
                .loop(() -> {
                    if (gamepad1.left_trigger > 0.2) {
                        intake.setIn();
                    } else if (gamepad1.right_trigger > 0.2) {
                        intake.setOut();
                    } else {
                        intake.setPID();
                    }
                })
                .transition(() -> driver1.pressedLeftBumper(), State.TRANSFERRING, this::startTransfer)

                .state(State.TRANSFERRING)
                .transition(() -> sequenceDone, State.AIMING, () -> readyRumbled = false)
                .transition(() -> driver1.pressedB(), State.RESETTING, this::startCancel)

                .state(State.AIMING)
                .transition(() -> driver1.pressedRightBumper(), State.RESETTING, this::startShoot)
                .transition(() -> driver1.pressedB(), State.RESETTING, this::startCancel)

                .state(State.RESETTING)
                .transition(() -> sequenceDone, State.INTAKING)

                .build();

        sm.setState(State.INTAKING);

        elevator.setUp();
        elevator.write();
        elevator.setDown();
        elevator.write();
        transfer.setAllDown();
        transfer.write();
        turret.setAngle(0);
        turret.write();

        sm.start();
    }

    @Override
    public void initializePeriodic() {
        telemetry.addLine("DRIVE: LEFT STICK + RIGHT STICK");
        telemetry.addLine("INTAKE: HOLD LEFT TRIGGER");
        telemetry.addLine("EJECT: HOLD RIGHT TRIGGER");
        telemetry.addLine("TRANSFER: LEFT BUMPER");
        telemetry.addLine("AIM TURRET (AFTER TRANSFER): DPAD LEFT/RIGHT");
        telemetry.addLine("SHOOT: RIGHT BUMPER");
        telemetry.addLine("CANCEL: B");
    }

    @Override
    public void onStart() {
        new ElevatorDownCommand().schedule();
    }

    @Override
    public void periodic() {
        sm.update();

        sixWheel.teleDrive(gamepad1, 0.0001);

        if (sm.getState() == State.AIMING) {
            driveTurretManually();

            if (!readyRumbled && shooter.targetHit()) {
                gamepad1.rumble(READY_RUMBLE_MS);
                readyRumbled = true;
            }
        }
    }

    @Override
    public void telemetry() {
        telemetry.addData("State", sm.getState());
        telemetry.addData("Turret", "%.1f°", turret.getAngle());
        telemetry.addData("Shooter", "%.0f / %.0f / %.0f (target %.0f)",
                shooter.getLeftVel(), shooter.getMiddleVel(), shooter.getRightVel(), SHOOTER_VEL);
        telemetry.addData("Voltage", "%.2f V", Globals.voltage);
    }

    private void driveTurretManually() {
        double power = 0;
        if (gamepad1.dpad_right) {
            power = TURRET_POWER * TURRET_DIRECTION;
        } else if (gamepad1.dpad_left) {
            power = -TURRET_POWER * TURRET_DIRECTION;
        }

        // The turret has no software limits in manual mode, so enforce them here.
        double angle = turret.getAngle();
        if ((angle >= TURRET_LIMIT_DEG && power > 0) || (angle <= -TURRET_LIMIT_DEG && power < 0)) {
            power = 0;
        }

        turret.setPower(power);
    }

    private void startTransfer() {
        gamepad1.rumble(RUMBLE_MS);
        sequenceDone = false;
        new SequentialCommandGroup(
                new ParallelizeIntakeCommand(),
                new AllTransferDownCommand(),
                new CenterTurretCommand(),
                new ShootWithVelocityCommand(SHOOTER_VEL),
                new SetHoodAngleCommand(HOOD_ANGLE),
                // The turret can be far off center after manual aiming, and raising the
                // elevator before it arrives would jam the balls.
                new WaitUntilCommand(turret::atTarget),
                new WaitCommand(30),
                new ElevatorUpCommand(),
                new WaitCommand(330),
                new ElevatorMiddleCommand(),
                new WaitCommand(130),
                new AllTransferMiddleCommand(),
                new InstantCommand(() -> sequenceDone = true)
        ).schedule();
    }

    private void startShoot() {
        gamepad1.rumble(RUMBLE_MS);
        // Manual power persists until overwritten, so stop a held dpad from carrying over.
        turret.setPower(0);
        sequenceDone = false;
        new SequentialCommandGroup(
                new LeftTransferUpCommand(),
                new RightTransferUpCommand(),
                new MiddleTransferUpCommand(),
                new WaitCommand(250),
                resetForIntake()
        ).schedule();
    }

    private void startCancel() {
        gamepad1.rumble(RUMBLE_MS);
        turret.setPower(0);
        sequenceDone = false;
        resetForIntake().schedule();
    }

    // Stops the shooter instead of idling it like ResetForIntakeCommand: idle power spins the
    // flywheels faster than SHOOTER_VEL and would leave them running between shots.
    private SequentialCommandGroup resetForIntake() {
        return new SequentialCommandGroup(
                new AllTransferDownCommand(),
                new CenterTurretCommand(),
                new TurnOffShooterCommand(),
                new ElevatorDownCommand(),
                new InstantCommand(() -> sequenceDone = true)
        );
    }
}
