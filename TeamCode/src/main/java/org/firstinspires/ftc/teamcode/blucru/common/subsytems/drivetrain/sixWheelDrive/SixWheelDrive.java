package org.firstinspires.ftc.teamcode.blucru.common.subsytems.drivetrain.sixWheelDrive;

import com.acmerobotics.dashboard.config.Config;
import com.seattlesolvers.solverslib.command.Subsystem;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.Robot;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.drivetrain.sixWheelDrive.purePursuit.PurePursuitComputer;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.drivetrain.sixWheelDrive.purePursuit.SixWheelPID;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.outtake.shooter.ShooterAutoAimInterpolation;
import org.firstinspires.ftc.teamcode.blucru.common.util.Alliance;
import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;
import org.firstinspires.ftc.teamcode.blucru.common.util.Point2d;
import org.firstinspires.ftc.teamcode.blucru.common.util.Pose2d;
import org.firstinspires.ftc.teamcode.blucru.common.util.Vector2d;

@Config
public class SixWheelDrive extends SixWheelDriveBase implements Subsystem {
    private double drivePower;
    private Point2d[] path;
    private Double targetHeading; // Target heading for turnTo command
    private Boolean forceReverse = null; // Forced driving direction (null = auto)
    private Double simulatedBatteryVoltage;
    private PurePursuitComputer computer;

    // Look-ahead distance: larger = smoother but wider turns, smaller = tighter but
    // jerkier
    public static double LOOK_AHEAD_DIST = 15.0;
    public static double END_TOLERANCE = 1.5;
    public static double HEADING_TOLERANCE = 5.0; // Degrees
    private SixWheelPID pid;
    private double targetX;

    public SixWheelDrive() {
        super();
        drivePower = 1;
        path = null;
        targetHeading = null;
        simulatedBatteryVoltage = null;
        computer = new PurePursuitComputer();
        pid = new SixWheelPID();
    }

    @Override
    public void init() {
        super.init();
    }

    public void read() {
        super.read();
    }

    public void write() {
        switch (dtState) {
            case IDLE:
                break;
            case PID:
                // Check if we're close enough to the end point
                if (path != null && path.length > 0) {
                    Point2d endPoint = path[path.length - 1];
                    double distToEnd = Math.sqrt(
                            Math.pow(localizer.getPose().getX() - endPoint.getX(), 2) +
                                    Math.pow(localizer.getPose().getY() - endPoint.getY(), 2));

                    if (distToEnd < END_TOLERANCE) {
                        // Close enough - stop
                        switchToIdle();
                        break;
                    }
                }

                double[] powers = computer.computeRotAndXY(path, localizer.getPose(), localizer.getVel(),
                        LOOK_AHEAD_DIST, pid, forceReverse);
                double driveScale = getSimulatedBatteryScale();
                drive(powers[0] * driveScale, -powers[1] * driveScale); // Negate rotation to match TURN convention
                break;
            case TURN:
                // Turn in place to target heading
                if (targetHeading == null) {
                    switchToIdle();
                    break;
                }

                double robotHeadingDeg = Math.toDegrees(localizer.getPose().getH());
                double headingError = targetHeading - robotHeadingDeg;

                // Normalize to [-180, 180]
                while (headingError > 180) {
                    headingError -= 360;
                }
                while (headingError <= -180) {
                    headingError += 360;
                }

                // Check if we're within tolerance
                if (Math.abs(headingError) < HEADING_TOLERANCE) {
                    switchToIdle();
                    break;
                }

                // Calculate rotation command
                double rotVel = pid.getHeadingVelToTargetTurnTo(localizer.getPose(), targetHeading,
                        localizer.getVel().getH());
                drive(0, -rotVel); // No linear movement, only rotation

//                Globals.telemetry.addData("Turn Target", targetHeading);
//                Globals.telemetry.addData("Current Heading", robotHeadingDeg);
//                Globals.telemetry.addData("Heading Error", headingError);
                break;
            case TELE_DRIVE:
                break;
            case LINE_TO_X:
                drive(-pid.lineToX(targetX, localizer.getPose(), localizer.getVel()), 0);
        }

        super.write();
    }

    public void teleDrive(Gamepad g1, double tol) {
        double x = Math.min(cubicScaling(-g1.left_stick_y), 1) * 0.90;
        double r = Math.min(cubicScaling(g1.right_stick_x), 1) * 0.75;

//        double multiplier = Math.min(1, Math.pow((Globals.voltage / 12.5), 2));
//        double multiplier = 1;
        double multiplier = Math.min(1, (Globals.voltage / 12.5));


        if (Math.abs(x) <= tol) {
            x = 0;
        }
        if (Math.abs(r) <= tol) {
            r = 0;
        }

        if (x == 0 && r == 0) {
            if (dtState == State.PID) {
                // in pid
            } else {
                // either stopped driving or idle alr
                dtState = State.IDLE;
                drive(0, 0);
            }
        } else {
            dtState = State.TELE_DRIVE;
            double[] powers = SixWheelKinematics.getPowers(x * multiplier, r * multiplier);
            double left = powers[0] * drivePower;
            double right = powers[1] * drivePower;
            drive((left + right) / 2, (left - right) / 2);
        }

    }

    public void setDrivePower(double power) {
        this.drivePower = power;
    }

    public double getDrivePower() {
        return drivePower;
    }

    public void setSimulatedBatteryVoltage(Double simulatedBatteryVoltage) {
        this.simulatedBatteryVoltage = simulatedBatteryVoltage;
    }

    public void clearSimulatedBatteryVoltage() {
        simulatedBatteryVoltage = null;
    }

    private double getSimulatedBatteryScale() {
        if (simulatedBatteryVoltage == null) {
            return 1.0;
        }

        double rawVoltage = Robot.getInstance().getRawVoltage();
        if (rawVoltage <= 0) {
            return 1.0;
        }

        return Math.min(1.0, simulatedBatteryVoltage / rawVoltage);
    }

    public void followPath(Point2d[] path) {
        followPath(path, null);
    }

    public void followPath(Point2d[] path, Boolean reverse) {
        this.path = path;
        this.targetHeading = null;
        this.forceReverse = reverse;
        computer.resetLastFoundIndex();
        pid.resetBackwardsDrivingState();
        dtState = State.PID;
    }

    /**
     * Turn in place to a target heading
     *
     * @param headingDegrees Target heading in degrees (0 = right, 90 = up, 180 =
     *                       left, -90 = down)
     */
    public void turnTo(double headingDegrees) {
        this.targetHeading = headingDegrees;
        this.path = null;
        dtState = State.TURN;
    }

    /**
     * Check if the turn is complete
     *
     * @return true if robot is within HEADING_TOLERANCE of target heading
     */
    public boolean isTurnComplete() {
        if (dtState != State.TURN || targetHeading == null) {
            return true; // Not turning
        }

        double robotHeadingDeg = Math.toDegrees(localizer.getPose().getH());
        double headingError = targetHeading - robotHeadingDeg;

        // Normalize to [-180, 180]
        while (headingError > 180) {
            headingError -= 360;
        }
        while (headingError <= -180) {
            headingError += 360;
        }

        return Math.abs(headingError) < HEADING_TOLERANCE;
    }

    public void switchToIdle() {
        clearSimulatedBatteryVoltage();
        drive(0, 0);
        dtState = State.IDLE;
    }

    public void setPosition(Pose2d pose) {
        localizer.setPosition(pose);
    }

    public void setXY(Vector2d xy) {
        localizer.setPosition(xy.getX(), xy.getY(), localizer.getHeading());
    }

    public void setHeading(double heading) {
        localizer.setHeading(heading);
    }

    public SixWheelPID getPID() {
        return pid;
    }

    public PurePursuitComputer getPurePursuitComputer() {
        return computer;
    }

    public double getLookAheadDist() {
        return LOOK_AHEAD_DIST;
    }

    @Override
    public void telemetry(Telemetry telemetry) {
        super.telemetry(telemetry);
    }

    public double cubicScaling(double value) {
        return 64 / 27.0 * value * value * value;
    }

    public void updatePID() {
        pid.updatePID();
    }

    public void lineToX(double x) {
        targetX = x;
        dtState = State.LINE_TO_X;
    }

    public State getState() {
        return dtState;
    }

    public Pose2d getVelPose() {
        //double[] interpolationData = ShooterAutoAimInterpolation.interpolateMiddle(Robot.getInstance().turretCam.getTagDistToMiddleShooter());
        double time = 1;
        double x = localizer.getPose().getX() + (time * localizer.getVel().getX());
        double y = localizer.getPose().getY() + (time * localizer.getVel().getY());
        double h = localizer.getHeading();
        return new Pose2d(x, y, h);
    }
    public double getTimeInAir(double[] interpolation) {
        double dist = localizer.getPose().getDistTo(new Pose2d(Globals.shootingGoalLPose,0));
        if (Globals.alliance == Alliance.RED){
            dist = localizer.getPose().getDistTo(new Pose2d(Globals.shootingGoalRPose,0));
        }
        return dist / (Robot.getInstance().shooter.getBallExitVel(interpolation[0]) * Math.cos(Math.toRadians(interpolation[1]))) ;
    }
}
