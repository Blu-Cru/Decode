package org.firstinspires.ftc.teamcode.blucru.common.subsytems.elevator;

import com.acmerobotics.dashboard.config.Config;
import com.seattlesolvers.solverslib.command.Subsystem;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.blucru.common.hardware.BluBrushlandLabsColorRangefinder;
import org.firstinspires.ftc.teamcode.blucru.common.hardware.servo.BluServo;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.BluSubsystem;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.outtake.shooter.ShooterMotifCoordinator;
import org.firstinspires.ftc.teamcode.blucru.common.util.BallColor;

@Config
public class Elevator implements BluSubsystem, Subsystem {
    private BluServo elevatorServoLeft;
    private BluServo elevatorServoRight;
    public static double DOWN_POSITION_LEFT = 0.39;// TODO: find positions
    public static double UP_POSITION_LEFT = 0.6;
    public static double MIDDLE_POSITION_LEFT = 0.48;
    public static double INTAKE_MIDDLE_POSITION_LEFT = 0.42;

    public static double DOWN_POSITION_RIGHT = 0.39;// TODO: find positions
    public static double UP_POSITION_RIGHT = 0.6;
    public static double MIDDLE_POSITION_RIGHT = 0.48;
    public static double INTAKE_MIDDLE_POSITION_RIGHT = 0.43;
    private BluBrushlandLabsColorRangefinder leftSensorRight, leftSensorLeft, middlesensorFront, middlesensorBack, rightSensorLeft,
            rightSensorRight;

    public Elevator() {

        leftSensorRight = new BluBrushlandLabsColorRangefinder("purpleLeftRight", "greenLeftRight");
        leftSensorLeft = new BluBrushlandLabsColorRangefinder("purpleLeftLeft", "greenLeftLeft");
        middlesensorFront = new BluBrushlandLabsColorRangefinder("purpleMiddleFront", "greenMiddleFront");
        middlesensorBack = new BluBrushlandLabsColorRangefinder("purpleMiddleBack","greenMiddleBack");
        rightSensorLeft = new BluBrushlandLabsColorRangefinder("purpleRightLeft", "greenRightLeft");
        rightSensorRight = new BluBrushlandLabsColorRangefinder("purpleRightRight", "greenRightRight");
        elevatorServoLeft = new BluServo("elevatorLeft");
        elevatorServoRight = new BluServo("elevatorRight");
        setDown();
        write();
    }

    @Override
    public void init() {
        elevatorServoLeft.init();
        elevatorServoRight.init();
    }

    @Override
    public void read() {
        elevatorServoLeft.read();
        elevatorServoRight.read();
        leftSensorLeft.read();
        leftSensorRight.read();
        middlesensorFront.read();
        middlesensorBack.read();
        rightSensorRight.read();
        rightSensorLeft.read();
        updateLeftBallColor();
        updateMiddleBallColor();
        updateRightBallColor();
    }

    @Override
    public void write() {
        elevatorServoLeft.write();
        elevatorServoRight.write();
    }

    public void setUp() {
        elevatorServoLeft.setPos(UP_POSITION_LEFT);
        elevatorServoRight.setPos(UP_POSITION_RIGHT);
    }

    public void setDown() {
        elevatorServoLeft.setPos(DOWN_POSITION_LEFT);
        elevatorServoRight.setPos(DOWN_POSITION_RIGHT);
    }

    public void setMiddle() {
        elevatorServoLeft.setPos(MIDDLE_POSITION_LEFT);
        elevatorServoRight.setPos(MIDDLE_POSITION_RIGHT);
    }
    public void setMiddleForIntake() {
        elevatorServoLeft.setPos(INTAKE_MIDDLE_POSITION_LEFT);
        elevatorServoRight.setPos(INTAKE_MIDDLE_POSITION_RIGHT);
    }

    public boolean isFull(){
        return leftBallDetected() && middleBallDetected() && rightBallDetected();
    }

    public boolean leftBallDetected() {
        return leftSensorRight.ballDetected() || leftSensorLeft.ballDetected();
    }

    public boolean middleBallDetected() {
        return middlesensorBack.ballDetected() || middlesensorFront.ballDetected();
    }

    public boolean rightBallDetected() {
        return rightSensorLeft.ballDetected() || rightSensorRight.ballDetected();
    }

    public BallColor getLeftBallColor() {
        if (leftSensorLeft.greenBall() || leftSensorRight.greenBall()) return BallColor.GREEN;
        if (leftSensorLeft.purpleBall() || leftSensorRight.purpleBall()) return BallColor.PURPLE;
        return BallColor.UNKNOWN;
    }

    public BallColor getMiddleBallColor() {
        if (middlesensorBack.greenBall() || middlesensorFront.greenBall()) return BallColor.GREEN;
        if (middlesensorBack.purpleBall() || middlesensorFront.purpleBall()) return BallColor.PURPLE;
        return BallColor.UNKNOWN;
    }

    public BallColor getRightBallColor() {
        if (rightSensorRight.greenBall() || rightSensorLeft.greenBall()) return BallColor.GREEN;
        if (rightSensorRight.purpleBall() || rightSensorLeft.purpleBall()) return BallColor.PURPLE;
        return BallColor.UNKNOWN;
    }

    public void updateLeftBallColor() {
        BallColor targetColor = BallColor.UNKNOWN;
        if (leftSensorLeft.greenBall() || leftSensorRight.greenBall()){
            targetColor = BallColor.GREEN;
        } else if (leftSensorLeft.purpleBall() || leftSensorRight.purpleBall()){
            targetColor = BallColor.PURPLE;
        }

        ShooterMotifCoordinator.setLeftColor(targetColor);
    }

    public void updateMiddleBallColor() {
        BallColor targetColor = BallColor.UNKNOWN;
        if (middlesensorBack.greenBall() || middlesensorFront.greenBall()){
            targetColor = BallColor.GREEN;
        } else if (middlesensorBack.purpleBall() || middlesensorFront.purpleBall()){
            targetColor = BallColor.PURPLE;
        }

        ShooterMotifCoordinator.setMiddleColor(targetColor);
    }

    public void updateRightBallColor() {
        BallColor targetColor = BallColor.UNKNOWN;
        if (rightSensorRight.greenBall() || rightSensorLeft.greenBall()){
            targetColor = BallColor.GREEN;
        } else if (rightSensorRight.purpleBall() || rightSensorLeft.purpleBall()){
            targetColor = BallColor.PURPLE;
        }

        ShooterMotifCoordinator.setRightColor(targetColor);
    }

    public void turnOffElevatorServo() {
        elevatorServoLeft.disable();
        elevatorServoRight.disable();
        // always want to write after a disable
        elevatorServoLeft.write();
        elevatorServoRight.write();
    }

    @Override
    public void telemetry(Telemetry telemetry) {
        telemetry.addData("LEFT slot",   label(getLeftBallColor()));
        telemetry.addData("MIDDLE slot", label(getMiddleBallColor()));
        telemetry.addData("RIGHT slot",  label(getRightBallColor()));
        telemetry.addData("Elevator Full?", isFull());

    }

    private String label(BallColor c) {
        if (c == BallColor.GREEN) return "GREEN";
        if (c == BallColor.PURPLE) return "PURPLE";
        return "NONE";
    }


    @Override
    public void reset() {
        setDown();
    }
}
