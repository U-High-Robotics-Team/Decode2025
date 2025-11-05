package org.firstinspires.ftc.teamcode.OfficalCode;
import org.firstinspires.ftc.teamcode.OfficalCode.DecodeTeleOp.RobotStates;

public class RobotTarget {
    public double x;
    public double y;
    public double heading;
    public double time;

    // Use the RobotState enum from MultiTaskAuto
    public DecodeTeleOp.RobotStates state;

    public RobotTarget(double x, double y, double heading, double time, DecodeTeleOp.RobotStates state) {
        this.x = x;
        this.y = y;
        this.heading = heading;
        this.time = time;
        this.state = state;
    }
}