package teamcode;

import virtual_robot.controller.LinearOpMode;
import virtual_robot.util.navigation.AngleUnit;
import virtual_robot.util.navigation.AxesOrder;
import virtual_robot.util.navigation.AxesReference;
import virtual_robot.util.navigation.Orientation;

import java.util.ArrayList;
import java.util.List;

import static java.lang.Math.*;
import static teamcode.MathFunctions.*;

public class RobotMovement extends LinearOpMode {
    Hardware robot = new Hardware();
    static final double     P_TURN_COEFF           = 0.15;     // Larger is more responsive, but also less stable
    private double mindistanceToTarget = 20;

    public ArrayList<CurvePoint> followCurve(ArrayList<CurvePoint> allPoints, double followAngle){
        CurvePoint followMe = getFollowPointPath(allPoints,allPoints.get(0).followDistance);
        telemetry.addData("followMe:","X => " + followMe.x + " Y => " + followMe.y );

        return gotoPosition(followMe.x,followMe.y,followMe.moveSpeed,followAngle,followMe.turnSpeed,allPoints);
    }

    public CurvePoint getFollowPointPath(ArrayList<CurvePoint> pathPoints,double followRadius){

        CurvePoint followMe = new CurvePoint(pathPoints.get(0));

        for(int i = 0; i < pathPoints.size() - 1; i++){
            telemetry.addData("i",i);
            CurvePoint startLine = pathPoints.get(0);//            CurvePoint startLine = pathPoints.get(i);
            CurvePoint endLine = pathPoints.get(1);//             CurvePoint endLine = pathPoints.get(i + 1);
            List<Point> intersections = getCircleLineIntersectionPoint(bot.getPoint(),followRadius,startLine.toPoint(),endLine.toPoint());
            //telemetry.addData("intersections.size()",intersections.size());
            double longestDistance = 10000000;

            for(Point thisIntersection : intersections){
                double distance = abs(dist2D(thisIntersection,bot.getPoint()));

                if(distance < longestDistance){
                    telemetry.addData("thisIntersection",new Point(thisIntersection.x,thisIntersection.y));
                    longestDistance = distance;
                    followMe.setPoint(thisIntersection);
                }
            }
        }
        return followMe;
    }

    @Override
    public void runOpMode() {
    }

    /**
     *  @param x
     * @param y
     * @param movementSpeed
     * @param preferredAngle
     * @param turnSpeed
     * @return
     */
    public ArrayList<CurvePoint> gotoPosition(double x, double y, double movementSpeed, double preferredAngle, double turnSpeed , ArrayList<CurvePoint> path){

        double distanceToTarget = hypot(x - botX(),y - botY());
        telemetry.addData("distanceToTarget",distanceToTarget);
        double absoluteAngleToTarget = atan2(y - botY(), x - botX());
        // telemetry.addData("absoluteAngleToTarget", absoluteAngleToTarget);
        double relativeAngleToPoint = AngleWrap(absoluteAngleToTarget - (bot.getHeadingRadians() - toRadians(90)));
        // telemetry.addData("relativeAngleToPoint", relativeAngleToPoint);
        double relativeXToPoint = sin(relativeAngleToPoint) * distanceToTarget;
        //telemetry.addData("relativeXToPoint", relativeXToPoint);
        double relativeYToPoint = cos(relativeAngleToPoint) * distanceToTarget;
        //telemetry.addData("relativeYToPoint", relativeYToPoint);


        double movementXPower = relativeYToPoint / (abs(relativeXToPoint) + abs(relativeYToPoint));
        //telemetry.addData("movementXPower", movementXPower);
        double movementYPower = relativeXToPoint / (abs(relativeXToPoint) + abs(relativeYToPoint));
        //telemetry.addData("movementYPower", movementYPower);

        movementXPower *= movementSpeed;
        movementYPower *= movementSpeed;
        drive(movementXPower, movementYPower);
//            double relativeTurnAngle = relativeAngleToPoint - toRadians(180) + preferredAngle;
//            turn(relativeTurnAngle, Range.clip(relativeTurnAngle / toRadians(30), -1, 1) * turnSpeed);
        if(distanceToTarget < mindistanceToTarget){
            for(int i = 0; i < path.size();i++){
                if(path.get(i).toPoint().equals(new Point(x, y))){
                    path.remove(path.get(i));
                }
            }
        }
        return path;

    }

    /**
     *
     * @param angle
     * @param power
     */
    private void turn(double angle,double power) {
        double leftfrontSpeed;
        double rightfrontSpeed;
        double leftbackSpeed;
        double rightbackSpeed;

        double error;
        double steer;;

        error = getError(angle);
        steer = getSteer(error, P_TURN_COEFF);
        steer = limit(steer,power);
        leftfrontSpeed = robot.frontLeft.getPower() + steer;
        rightfrontSpeed = robot.frontRight.getPower() - steer;
        leftbackSpeed = robot.backLeft.getPower() + steer;
        rightbackSpeed = robot.backRight.getPower() - steer;


        robot.frontLeft.setPower(leftfrontSpeed);
        robot.frontRight.setPower(rightfrontSpeed);
        robot.backLeft.setPower(leftbackSpeed);
        robot.backRight.setPower(rightbackSpeed);
    }

    /**
     *
     * @param powerX
     * @param powerY
     */
    public void drive(double powerX, double powerY){


        double power = hypot(powerX,powerY);
        double Degree = getHeading() + atan2(powerY,powerX);

        telemetry.addData("befor","X - " + powerX + "\n Y - " + powerY);
        powerY = power * sin(Degree);
        powerX = power * cos(Degree);

        telemetry.addData("after","X - " + powerX + "\n Y - " + powerY);

        robot.backLeft.setPower(powerX);
        robot.frontRight.setPower(powerX);

        robot.backRight.setPower(powerY);
        robot.frontLeft.setPower(powerY);
    }


    /**
     *
     * @param targetAngle
     * @return
     */
    public double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - getHeading();
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     * @param error   Error angle in robot relative degrees
     * @param PCoeff  Proportional Gain Coefficient
     * @return
     */
    public double getSteer(double error, double PCoeff) {
        return error * PCoeff;
    }

    public double getHeading(){
        Orientation orientation = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        return orientation.firstAngle;
    }

    public double botX(){

        return bot.getPoint().x + 283.5;
    }

    public double botY(){
        return abs(bot.getPoint().y - 283.5);
    }

    /**
     *
     * @param value
     * @param maxmin
     * @return
     */
    public double limit(double value,double maxmin) {
        return max(-maxmin, min(value, maxmin));
    }

}
