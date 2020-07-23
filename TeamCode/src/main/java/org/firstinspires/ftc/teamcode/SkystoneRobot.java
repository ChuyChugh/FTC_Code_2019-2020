package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.Robot;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.hardware.JSTEncoder;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.arcrobotics.ftclib.util.Safety;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.motors.YellowJacket435;
import org.firstinspires.ftc.teamcode.hardware.motors.YellowJacketEx;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Arm;
import org.firstinspires.ftc.teamcode.hardware.subsystems.DoublePulleyLift;
import org.firstinspires.ftc.teamcode.hardware.subsystems.FoundationGrabber;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Grabber;
import org.firstinspires.ftc.teamcode.hardware.subsystems.IntakeTwoWheel;
import org.firstinspires.ftc.teamcode.opmodes.OdometryGlobalCoordinatePosition;

public class SkystoneRobot extends Robot {
    private YellowJacketEx m_frontLeft, m_frontRight, m_backLeft, m_backRight;
    private MotorGroup m_left, m_right;
    private MecanumDrive m_drive;


    private YellowJacket435 m_intakeLeft, m_intakeRight;
    private YellowJacketEx m_leftLifting, m_rightLifting;


    private IntakeTwoWheel m_intake;
    private DcMotor verticalLeft, verticalRight, horizontal;

    private RevIMU m_imu;
    private HolonomicOdometry m_odometry;
    private JSTEncoder m_encoderLeft, m_encoderRight, m_centralEncoder;
    private DoublePulleyLift m_lift;
    private Arm m_arm;
    private Grabber grabber;
    private FoundationGrabber foundationGrabber;

    private SimpleServo s_left, s_right, s_grabber, servoLeft, servoRight;


    private OdometryGlobalCoordinatePosition globalPositionUpdate;
    private Thread positionThread = new Thread(globalPositionUpdate);

    private final double COUNTS_PER_INCH = 145.6 / (4 * Math.PI);




    public SkystoneRobot(HardwareMap hardwareMap){
        m_frontLeft = new YellowJacketEx(
                new YellowJacket435(hardwareMap, "fL")
        );
        m_frontRight = new YellowJacketEx(
                new YellowJacket435(hardwareMap, "fR")
        );
        m_backLeft = new YellowJacketEx(
                new YellowJacket435(hardwareMap, "bL")
        );
        m_backRight = new YellowJacketEx(
                new YellowJacket435(hardwareMap, "bR")
        );
        m_intakeLeft = new YellowJacket435(hardwareMap, "iL");
        m_intakeRight = new YellowJacket435(hardwareMap, "iR");
        s_left = new SimpleServo(hardwareMap,"armL" );
        s_right = new SimpleServo(hardwareMap,"armL" );
        s_grabber = new SimpleServo(hardwareMap, "grabber");
        servoLeft = new SimpleServo(hardwareMap, "foundationL");
        servoRight = new SimpleServo(hardwareMap, "foundationR");

        m_leftLifting = new YellowJacketEx(
                new YellowJacket435(hardwareMap, "leftL")
        );
        m_rightLifting = new YellowJacketEx(
                new YellowJacket435(hardwareMap, "rightL")
        );
        m_left = new MotorGroup(m_frontLeft, m_backLeft);
        m_right = new MotorGroup(m_frontRight, m_backLeft);


        m_drive = new MecanumDrive(m_left, m_right);

        m_intake = new IntakeTwoWheel(m_intakeLeft, m_intakeRight);

        m_imu = new RevIMU(hardwareMap);
        m_odometry = new HolonomicOdometry(16.77); // our trackwidth is 16.77 inches


        m_encoderLeft = new JSTEncoder(hardwareMap, "encoderLeft");
        m_encoderRight = new JSTEncoder(hardwareMap, "encoderRight");
        m_centralEncoder = new JSTEncoder(hardwareMap, "encoderCenter");

        globalPositionUpdate = new OdometryGlobalCoordinatePosition(verticalLeft, verticalRight, horizontal, COUNTS_PER_INCH, 75);



        m_encoderLeft.setDistancePerPulse(2.0 * Math.PI / 8092); // rev through-bore encoder
        m_encoderRight.setDistancePerPulse(2.0 * Math.PI / 8092);
        m_centralEncoder.setDistancePerPulse(2.0 * Math.PI / 8092);


        m_lift = new DoublePulleyLift(m_leftLifting, m_rightLifting);
        grabber = new Grabber(s_grabber);
        m_arm = new Arm(s_left, s_right);
        foundationGrabber = new FoundationGrabber(servoLeft, servoRight);

        m_frontLeft.setMode(MotorEx.RunMode.STOP_AND_RESET_ENCODER);
        m_frontRight.setMode(MotorEx.RunMode.STOP_AND_RESET_ENCODER);
        m_backLeft.setMode(MotorEx.RunMode.STOP_AND_RESET_ENCODER);
        m_backRight.setMode(MotorEx.RunMode.STOP_AND_RESET_ENCODER);

        m_frontLeft.setMode(MotorEx.RunMode.RUN_WITHOUT_ENCODER);
        m_frontRight.setMode(MotorEx.RunMode.RUN_WITHOUT_ENCODER);
        m_backLeft.setMode(MotorEx.RunMode.RUN_WITHOUT_ENCODER);
        m_backRight.setMode(MotorEx.RunMode.RUN_WITHOUT_ENCODER);

        verticalLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        verticalLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        verticalRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        horizontal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        m_frontLeft.setMode(MotorEx.ZeroPowerBehavior.BREAK);
        m_frontRight.setMode(MotorEx.ZeroPowerBehavior.BREAK);
        m_backLeft.setMode(MotorEx.ZeroPowerBehavior.BREAK);
        m_backRight.setMode(MotorEx.ZeroPowerBehavior.BREAK);


    }
    //autonomous driving using odometers
    public void drive(double targetXPosition, double targetYPosition, double robotPower, double desiredRobotOrientation, double allowableDistanceError, double allowableRotationError){
/*
Makes the field into a coordinate system which makes traveling much more efficient. targetXPosition is the X2 coordinate, and targetYPosition is the Y2 coordinate.
 */
        targetXPosition *= COUNTS_PER_INCH;
        targetYPosition *= COUNTS_PER_INCH;
        allowableDistanceError *= COUNTS_PER_INCH;
        double distanceToX = targetXPosition - globalPositionUpdate.returnXCoordinate(); //x distance needed to travel from X1 to X2
        double distanceToY = targetYPosition - globalPositionUpdate.returnYCoordinate();
        double robotMoveAngle = desiredRobotOrientation - globalPositionUpdate.returnOrientation();
        double distance = Math.hypot(distanceToX, distanceToY);
        while(distance > allowableDistanceError){
            distanceToY = targetYPosition - globalPositionUpdate.returnYCoordinate();
            distanceToX = targetXPosition - globalPositionUpdate.returnXCoordinate();
            robotMoveAngle = desiredRobotOrientation - globalPositionUpdate.returnOrientation();
            double movementAngle = Math.toDegrees(Math.atan2(distanceToX, distanceToY));
            //x magnitude vector
            double robotMoveX = calculateX(movementAngle, robotPower);
            //y magnitude vector
            double robotMoveY = calculateY(movementAngle, robotPower);
            if (m_safety == Safety.SWIFT){
                    m_drive.driveRobotCentric(
                            robotMoveX,
                            robotMoveY,
                            0
                            );
            }
        }
        while(robotMoveAngle > allowableRotationError){
            if(m_safety == Safety.SWIFT){
                m_drive.driveRobotCentric(
                        0,
                        0,
                        robotMoveAngle/360 * robotPower
                );
            }
        }

    }
    //teleop driving
    public void drive(double x, double y, double turn, double power, double gyro){
        m_drive.driveFieldCentric(x * power, y * power, turn * power, gyro);
    }

    public void reverseEncoders(){
        globalPositionUpdate.reverseRightEncoder();
        globalPositionUpdate.reverseNormalEncoder();
    }

    public double returnXPosition(){ return globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH; }
    public double returnYPosition() { return globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH; }
    public double orientation(){
        return globalPositionUpdate.returnOrientation();
    }

    public void intake() { m_intake.intake(); }
    public void outtake() {
        m_intake.outtake();
    }
    public void stopIntake()
    {
        m_intake.stop();
    }

    public void armDownPosition(){ m_arm.grabbingPosition(); }
    public void armReleasingPosition(){ m_arm.releasingPosition(); }

    public void grabBlock(){
        grabber.grab();
    }
    public void releaseBlock() {
        grabber.release();
    }

    public void grabFoundation(){
        foundationGrabber.grab();
    }
    public void releaseFoundation(){ foundationGrabber.release(); }

    public double IMUHeading(){return m_imu.getHeading();}


    //autonomous lift
    public void liftToPosition(double position, double speed) { m_lift.liftToPosition(position, speed); }
    //TeleOp lift
    public void lift(double power){ m_lift.lift(power);}

    public void start(){ positionThread.start(); }
    public boolean alive(){ return positionThread.isAlive(); }

    public int verticalLeftPosition(){ return verticalLeft.getCurrentPosition(); }
    public int verticalRightPosition(){ return verticalRight.getCurrentPosition(); }
    public int horizontalPosition(){ return horizontal.getCurrentPosition(); }

    public void disable() {
        m_drive.stopMotor();
        m_lift.disable();
        m_intake.disable();
        m_arm.disable();
        grabber.disable();
        foundationGrabber.disable();
    }
    /*
    These methods are meant for calculating a distance for the robot to move in a vector with an angle and a speed.
    Notice how the calculate x uses sine and y uses cosine. This is due to two reasons. On the unit circle, we want
    PI/2 as our 0 radian instead of PI/2 as default because modeling the robot through autonomous programming would be easier when the forward is 0 radians.
    Second reason: adding pi/2 to sine and cosine would just make sine cosine and cosine sin through the trig identities.
     */
    private double calculateX(double desiredAngle, double speed) {
        return Math.sin(Math.toRadians(desiredAngle)) * speed;

    }

    private double calculateY(double desiredAngle, double speed) {
        return Math.cos(Math.toRadians(desiredAngle)) * speed;
    }

    //This methods reads the odometry module encoders and reads the distance from them.
    public Pose2d getRobotPose() {
        m_odometry.update(
                Math.toRadians(m_imu.getHeading()),
                m_centralEncoder.getDistance(),
                m_encoderLeft.getDistance(),
                m_encoderRight.getDistance()
        );
        return m_odometry.getPose();
    }
}
