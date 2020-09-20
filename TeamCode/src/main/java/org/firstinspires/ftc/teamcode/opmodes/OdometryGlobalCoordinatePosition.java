package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.hardware.JSTEncoder;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
/*
Made by Margaret Thatcher
 */
public class OdometryGlobalCoordinatePosition implements Runnable{
    //Odometry wheels
    private JSTEncoder left, right, center;

    //Thead run condition
    private boolean running = true;

    //Position variables used for storage and calculations
    double leftPosition = 0, rightPosition = 0, centerPosition = 0, orientationChange = 0;
    private double previousLeftPosition = 0, previousRightPosition = 0, previousCenterPosition = 0;
    private double robotXCoordinate = 0, robotYCoordinate = 0, robotOrientation = 0;

    //Algorithm constants
    private double robotEncoderWheelDistance;
    private double encoderWheelDistance;
    private double horizontalEncoderTickPerDegreeOffset;
    private double horizontalOffset;

    //Sleep time interval (milliseconds) for the position update thread
    private int sleepTime;

    //Files to access the algorithm constants
    private File wheelBaseSeparationFile = AppUtil.getInstance().getSettingsFile("wheelBaseSeparation.txt");
    private File horizontalTickOffsetFile = AppUtil.getInstance().getSettingsFile("horizontalTickOffset.txt");


    /**
     * Constructor for GlobalCoordinatePosition Thread
     * @param left :left odometry encoder, facing the vertical direction
     * @param right :right odometry encoder, facing the vertical direction
     * @param center :horizontal odometry encoder, perpendicular to the other two odometry encoder wheels
     * @param threadSleepDelay delay in milliseconds for the GlobalPositionUpdate thread (50-75 milliseconds is suggested)
     */

    public OdometryGlobalCoordinatePosition(JSTEncoder left, JSTEncoder right, JSTEncoder center, double COUNTS_PER_INCH, int threadSleepDelay){
        this.left = left;
        this.right = right;
        this.center = center;
        sleepTime = threadSleepDelay;
        robotEncoderWheelDistance = Double.parseDouble(ReadWriteFile.readFile(wheelBaseSeparationFile).trim()) * COUNTS_PER_INCH;
        this.horizontalEncoderTickPerDegreeOffset = Double.parseDouble(ReadWriteFile.readFile(horizontalTickOffsetFile).trim());
    }

    /**
     * Updates the global (x, y, theta) coordinate position of the robot using the odometry encoders
     */
    private void globalCoordinatePositionUpdate(){
        //Get Current Positions
        leftPosition = (left.getCounts());
        rightPosition = (right.getCounts());


        double leftC = leftPosition - previousLeftPosition;
        double rightC = rightPosition - previousRightPosition;


        //Calculate Angle
        orientationChange = (leftC-rightC)/encoderWheelDistance;
        robotOrientation += orientationChange;


        //Get the components of the motion
        centerPosition = (center.getCounts());
        double rawChange = centerPosition - previousCenterPosition;
        double horizontalChange = rawChange - (orientationChange*horizontalOffset);

        double srihith = (rightC + leftC)/2;
        double kanav = horizontalChange;


        //Calculate and update the position values
        robotXCoordinate += (srihith*Math.sin(robotOrientation))+(kanav*Math.cos(robotOrientation));
        robotYCoordinate += (srihith*Math.cos(robotOrientation))-(kanav*Math.sin(robotOrientation));

        previousLeftPosition = leftPosition;
        previousRightPosition = rightPosition;
        previousCenterPosition = centerPosition;

    }

    /**
     * Returns the robot's global x coordinate
     * @return global x coordinate
     */
    public double returnXCoordinate(){ return robotXCoordinate; }

    /**
     * Returns the robot's global y coordinate
     * @return global y coordinate
     */
    public double returnYCoordinate(){ return robotYCoordinate; }

    /**
     * Returns the robot's global orientation
     * @return global orientation, in degrees
     */
    public double returnOrientation(){ return Math.toDegrees(robotOrientation) % 360; }

    /**
     * Stops the position update thread
     */
    public void stop(){ running = false; }


    /**
     * Runs the thread
     */
    @Override
    public void run() {
        while(running) {
            globalCoordinatePositionUpdate();
            try {
                Thread.sleep(sleepTime);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }
}
