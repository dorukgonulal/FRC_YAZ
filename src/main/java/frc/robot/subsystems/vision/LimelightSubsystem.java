package frc.robot.subsystems.vision;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import java.util.LinkedList;

import edu.wpi.first.math.util.Units;

public class LimelightSubsystem extends SubsystemBase {

    private NetworkTable limelightTable;

    private boolean isConnected;
    private long lastUpdatedTime;
    private long lastTimeChecked;
    private final int windowSize = 5;
    private final LinkedList<Double> yawValues = new LinkedList<>();

    public LimelightSubsystem() {
        limelightTable = NetworkTableInstance.getDefault().getTable("limelight-reef");

        SmartDashboard.putBoolean("Valid Target", hasValidTarget());

        intializeLimelight(1);

        
        isConnected = true;
    }

    public void intializeLimelight(int pipeline) {

        //setLedMode(3);
        setCamMode(0);
        setUltraPipeline(pipeline);
        //setPiPMode(2);

    }

    /**
     * Wether the Limelight has a valid target
     * 
     * @return A boolean that is true when the Limelight has a valid target.
     */
    public boolean hasValidTarget() {

        return limelightTable.getEntry("tv").getDouble(0) == 1;

    }

    /**
     * Gets horizontal offset of the Limelight's crosshair
     * 
     * @return The horizontal offset of the Limelight's crosshair.
     */

    public double getTargetVerticalOffset() {
            // Limelight'ın ty değerini NetworkTable'dan al
            return NetworkTableInstance.getDefault()
                    .getTable("limelight-reef")
                    .getEntry("ty")
                    .getDouble(0.0); // Eğer değer yoksa 0.0 döner
        }
    
    

    /**
     * Gets vertical offset of the Limelight's crosshair
     * 
     * @return The vertical offset of the Limelight's crosshair.
     */

    public double getTargetHorizontalOffset() {
            // Limelight'ın tx değerini NetworkTable'dan al
            return NetworkTableInstance.getDefault()
                    .getTable("limelight-reef")
                    .getEntry("tx")
                    .getDouble(0.0); // Eğer değer yoksa 0.0 döner
        }
    
    
    

    /**
     * Gets the targets area in relation the the image size
     * 
     * @return The area of the target in relationship to the image size.
     */
    public double getTargetArea() {

        return limelightTable.getEntry("ta").getDouble(0.0);

    }

    /**
     * Gets distance from the front of the shooter to the target.
     * 
     * @return The distance from the shooter to the LimeLight target.
     */
    // public double getTargetDistance() {
    //     NetworkTableEntry ty = limelightTable.getEntry("ty");
    //     double targetOffsetAngle_Vertical = ty.getDouble(0.0);

    //     double angleToGoalDegrees =  Constants.LimelightConstants.limelightMountAngleDegrees + targetOffsetAngle_Vertical;
    //     double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

    //     //calculate distance
    //     double distanceFromLimelightToGoalMeters = (Constants.LimelightConstants.goalHeightMeter - Constants.LimelightConstants.limelightLensHeightMeter) / Math.tan(Constants.LimelightConstants.limelightMountAngleDegrees);

    //     return distanceFromLimelightToGoalMeters; 
    // }



    /**
     * Gets the current camera mode of the Limelight.
     * 
     * @param defaultValue The default value if the NetworkTableEntry could not be
     *                     found.
     * @return The current camera mode of the Limelight.
     */
    public double getCamMode(double defaultValue) {

        return limelightTable.getEntry("camMode").getDouble(defaultValue);
    
    }

    /**
     * Gets the current LED mode of the Limelight.
     * 
     * @param defaultValue The default value if the NetworkTableEntry could not be
     *                     found.
     * @return The current LED mode of the Limelight.
     */
    public double getLedMode(double defaultValue) {

        return limelightTable.getEntry("ledMode").getDouble(defaultValue);

    }

    public boolean getConnected() {

        return isConnected;
    
    }

    /**
     * Sets the camera mode of the Limelight to a given camera mode.
     * 
     * @param camMode The given camera mode to set the LimeLight to.
     */
    public void setCamMode(double camMode) {

        limelightTable.getEntry("camMode").setNumber(camMode);
    
    }

    /**
     * Set the LED mode of the Limelight to a given LED mode.
     * 
     * @param ledMode The given LED mode to set the Limelight to.
     */
    public void setLedMode(double ledMode) {
        limelightTable.getEntry("ledMode").setNumber(ledMode);
    }

    public void setPiPMode(double pipMode) {
        limelightTable.getEntry("stream").setNumber(pipMode);
    }

    public void setPipeline(double pipeline){

        limelightTable.getEntry("pipeline").setNumber(pipeline);

    }

    public void setUltraPipeline(int pipeline) {

		NetworkTableEntry pipelineEntry = limelightTable.getEntry("pipeline");
    	pipelineEntry.setNumber(pipeline);

    }

    static final String sanitizeName(String name) {
        if (name == "" || name == null) {
            return "limelight-reef";
        }
        return name;
    }

    public static NetworkTable getLimelightNTTable(String tableName) {
        return NetworkTableInstance.getDefault().getTable(sanitizeName(tableName));
    }

    public static NetworkTableEntry getLimelightNTTableEntry(String tableName, String entryName) {

        return getLimelightNTTable(tableName).getEntry(entryName);

    }




    public static void setLimelightNTDouble(String tableName, String entryName, double val) {

        getLimelightNTTableEntry(tableName, entryName).setDouble(val);

    }

    public static void setCameraMode_Processor(String limelightName) {

        setLimelightNTDouble(limelightName, "camMode", 0);

    }

    public static void setPipelineIndex(String limelightName, int pipelineIndex) {

        setLimelightNTDouble(limelightName, "pipeline", pipelineIndex);

    }

    public static void setCameraMode_Driver(String limelightName) {

        setLimelightNTDouble(limelightName, "camMode", 1);

    }

    public double getYaw() {
        double[] tagPose = limelightTable.getEntry("targetpose_cameraspace").getDoubleArray(new double[6]);
        return tagPose.length > 4 ? tagPose[4] : 0.0;
    }

    public double getTagXRobotSpace() {
        // targetpose_robotspace => [x, y, z, pitch, yaw, roll]
        double[] pose = limelightTable.getEntry("targetpose_robotspace").getDoubleArray(new double[6]);
        return (pose.length > 0) ? pose[0] : 0.0;
    }
    
    public double getTagYRobotSpace() {
        double[] pose = limelightTable.getEntry("targetpose_robotspace").getDoubleArray(new double[6]);
        return (pose.length > 1) ? pose[1] : 0.0;
    }
    
    public double getTagYawRobotSpace() {
        // Typically index 4 is yaw in degrees
        double[] pose = limelightTable.getEntry("targetpose_robotspace").getDoubleArray(new double[6]);
        return (pose.length > 4) ? pose[4] : 0.0;
    }

    public double getFilteredYaw() {
        double rawYaw = getYaw();
        if (yawValues.size() >= windowSize) {
            yawValues.removeFirst();
        }
        yawValues.add(rawYaw);
        
        double sum = 0;
        for (double val : yawValues) {
            sum += val;
        }
        return sum / yawValues.size();
    }

    public double getTargetDistance() {
        double[] tagPose = limelightTable.getEntry("targetpose_cameraspace").getDoubleArray(new double[6]);
        return tagPose.length > 3 ? tagPose[2] : 0.0;
    }

    public double get3DHorizontalOffset(){
        double[] tagPose = limelightTable.getEntry("targetpose_cameraspace").getDoubleArray(new double[6]);
        return tagPose.length > 1 ? tagPose[0] : 0.0;
    }

    @Override
    public void periodic() {
        isConnected = true;
        long anyLastUpdated = limelightTable.getEntry("ta").getLastChange();
        long current = System.nanoTime();

        SmartDashboard.putNumber("TX:", getTargetHorizontalOffset());

        if (anyLastUpdated == lastUpdatedTime) {
            if (current - lastTimeChecked > 1_000_000_000) {
                isConnected = false;
            }
        }

        else {
            lastUpdatedTime = anyLastUpdated;
            lastTimeChecked = current;
        }

    }
}