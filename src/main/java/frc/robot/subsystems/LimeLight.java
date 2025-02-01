package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.RobotContainer;
import frc.robot.constants.LimeLightConstants;
import frc.robot.trobot5013lib.LimelightHelpers;
import webblib.util.RectanglePoseArea;

public class LimeLight extends SubsystemBase {
  /** Creates a new LimeLight. */
  public static final RectanglePoseArea field =
  new RectanglePoseArea(new Translation2d(0.0, 0.0), new Translation2d(16.54, 8.02));
  private NetworkTable table;
  private NetworkTableEntry tx;
  private NetworkTableEntry ty;
  private NetworkTableEntry ta;
  private NetworkTableEntry tv;
  private boolean aprilTagViable;
  private RobotContainer m_robotContainer;
  Alliance alliance;
  private Boolean enable = true;
  private Boolean trust = false;
  private int fieldError = 0;
  private int distanceError = 0;
  private Pose2d botpose;
  private String name;
  private Boolean doRejectUpdate;

  public LimeLight(String name, boolean aprilTagViable) {
    /**
     * tx - Horizontal Offset
     * ty - Vertical Offset 
     * ta - Area of target 
     * tv - Target Visible
     */
    this.name = name;
    this.aprilTagViable = aprilTagViable;

    this.table = NetworkTableInstance.getDefault().getTable(name);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    doRejectUpdate = false;

    CommandSwerveDrivetrain drivebase = RobotContainer.getInstance().getDrivetrain();

    LimelightHelpers.SetRobotOrientation(name, drivebase.getRotation3d().getAngle(), 0, 0, 0, 0, 0);
    LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);
    if(mt2 != null){
      //if(Math.abs(drivebase.getPigeon2().) > 720) // if our angular velocity is greater than 720 degrees per second, ignore vision updates     //{
      //  doRejectUpdate = true;
      //}
      if(mt2.tagCount == 0)
      {
        doRejectUpdate = true;
      }
      if(!doRejectUpdate)
      {
        drivebase.addVisionMeasurement(
            mt2.pose,
            mt2.timestampSeconds);
      }
    }
  }

  public void setTrust(boolean newTrust){
    trust = newTrust;
  }
  public void setAlliance(Alliance alliance) {
    this.alliance = alliance;
  }
  public double getHorizontalAngleOfErrorDegrees(){
    //+1 is a fudge factor cor camera mounting
    return getTx().getDouble(0.0) + LimeLightConstants.HORIZONTAL_OFFSET;
  }

  public double getVerticalAngleOfErrorDegrees(){
    //+1 is a fudge factor cor camera mounting
    return getTy().getDouble(0.0) + LimeLightConstants.VERTICAL_OFFSET;
  }

  public void setPipelineAprilTag(){
    aprilTagViable = true;
    setPipeline(LimeLightConstants.APRIL_TAG_TARGETING);
  }

  public void setPipelineObjectDecection(){
    aprilTagViable = false;
    setPipeline(LimeLightConstants.GAME_PIECE_RECOGNITION);
  }


  public NetworkTableEntry getTx() {
    return tx;
  }

  public NetworkTableEntry getTy() {
    return ty;
  }

  public NetworkTableEntry getTa() {
    return ta;
  }

  public double getTxAngleRadians() {
    return Units.degreesToRadians(tx.getDouble(0));
  }

  public double getTargetAngleRadians() {
    return getTxAngleRadians()+m_robotContainer.getDrivetrain().getPose().getRotation().getRadians();
  }
  public boolean hasTarget(){
    SmartDashboard.putNumber("tv; ", tv.getDouble(0));
    return tv.getDouble(0) != 0;
  }
  public void setPipeline(int pipeline){
    table.getEntry("pipeline").setNumber(pipeline);
  }
  public Command setPipelineCommand(int pipeline){
    Command result = runOnce(()->setPipeline(pipeline));
    return result;
  }
}


