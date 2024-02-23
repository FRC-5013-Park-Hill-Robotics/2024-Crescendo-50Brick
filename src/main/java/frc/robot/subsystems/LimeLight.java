package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.RobotContainer;
import frc.robot.constants.LimelightConstants;
import frc.robot.trobot5013lib.LimelightHelpers;
import webblib.util.RectanglePoseArea;

public class Limelight extends SubsystemBase {
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
<<<<<<< Updated upstream
  public Limelight(String name, boolean aprilTagViable) {
=======
  private final DoubleArrayPublisher limelightPub;
  public LimeLight(String name, boolean aprilTagViable) {
>>>>>>> Stashed changes
    /**
     * tx - Horizontal Offset
     * ty - Vertical Offset 
     * ta - Area of target 
     * tv - Target Visible
     */
    this.name = name;
    this.table = NetworkTableInstance.getDefault().getTable(name);
    this.limelightPub = table.getDoubleArrayTopic("llPose").publish();
    this.aprilTagViable = aprilTagViable;

    this.tx = table.getEntry("tx");
    this.ty = table.getEntry("ty");
    this.ta = table.getEntry("ta");
    this.tv = table.getEntry("tv");
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

     //read values periodically
    double x = this.tx.getDouble(0.0);
    double y = this.ty.getDouble(0.0);
    double area = this.ta.getDouble(0.0);


    //post to smart dashboard periodically
    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", area);


    if (aprilTagViable ) {
<<<<<<< Updated upstream

      CommandSwerveDrivetrain drivebase = RobotContainer.getInstance().getDrivetrain();
      LimelightHelpers.Results result =
            LimelightHelpers.getLatestResults(name).targetingResults;
        if (!(result.botpose[0] == 0 && result.botpose[1] == 0)) {
          if (alliance == Alliance.Blue) {
            botpose = LimelightHelpers.toPose2D(result.botpose_wpiblue);
          } else if (alliance == Alliance.Red) {
            botpose = LimelightHelpers.toPose2D(result.botpose_wpired);
          }
          if (botpose != null){
          if (field.isPoseWithinArea(botpose)) {
            if (drivebase.getPose().getTranslation().getDistance(botpose.getTranslation()) < 0.33
                || trust || result.targets_Fiducials.length > 1) {
              SmartDashboard.putBoolean("Printing", true);
              drivebase.addVisionMeasurement(
                  botpose,
                  Timer.getFPGATimestamp()
                      - (result.latency_capture / 1000.0)
                      - (result.latency_pipeline / 1000.0),
                  true,
                  1.0);
              
            } else {
              distanceError++;
              SmartDashboard.putNumber("Limelight Error", distanceError);
            }
          } else {
            fieldError++;
            SmartDashboard.putBoolean("Printing", false);
            SmartDashboard.putNumber("Field Error", fieldError);
=======
      CommandSwerveDrivetrain drivetrain = RobotContainer.getInstance().getDrivetrain();
      Double targetDistance = LimelightHelpers.getTargetPose3d_CameraSpace(name).getTranslation().getDistance(new Translation3d());
      System.out.println("Target Distance:" + targetDistance);
      // Tune this for your robot around how much variance you see in the pose at a given distance
      Double confidence = 1 - ((targetDistance - 1) / 6);
      LimelightHelpers.Results result =
          LimelightHelpers.getLatestResults(name).targetingResults;
      if (result.valid) {
        System.out.println("Result is valid");
        botpose = LimelightHelpers.getBotPose2d_wpiBlue(name);
        limelightPub.set(new double[] {
          botpose.getX(),
          botpose.getY(),
          botpose.getRotation().getDegrees()
        });
        SmartDashboard.putNumber("X Limelight Botpose", botpose.getX());
        SmartDashboard.putNumber("Y Limelight Botpose", botpose.getY());

        if (field.isPoseWithinArea(botpose)) {
          System.out.println("Pose is within area");
          if (drivetrain.getState().Pose.getTranslation().getDistance(botpose.getTranslation()) < 0.5
              || trust
              || result.targets_Fiducials.length > 1) {
            System.out.println("Able to do vision measurement");
            drivetrain.addVisionMeasurement(
                botpose,
                Timer.getFPGATimestamp()
                    - (result.latency_capture / 1000.0)
                    - (result.latency_pipeline / 1000.0),
                VecBuilder.fill(confidence, confidence, .01));
          } else {
            distanceError++;
            SmartDashboard.putNumber("Limelight Error", distanceError);
>>>>>>> Stashed changes
          }
        } else {
          fieldError++;
          SmartDashboard.putNumber("Field Error", fieldError);
        }
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
    return getTx().getDouble(0.0) + LimelightConstants.HORIZONTAL_OFFSET;
  }

  public double getVerticalAngleOfErrorDegrees(){
    //+1 is a fudge factor cor camera mounting
    return getTy().getDouble(0.0) + LimelightConstants.VERTICAL_OFFSET;
  }

  public void setPipelineAprilTag(){
    aprilTagViable = true;
    setPipeline(LimelightConstants.APRIL_TAG_TARGETING);
  }

  public void setPipelineObjectDecection(){
    aprilTagViable = false;
    setPipeline(LimelightConstants.GAME_PIECE_RECOGNITION);
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
<<<<<<< Updated upstream
}


=======
}
>>>>>>> Stashed changes
