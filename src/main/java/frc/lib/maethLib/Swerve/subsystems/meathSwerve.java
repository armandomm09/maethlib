/**
 * Writen by Armando Mac Beath
 * 
 * {@MÆTH}
 */
package frc.lib.maethLib.Swerve.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.maethLib.Swerve.constants.swerveDriveConstants;
import frc.lib.maethLib.Swerve.constants.swerveSpecs;

public class meathSwerve extends SubsystemBase {

    
    /**
     * 2D Field for Smart Dashboard visualization
     */
    private final Field2d field = new Field2d();

    //Creation for 4 equal modules whith its constants and module number
    private final SwerveModule frontLeft;

    private final SwerveModule frontRight;

    private final SwerveModule rearLeft;

    private final SwerveModule rearRight;

    //Creation of a gyroscope **NAVX
    private final AHRS gyro = new AHRS(SPI.Port.kMXP);
    
    //Instance of the swerve
    private static meathSwerve instance;

    //Creation of the odometry *** Used in field and pathplanning
     final SwerveDriveOdometry odometry;

     private final swerveDriveConstants constants;

     private final swerveSpecs specs;
    
         

    public meathSwerve(swerveDriveConstants constants) {

        specs = constants.specs;

        this.constants = constants;


        frontLeft = new SwerveModule(constants, 1);
        frontRight = new SwerveModule(constants, 2);
        rearLeft = new SwerveModule(constants, 3);
        rearRight = new SwerveModule(constants, 4);


      odometry  = new SwerveDriveOdometry(
            constants.specs.kinematics, 
            getRotation2d(), 
            new SwerveModulePosition[]{
               frontLeft.getPosition(),
               frontRight.getPosition(),
               rearLeft.getPosition(),
               rearLeft.getPosition()
            });

       //Reset odometry when initianig the subsyste
        odometry.resetPosition(
            getRotation2d(), 
            new SwerveModulePosition[]{
               frontLeft.getPosition(),
               frontRight.getPosition(),
               rearLeft.getPosition(),
               rearLeft.getPosition()
            }, new Pose2d());
    
        
       //A thread so the code runs correctly when it builds on the robot
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading();
            } catch (Exception e) {
            }
        }).start();

    }

    

    //Public method for reseting the navX
    public void zeroHeading() {
        gyro.reset();
    }

   /**
    * Get Heading
    * @return the heading of the robot
    */
    public double getHeading() {
        return Math.IEEEremainder(gyro.getAngle(), 360);
    }

    /**
     * Get rotation 2D
     * @return the heading of the robot in 2D
     */
    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    /**
     * get pose
     * @return the pose of the robot in 2d
     */
    public Pose2d getPose(){
        return odometry.getPoseMeters();
    }

   /**Public method for reseting the odometry
     * @param pose insert a new Pose2d
     */
    public void resetOdometry(Pose2d pose){
        resetEncoders();
        odometry.resetPosition(getRotation2d(), 
        new SwerveModulePosition[]{
            frontLeft.getPosition(),
            frontRight.getPosition(),
            rearLeft.getPosition(),
            rearLeft.getPosition()
         }, pose);
    }

    /**Method for converting Chasssis speeds to {@SwerveModuleStates} and setting
     * them to the modules
     * @param isFieldOriented weather the driving is oriented to the field 
     * @param driveVelocity Driving Input
     * @param strafeVelocity Strafe Input
     * @param rotationVelocity Rotation Input
     */
    public void setChassisSpeeds(ChassisSpeeds chassisSpeeds){

        SwerveModuleState[] moduleStates = constants.specs.kinematics.toSwerveModuleStates(chassisSpeeds);

        // 6. Output each module states to wheels
        this.setModuleStates(moduleStates);
    }
   
    /**
     * Gett Kinematics
     * @return the kinematics of the robor
     */
    public SwerveDriveKinematics getKinematics(){
        return constants.specs.kinematics;
    }
    
    /**
     * get instance
     * @return the instance in use for the swerve
     */
     public static meathSwerve getInstance(swerveDriveConstants constants){
        if(instance == null){
            instance = new meathSwerve(constants);
        }
        return instance;
    }

    
    

    @Override
    public void periodic() {
       
        SmartDashboard.putNumber("Potencia 1", frontLeft.getDriveOutput());
        SmartDashboard.putNumber("Potencia 2", frontRight.getDriveOutput());
        SmartDashboard.putNumber("Potencia 3", rearLeft.getDriveOutput());
        SmartDashboard.putNumber("Potencia 4", rearRight.getDriveOutput());

        SmartDashboard.putNumber("Giro Robot", getHeading());

        //Updating the odometry of the robot
        odometry.update(getRotation2d(), 
        new SwerveModulePosition[]{
            frontLeft.getPosition(),
            frontRight.getPosition(),
            rearLeft.getPosition(),
            rearLeft.getPosition()
         });
 
        //Set the robot pose in the Field2d
        field.setRobotPose(odometry.getPoseMeters());

        //Visualization of the field
        SmartDashboard.putData("Field", field);

        //Debug info of the odometry
        System.out.println(odometry.getPoseMeters());
         
}
    
    //Public method for stoping the modules
    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        rearLeft.stop();
        rearRight.stop();
    }

    //Public method for reseting the encoders
    public void resetEncoders(){
        frontLeft.resetEncoders();
        frontRight.resetEncoders();
        rearLeft.resetEncoders();
        rearRight.resetEncoders();

    }

   /**Public method for setting the desired states into the modules
    * @param desiredStates an array of 4 {@SwerveModuleState} one for each module 
    * in the following order:
    * [0] -> {@frontLeftModule} 
    * [1] -> {@frontRightModule}
    * [2] -> {@rearLeftModule}
    * [3] -> {@rearRightModule}
    */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, constants.specs.kMaxDriveVelocity);
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        rearLeft.setDesiredState(desiredStates[2]);
        rearRight.setDesiredState(desiredStates[3]);
    }

    public swerveSpecs getSwerveSpecs(){
        return specs;
    }
}
