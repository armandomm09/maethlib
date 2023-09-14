package frc.lib.maethLib.Swerve.subsystems;


import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.maethLib.Swerve.constants.SwerveModuleConstants;
import frc.lib.maethLib.Swerve.constants.swerveDriveConstants;
import frc.lib.maethLib.Swerve.constants.swerveSpecs;

//Since we have 4 equal modules, we can create a class for the modules
public class SwerveModule {

    //Definition for drive and turn motor
    private final CANSparkMax driveMotor;
    private final CANSparkMax turnMotor;

    //Definition for drive, turn encoders
    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder turnEncoder;

    //Definition for PID turn encoder
    private final PIDController turnPIDController;

    //An int variable where we will keep the number of the module (For debug info)
    private final int moduleNumber;

    //Definition for the absolute encoder and its encoder type
    private final AbsoluteEncoder absoluteEncoder;
   private final SparkMaxAbsoluteEncoder.Type encodeerabstype = SparkMaxAbsoluteEncoder.Type.kDutyCycle;

   private final swerveSpecs specs;

   private SwerveModuleConstants moduleConstants;


   /** Creation of a SwerveModule object
     * @param constants an object of {@SwerveModuleConstants} where we keep all our constants
     * for each module
     * Watch {@MODS}
     * 
     * @param moduleNumber the number of the module for debug info
     **/
    public SwerveModule(swerveDriveConstants constants, int moduleNumber) {

        specs = constants.specs;
        
        switch (moduleNumber) {
            case 1:
                moduleConstants = constants.frontLeftConstants;
                break;
            case 2:
                moduleConstants = constants.frontRightConstants;
                break;
            case 3:
                moduleConstants = constants.rearLeftConstants;
                break;
            case 4: 
                moduleConstants = constants.rearRightConstants;
                break;
        }

        //Initialize all our arguments

        //Module number initialization
        this.moduleNumber = moduleNumber;
        
        //Motors initialization
        driveMotor = new CANSparkMax(moduleConstants.driveMotorID, MotorType.kBrushless);
        driveMotor.restoreFactoryDefaults();
        turnMotor = new CANSparkMax(moduleConstants.turnMotorID, MotorType.kBrushless);
        turnMotor.restoreFactoryDefaults();

        //Set inverted for the motors ***BOOLEAN
        driveMotor.setInverted(moduleConstants.driveMotorInverted);
        turnMotor.setInverted(moduleConstants.turnMotorInverted);

       /** 
        * When driving a Swerve Robot, we will have better 
        * driving if the motors are on brake mode
        */
        driveMotor.setIdleMode(IdleMode.kBrake);
        turnMotor.setIdleMode(IdleMode.kBrake);

        //Initialization for encoders
        driveEncoder = driveMotor.getEncoder();
        turnEncoder = turnMotor.getEncoder();

        //Initialization for absolute encoder
        absoluteEncoder = turnMotor.getAbsoluteEncoder(encodeerabstype);
        absoluteEncoder.setPositionConversionFactor(2 * Math.PI);
        absoluteEncoder.setInverted(moduleConstants.absoluteEncoderReversed);
        absoluteEncoder.setZeroOffset(moduleConstants.absoluteEncoderOffsetRad);

       /**
        * Set the periodic frame rate to 20 ms 
        * for a more constant reading of the turning positon of the module
        */
        turnMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 15);

       /**Set the position and velociity conversion factors  
        * Meters, Meters per Seconds, Radians and Radians per second
        */
        driveEncoder.setPositionConversionFactor(specs.kDriveEncoderRot2Meter);
        driveEncoder.setVelocityConversionFactor(specs.kDriveEncoderRPM2MeterPerSec);
        turnEncoder.setPositionConversionFactor(specs.kTurningEncoderRot2Rad);
        turnEncoder.setVelocityConversionFactor(specs.kTurningEncoderRPM2RadPerSec);

        //Initialize the PID controller
        turnPIDController = new PIDController(specs.kPTurning, 0, 0);
        
       /**Enables continuous input.
        * Rather then using the max and min input range as constraints, it considers them to be the
        * same point and automatically calculates the shortest route to the setpoint
        */
        turnPIDController.enableContinuousInput(-Math.PI, Math.PI);

       /**
        * We reset the position of the encoders when we start the robot 
        * through a void function
        */
        resetEncoders();
    }

   /**A public function to get the drive position in Meters
    * 
    * @return Drive Encoder Position
    */
    public double getDrivePosition() {
        return driveEncoder.getPosition();
    }

   /**A public function to get the turning position in Radians
    * 
    * @return Turn Encoder Position
    */
    public double getTurnPosition() {
        return turnEncoder.getPosition();
    }

   /**A public function to get the drive velocity in Meters Per Second
    * 
    * @return Drive Encoder velocity
    */
    public double getDriveVelocity() {
        return driveEncoder.getVelocity();
    }

   /**A public function to get the turning velocity in Radians per Second
    * 
    * @return Turn Encoder Velocity
    */
    public double getGiroVelocidad() {
        return turnEncoder.getVelocity();
    }

   /**
    * Get the absolute position from the module
    * @return Rotation Gear Absolute Position
    */
    public double getAbsolutePosition(){
        return absoluteEncoder.getPosition();
    }

    /**
    * A public function to get the turning position in the form of Rotation2D
    * 
    * @return Turn Position in Rotation2d
    */
    public Rotation2d get2DTurnPosition(){
        return Rotation2d.fromRadians(getTurnPosition());
    }

   /**
    * A public function to reset the position of the encoders
    */
    public void resetEncoders() {
        driveEncoder.setPosition(0);
        turnEncoder.setPosition(getAbsolutePosition());
    }

   /**
    * A public function to get the Applied output for the drive motor ***Used for debug
    *
    * @return Applied output to Drive Motor
    */
    public double getDriveOutput(){
        return driveMotor.getAppliedOutput();
    }

   /**
    *@return The drive velocity and the turning position in the form of 
    * Swerve Module State (Used by WPI Libraries)
    */
    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurnPosition()));
    }

   /** 
    * @return the drive position and the turning position in the form of 
    * Swerve Module Position (Used for Odometry and Pathplanning)
    */
    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(getDrivePosition(), get2DTurnPosition());
    }

   /**Sets the Module State for the module
    * @param state The desired state for the module
    */
    public void setDesiredState(SwerveModuleState state) {

        //A deadzone for the controll so we do not recieve unwanted inputs
        if (Math.abs(state.speedMetersPerSecond) < 0.005) {
            stop();
            return;
        }


       /**
        * Optimize the path for the turning motor so it doesn't 
        * have to turn more than 90 degrees
        */
        state = SwerveModuleState.optimize(state, getState().angle);
        
        //Set the drive motors output              **The output divided by our max velocity
        driveMotor.set(state.speedMetersPerSecond / specs.kMaxDriveVelocity);

       //Set the turn motor position with PID
        turnMotor.set(turnPIDController.calculate(getTurnPosition(), state.angle.getRadians()));
        SmartDashboard.putString("Modulo numero" + moduleNumber + "state", state.toString());
    }

    //Public method for stopping both motors
    public void stop() {
        driveMotor.set(0);
        turnMotor.set(0);
    }
}
