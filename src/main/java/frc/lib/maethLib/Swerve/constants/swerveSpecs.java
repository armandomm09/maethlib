package frc.lib.maethLib.Swerve.constants;

import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

public class swerveSpecs {
    
    //Max velocity for the chassis
    public double kMaxDriveVelocity;

    //Kinematics for the chassis
    public SwerveDriveKinematics kinematics;

    //Acceleration limiter for the driving
    public double driveAccelerationLimiter;

    //Acceleration limiter for the rotation
    public double rotationAccelerationLimiter;

    //Diameter of the Wheel in meters
    public double kWheelDiameterMeters;
    
    //Gear ratio of the drive motor
    public double kDriveMotorGearRatio;

    //Gear ratio of the turning motor
    public double kTurningMotorGearRatio;

    // Drive position conversion factor from rotation to meters
    public double kDriveEncoderRot2Meter;
       
    // Turning position conversion factor from rotation to radias
    public double kTurningEncoderRot2Rad;
   
    // Drive velocity conversion factor from RPM to M/S
    public double kDriveEncoderRPM2MeterPerSec;
   
    // Turning velocity conversion factor from RPM to Rads/Sec
    public double kTurningEncoderRPM2RadPerSec;
   
    //P constant for the turn motor
    public double kPTurning;

    //A deadband so we don't gen unwanted inputs
    public double kDeadband;

    public swerveSpecs(

     double kMaxDriveVelocity,
     SwerveDriveKinematics kinematics,
     double driveAccelerationLimiter,
     double rotationAccelerationLimiter,
     double kWheelDiameterMeters,
     double kDriveMotorGearRatio,
     double kTurningMotorGearRatio,
     double kDriveEncoderRot2Meter,
     double kTurningEncoderRot2Rad,
     double kDriveEncoderRPM2MeterPerSec,
     double kTurningEncoderRPM2RadPerSec,
     double kPTurning,
     double kDeadband


    ){


    this.kMaxDriveVelocity = kMaxDriveVelocity;
    this.kinematics = kinematics;
    this.driveAccelerationLimiter = driveAccelerationLimiter;
    this.rotationAccelerationLimiter = rotationAccelerationLimiter;
    this.kWheelDiameterMeters = kWheelDiameterMeters;
    this.kDriveMotorGearRatio = kDriveMotorGearRatio;
    this.kTurningMotorGearRatio = kTurningMotorGearRatio;
    this.kDriveEncoderRot2Meter = kDriveEncoderRot2Meter;
    this.kTurningEncoderRot2Rad = kTurningEncoderRot2Rad;
    this.kDriveEncoderRPM2MeterPerSec = kDriveEncoderRPM2MeterPerSec;
    this.kTurningEncoderRPM2RadPerSec = kTurningEncoderRPM2RadPerSec;
    this.kPTurning = kPTurning;
    this.kDeadband = kDeadband;
    }



}
