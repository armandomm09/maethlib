package frc.lib.maethLib.Swerve.constants;

public class swerveDriveConstants {

    public SwerveModuleConstants frontLeftConstants;
    public SwerveModuleConstants frontRightConstants;
    public SwerveModuleConstants rearLeftConstants;
    public SwerveModuleConstants rearRightConstants;
    public swerveSpecs specs;

    public swerveDriveConstants(
        SwerveModuleConstants frontLeftConstants,
        SwerveModuleConstants frontRightConstants,
        SwerveModuleConstants rearLeftConstants,
        SwerveModuleConstants rearRightConstants,
        swerveSpecs specs 
    ){

        this.frontLeftConstants = frontLeftConstants;
        this.frontRightConstants = frontRightConstants;
        this.rearLeftConstants = rearLeftConstants;
        this.rearRightConstants = rearRightConstants;
        this.specs = specs;

    }

    
}
