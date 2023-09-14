package frc.lib.maethLib.Swerve.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.maethLib.Swerve.constants.swerveSpecs;
import frc.lib.maethLib.Swerve.subsystems.meathSwerve;

public class swerveDriveComando extends CommandBase {

    private final meathSwerve swerveSubsystem;
    private final Supplier<Double> driveFunction, strafeFunction, giroFunction;
    private final Supplier<Boolean> isFieldOriented;
    private final SlewRateLimiter xLimiter, yLimiter, giroLimiter;
    private final  Boolean joystickDrive;
    private final swerveSpecs specs;


    /**
     * Driving command
     * 
     * @param swerveSubsystem Swerve subsystem in use
     * @param driveVelocityFunction Supplier for the drive function
     * @param strafeVelocityFunction Supplier for the strafe function
     * @param rotationVelocityFunction Supplier for the rotation function
     * @param fieldOrientedFunction Boolean for whether the drive will be field oriented
     */
    public swerveDriveComando(meathSwerve subsistemaSwerve,
            Supplier<Double> velAvanceFuncion, Supplier<Double> velStrafeFuncion, Supplier<Double> velGiroFuncion,
            Supplier<Boolean> fieldOrientedFunction, Boolean joystickDrive
            ) {
       
        /**
         * Subsystem
         */
        this.swerveSubsystem = subsistemaSwerve;
        
        this.specs = subsistemaSwerve.getSwerveSpecs();

        this.driveFunction = velAvanceFuncion;
        this.strafeFunction = velStrafeFuncion;
        this.giroFunction = velGiroFuncion;
        
        this.isFieldOriented = fieldOrientedFunction;
        
        this.joystickDrive = joystickDrive;
        
       /**
        * Limiters for acceleration and a better moving of the robot
        * 
        */
        this.xLimiter = new SlewRateLimiter(specs.driveAccelerationLimiter);
        this.yLimiter = new SlewRateLimiter(specs.driveAccelerationLimiter);
        this.giroLimiter = new SlewRateLimiter(specs.rotationAccelerationLimiter);
        
       
        addRequirements(subsistemaSwerve);
    }

    @Override
    public void initialize() {
    
        
    }

    @Override
    public void execute() {

        // 1. Get real-time joystick inputs
 if(joystickDrive == true){
        double driveVel;
        double strafeVel;
        double rotationVel; 


    driveVel = driveFunction.get();
    strafeVel = strafeFunction.get();
    rotationVel = giroFunction.get();  


        // 2. Apply deadband
        driveVel = Math.abs(driveVel) > 0.05 ? driveVel : 0.0;
        strafeVel = Math.abs(strafeVel) > 0.05 ? strafeVel : 0.0;
        rotationVel = Math.abs(rotationVel) > 0.05 ? rotationVel : 0.0;

         // 3. Make the driving smoother
        driveVel = xLimiter.calculate(driveVel) * 3.5;
        strafeVel = yLimiter.calculate(strafeVel) * 3.5;
        rotationVel = giroLimiter.calculate(rotationVel)
                * 6;

        // 4. Construct desired chassis speeds
        ChassisSpeeds chassisSpeeds;
        if (isFieldOriented.get()) {
            // Relative to field
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    driveVel, strafeVel, rotationVel, swerveSubsystem.getRotation2d());
        } else {
             //Relative to robot
            chassisSpeeds = new ChassisSpeeds(driveVel, strafeVel, rotationVel);
        }

        // 5. Convert chassis speeds to individual module states
        SwerveModuleState[] moduleStates = specs.kinematics.toSwerveModuleStates(chassisSpeeds);

        // 6. Output each module states to wheels
        swerveSubsystem.setModuleStates(moduleStates);
    } else {
        return;
    }
    } 

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
