// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Autonomous;

import java.util.ArrayList;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrainSubsystems;

/** Add your docs here. */
public class DriveAuto  {

    DriveTrainSubsystems driveSub;
    PathPlannerTrajectory simplePath;
    PathPlannerTrajectory complexPath;
    ProfiledPIDController thetaController;

    public DriveAuto(DriveTrainSubsystems d) {
       
        driveSub = d;
        thetaController = new ProfiledPIDController(Constants.ThetaController, 0, 0, Constants.thetaControllerConstraints);//.4
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
    }

    public ArrayList<PathPlannerTrajectory> loadFourBallTrajectories() {
        //PathPlannerTrajectory[] trajectoryList = {};
        ArrayList<PathPlannerTrajectory> trajectoryList = new ArrayList<PathPlannerTrajectory>();
        for (int i = 0; i < Constants.fourBallAuto.length; i++) {
          trajectoryList.add(i, PathPlanner.loadPath(Constants.fourBallAuto[i], 2, 2));
        }
        return trajectoryList;
    }

    public PathPlannerTrajectory loadSimpleAutoTrajectory() {
        return PathPlanner.loadPath(Constants.simpleAuto, 2, 2);
    }

    public Command getFourBallAutoPath(int desiredPath) {
        ArrayList<PathPlannerTrajectory> loadedTrajectories = loadFourBallTrajectories();
        PPSwerveControllerCommand command = new PPSwerveControllerCommand(
        loadedTrajectories.get(desiredPath),
        driveSub::getPose,
        Constants.m_kinematics,
        new PIDController(Constants.TranslationController, 0, 0),//.3
        new PIDController(Constants.StrafeController, 0, 0),//.3
        thetaController,
        driveSub::setModules,
        driveSub); 

        // Run path following command, then stop at the end.
        return command.andThen(() -> driveSub.drive(new ChassisSpeeds(0,0,0)));
    }

    public Command getSimpleAutoPath() {
        PPSwerveControllerCommand command = new PPSwerveControllerCommand(
        loadSimpleAutoTrajectory(),
        driveSub::getPose,
        Constants.m_kinematics,
        new PIDController(Constants.TranslationController, 0, 0),//.3
        new PIDController(Constants.StrafeController, 0, 0),//.3
        thetaController,
        driveSub::setModules,
        driveSub); 

        // Run path following command, then stop at the end.
        return command.andThen(() -> driveSub.drive(new ChassisSpeeds(0,0,0)));
    }

    public Pose2d getComplexInitialPose() {
        return loadFourBallTrajectories().get(0).getInitialPose();
    }

    public Pose2d getSimpleInitialPose() {
        return loadSimpleAutoTrajectory().getInitialPose();
    }

}