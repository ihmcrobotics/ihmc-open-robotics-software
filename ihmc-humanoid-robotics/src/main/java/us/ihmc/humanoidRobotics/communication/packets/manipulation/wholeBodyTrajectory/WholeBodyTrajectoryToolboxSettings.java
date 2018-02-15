package us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory;

import java.util.Random;

import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.RigidBody;

public class WholeBodyTrajectoryToolboxSettings
{
   public static double timeCoefficient = 2.5;
   // TODO : will be get rid of. just for test.
   public static Random randomManager = new Random(1);

   // TODO : this is useless..?
   public static ConfigurationSpaceName[] getDefaultExplorationConfiguratSpaces(FullHumanoidRobotModel fullRobotModel, RigidBody rigidBody)
   {
      ConfigurationSpaceName[] configurationSpaces;

      if (rigidBody == fullRobotModel.getHand(RobotSide.LEFT))
      {
         configurationSpaces = new ConfigurationSpaceName[6];
         configurationSpaces[0] = ConfigurationSpaceName.X;
         configurationSpaces[1] = ConfigurationSpaceName.Y;
         configurationSpaces[2] = ConfigurationSpaceName.Z;
         configurationSpaces[3] = ConfigurationSpaceName.YAW;
         configurationSpaces[4] = ConfigurationSpaceName.PITCH;
         configurationSpaces[5] = ConfigurationSpaceName.ROLL;
      }
      else if (rigidBody == fullRobotModel.getHand(RobotSide.RIGHT))
      {
         configurationSpaces = new ConfigurationSpaceName[6];
         configurationSpaces[0] = ConfigurationSpaceName.X;
         configurationSpaces[1] = ConfigurationSpaceName.Y;
         configurationSpaces[2] = ConfigurationSpaceName.Z;
         configurationSpaces[3] = ConfigurationSpaceName.YAW;
         configurationSpaces[4] = ConfigurationSpaceName.PITCH;
         configurationSpaces[5] = ConfigurationSpaceName.ROLL;
      }
      else if (rigidBody == fullRobotModel.getPelvis())
      {
         configurationSpaces = new ConfigurationSpaceName[1];
         configurationSpaces[0] = ConfigurationSpaceName.Z;
         //         configurationSpaces[1] = ConfigurationSpaceName.YAW;
         //         configurationSpaces[2] = ConfigurationSpaceName.PITCH;
         //         configurationSpaces[3] = ConfigurationSpaceName.ROLL;
      }
      else if (rigidBody == fullRobotModel.getChest())
      {
         configurationSpaces = new ConfigurationSpaceName[3];
         configurationSpaces[0] = ConfigurationSpaceName.YAW;
         configurationSpaces[1] = ConfigurationSpaceName.PITCH;
         configurationSpaces[2] = ConfigurationSpaceName.ROLL;
      }
      else
         configurationSpaces = null;

      return configurationSpaces;
   }

   /**
    * This is for Atlas with RobotiQ hand. Control frame to original frame.
    */
   public static SideDependentList<Pose3D> getAtlasRobotiQHandControlFrames()
   {
      SideDependentList<Pose3D> frames = new SideDependentList<>();

      Point3D position;
      Quaternion orientation;

      for (RobotSide robotSide : RobotSide.values)
      {
         if (robotSide == RobotSide.LEFT)
         {
            position = new Point3D(0.0, 0.19, 0.0);
            orientation = new Quaternion();
            orientation.appendYawRotation(0.5 * Math.PI);
            orientation.appendRollRotation(-0.5 * Math.PI);
         }
         else
         {
            position = new Point3D(0.0, -0.19, 0.0);
            orientation = new Quaternion();
            orientation.appendYawRotation(-0.5 * Math.PI);
            orientation.appendRollRotation(0.5 * Math.PI);
         }

         frames.put(robotSide, new Pose3D(position, orientation));
      }

      return frames;
   }

   /**
    * This is for Atlas with RobotiQ hand. Control frame to original frame.
    */
   public static SideDependentList<Pose3D> getValkyrieHandControlFrames()
   {
      SideDependentList<Pose3D> frames = new SideDependentList<>();

      Point3D position;
      Quaternion orientation;

      for (RobotSide robotSide : RobotSide.values)
      {
         if (robotSide == RobotSide.LEFT)
         {
            position = new Point3D(0.02, 0.02, 0.0);
            orientation = new Quaternion();
            orientation.appendYawRotation(0.5 * Math.PI);
         }
         else
         {
            position = new Point3D(0.02, -0.02, 0.0);
            orientation = new Quaternion();
            orientation.appendYawRotation(-0.5 * Math.PI);
         }

         frames.put(robotSide, new Pose3D(position, orientation));
      }

      return frames;
   }
}
