package us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.RigidBody;

public class WholeBodyTrajectoryToolboxSettings
{
   public static List<RigidBody> getListOfRigidBody(FullHumanoidRobotModel fullRobotModel)
   {
      List<RigidBody> controlRigidBodyDefinitions = new ArrayList<RigidBody>();
      controlRigidBodyDefinitions.add(fullRobotModel.getHand(RobotSide.LEFT));
      controlRigidBodyDefinitions.add(fullRobotModel.getHand(RobotSide.RIGHT));
      controlRigidBodyDefinitions.add(fullRobotModel.getChest());
      controlRigidBodyDefinitions.add(fullRobotModel.getPelvis());
      return controlRigidBodyDefinitions;
   }

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
}
