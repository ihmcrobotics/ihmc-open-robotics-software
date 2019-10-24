package us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.collision;

import java.util.List;

import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.HumanoidKinematicsToolboxController;
import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.KinematicsToolboxController;
import us.ihmc.robotModels.FullHumanoidRobotModel;

/**
 * Defines a factory for creating the collision shapes of a humanoid robot that is compatible with
 * {@link HumanoidKinematicsToolboxController} and {@link KinematicsToolboxController}.
 * 
 * @author Sylvain Bertrand
 */
public interface HumanoidRobotKinematicsCollisionModel
{
   /**
    * Creates the collision shapes to be used with the given robot model.
    * 
    * @param fullRobotModel the full robot model that will be used with the collision shapes.
    * @return the list of collision shapes.
    */
   List<KinematicsCollidable> getRobotCollidables(FullHumanoidRobotModel fullRobotModel);
}
