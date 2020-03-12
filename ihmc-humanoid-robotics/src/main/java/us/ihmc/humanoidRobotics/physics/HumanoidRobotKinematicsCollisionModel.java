package us.ihmc.humanoidRobotics.physics;

import java.util.List;

import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.physics.KinematicsCollidable;

/**
 * Defines a factory for creating the collision shapes of a humanoid robot.
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
