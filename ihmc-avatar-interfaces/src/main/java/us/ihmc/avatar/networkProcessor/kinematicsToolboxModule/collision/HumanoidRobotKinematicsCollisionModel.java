package us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.collision;

import java.util.List;

import us.ihmc.robotModels.FullHumanoidRobotModel;

public interface HumanoidRobotKinematicsCollisionModel
{
   List<KinematicsCollidable> getRobotCollidables(FullHumanoidRobotModel fullRobotModel);
}
