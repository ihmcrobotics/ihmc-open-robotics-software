package us.ihmc.manipulation.planning.rrt.wholebodyplanning;

import us.ihmc.manipulation.planning.robotcollisionmodel.RobotCollisionModel;
import us.ihmc.robotModels.FullHumanoidRobotModel;

public class WheneverWholeBodyPoseTester
{
   FullHumanoidRobotModel fullRobotModel;
   RobotCollisionModel robotCollisionModel;
   
   public void WheneverWholeBodyPoseTester(FullHumanoidRobotModel fullRobotModel)
   {
      this.fullRobotModel = fullRobotModel;
   }
   
   public void setFullRobotModel(FullHumanoidRobotModel fullRobotModel)
   {
      this.fullRobotModel = fullRobotModel;      
   }
   
   public boolean isValidPose()
   {
      return true;
   }
}
