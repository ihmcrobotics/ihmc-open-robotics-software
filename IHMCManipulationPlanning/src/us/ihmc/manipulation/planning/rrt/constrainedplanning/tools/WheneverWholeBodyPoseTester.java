package us.ihmc.manipulation.planning.rrt.constrainedplanning.tools;

import us.ihmc.manipulation.planning.robotcollisionmodel.RobotCollisionModel;
import us.ihmc.robotModels.FullHumanoidRobotModel;

public abstract class WheneverWholeBodyPoseTester
{
   private FullHumanoidRobotModel fullRobotModel;
   private RobotCollisionModel robotCollisionModel;

   protected boolean isCollisionFree = true;
   protected boolean isFeasibleIKSolution = true;

   protected boolean isValidPose = true;

   public WheneverWholeBodyPoseTester(FullHumanoidRobotModel fullRobotModel)
   {
      this.fullRobotModel = fullRobotModel;
   }

   public void setFullRobotModel(FullHumanoidRobotModel fullRobotModel)
   {
      this.fullRobotModel = fullRobotModel;
   }

   public RobotCollisionModel getRobotCollisionModel()
   {
      return robotCollisionModel;
   }

   public FullHumanoidRobotModel getFullHumanoidRobotModel()
   {
      return fullRobotModel;
   }

   private void update()
   {
      robotCollisionModel = new RobotCollisionModel(fullRobotModel);
      addEnvironmentCollisionModel();

      robotCollisionModel.update();
      isCollisionFree = robotCollisionModel.getCollisionResult();
   }

   public boolean isValidPose()
   {
      update();

      if (isCollisionFree && isFeasibleIKSolution)
         isValidPose = true;
      else
         isValidPose = false;

      return isValidPose;
   }

   public abstract void addEnvironmentCollisionModel();
}
