package us.ihmc.manipulation.planning.rrt.constrainedplanning.configurationAndTimeSpace;

import us.ihmc.commons.PrintTools;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.manipulation.planning.rrt.constrainedplanning.tools.WheneverWholeBodyKinematicsSolver;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullHumanoidRobotModelFactory;

public class ConstrainedWholeBodyPlanningToolboxHelper
{
   public static void setConstrainedEndEffectorTrajectory(ConstrainedEndEffectorTrajectory constrainedEndEffectorTrajectory)
   {
      GenericTaskNode.constrainedEndEffectorTrajectory = constrainedEndEffectorTrajectory;
   }
   
   public static void setInitialFullRobotModel(FullHumanoidRobotModel toolboxFullRobotModel)
   {  
      toolboxFullRobotModel.updateFrames();
      GenericTaskNode.initialRobotModel = toolboxFullRobotModel;
      HumanoidReferenceFrames referenceFrames = new HumanoidReferenceFrames(toolboxFullRobotModel);
      GenericTaskNode.midZUpFrame = referenceFrames.getMidFootZUpGroundFrame();
   }
   
   public static void setFullRobotModelFactory(FullHumanoidRobotModelFactory toolboxFullRobotModelFactory)
   {
      if(GenericTaskNode.initialRobotModel != null)
         GenericTaskNode.nodeTester = new WheneverWholeBodyKinematicsSolver(toolboxFullRobotModelFactory, GenericTaskNode.initialRobotModel);
      else
         PrintTools.error("should be define initial robot model see 'GenericTaskNode.initialRobotModel' ");
   }


}
