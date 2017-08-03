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
      CTTaskNode.constrainedEndEffectorTrajectory = constrainedEndEffectorTrajectory;
   }
   
   public static void setInitialFullRobotModel(FullHumanoidRobotModel toolboxFullRobotModel)
   {  
      toolboxFullRobotModel.updateFrames();
      CTTaskNode.initialRobotModel = toolboxFullRobotModel;
      HumanoidReferenceFrames referenceFrames = new HumanoidReferenceFrames(toolboxFullRobotModel);
      CTTaskNode.midZUpFrame = referenceFrames.getMidFootZUpGroundFrame();
   }
   
   public static void setFullRobotModelFactory(FullHumanoidRobotModelFactory toolboxFullRobotModelFactory)
   {
      if(CTTaskNode.initialRobotModel != null)
         CTTaskNode.nodeTester = new WheneverWholeBodyKinematicsSolver(toolboxFullRobotModelFactory, CTTaskNode.initialRobotModel);
      else
         PrintTools.error("should be define initial robot model see 'GenericTaskNode.initialRobotModel' ");
   }
   
   public static void setMaximumUpdateOfTester(int value)
   {
      CTTaskNode.nodeTester.maximumCntForUpdateInternal = value;
   }
}
