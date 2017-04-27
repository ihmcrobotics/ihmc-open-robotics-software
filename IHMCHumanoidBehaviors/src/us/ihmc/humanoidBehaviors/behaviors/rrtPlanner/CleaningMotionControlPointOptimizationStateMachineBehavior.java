package us.ihmc.humanoidBehaviors.behaviors.rrtPlanner;

import java.util.ArrayList;

import us.ihmc.commons.PrintTools;
import us.ihmc.humanoidBehaviors.behaviors.rrtPlanner.CleaningMotionControlPointOptimizationStateMachineBehavior.CleaningMotionControlPointOptimizationState;
import us.ihmc.humanoidBehaviors.behaviors.wholebodyValidityTester.SolarPanelPoseValidityTester;
import us.ihmc.humanoidBehaviors.communication.CommunicationBridge;
import us.ihmc.humanoidBehaviors.stateMachine.StateMachineBehavior;
import us.ihmc.manipulation.planning.rrt.RRTNode;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.wholeBodyController.WholeBodyControllerParameters;

public class CleaningMotionControlPointOptimizationStateMachineBehavior extends StateMachineBehavior<CleaningMotionControlPointOptimizationState>
{
   private FullHumanoidRobotModel fullRobotModel;
   private WholeBodyControllerParameters wholeBodyControllerParameters;
         
   
   private SolarPanelPoseValidityTester testValidityBehavior;
   
   private int indexOfCurrentNode = 0;
   private ArrayList<RRTNode> nodes = new ArrayList<RRTNode>();
   
   private double nodesScore = 0;
   private boolean nodesValidity = true;
   
   public enum CleaningMotionControlPointOptimizationState
   {
      INITIALIZE, VALIDITY_TEST, WAITING_RESULT, DONE
   }
   
   public CleaningMotionControlPointOptimizationStateMachineBehavior(CommunicationBridge communicationBridge, DoubleYoVariable yoTime,  
                                         WholeBodyControllerParameters wholeBodyControllerParameters, FullHumanoidRobotModel fullRobotModel)
   {
      super("CleaningMotionControlPointOptimizationStateMachineBehavior", CleaningMotionControlPointOptimizationState.class, yoTime, communicationBridge);

      
      PrintTools.info("CleaningMotionControlPointOptimizationStateMachineBehavior ");
      
      this.fullRobotModel = fullRobotModel;
      this.wholeBodyControllerParameters = wholeBodyControllerParameters;
      
      testValidityBehavior = new SolarPanelPoseValidityTester(wholeBodyControllerParameters, communicationBridge, fullRobotModel);
      
      setUpStateMachine();
   }
   
   private void setUpStateMachine()
   {    
      
   }

   @Override
   public void onBehaviorExited()
   {
      // TODO Auto-generated method stub
   }
}
