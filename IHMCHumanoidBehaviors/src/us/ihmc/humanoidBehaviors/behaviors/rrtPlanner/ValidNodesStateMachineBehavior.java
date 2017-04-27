package us.ihmc.humanoidBehaviors.behaviors.rrtPlanner;

import java.util.ArrayList;

import us.ihmc.commons.PrintTools;
import us.ihmc.humanoidBehaviors.behaviors.AbstractBehavior;
import us.ihmc.humanoidBehaviors.behaviors.rrtPlanner.ValidNodesStateMachineBehavior.RRTExpandingStates;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.BehaviorAction;
import us.ihmc.humanoidBehaviors.behaviors.wholebodyValidityTester.SolarPanelPoseValidityTester;
import us.ihmc.humanoidBehaviors.communication.CommunicationBridge;
import us.ihmc.humanoidBehaviors.communication.CommunicationBridgeInterface;
import us.ihmc.humanoidBehaviors.stateMachine.StateMachineBehavior;
import us.ihmc.manipulation.planning.rrt.RRTNode;
import us.ihmc.manipulation.planning.solarpanelmotion.SolarPanel;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.StateTransitionCondition;
import us.ihmc.wholeBodyController.WholeBodyControllerParameters;

public class ValidNodesStateMachineBehavior extends StateMachineBehavior<RRTExpandingStates>
{      
   private FullHumanoidRobotModel fullRobotModel;
   private WholeBodyControllerParameters wholeBodyControllerParameters;
         
   private WaitingResultBehavior waitingResultBehavior;
   private TestDoneBehavior testDoneBehavior;
   
   private SolarPanelPoseValidityTester testValidityBehavior;
   
   private int indexOfCurrentNode = 0;
   private ArrayList<RRTNode> nodes = new ArrayList<RRTNode>();
   
   private double nodesScore = 0;
   private boolean nodesValidity = true;
   
   public enum RRTExpandingStates
   {
      INITIALIZE, VALIDITY_TEST, WAITING_RESULT, DONE
   }
   
   public ValidNodesStateMachineBehavior(ArrayList<RRTNode> nodes, CommunicationBridge communicationBridge, DoubleYoVariable yoTime,  
                                         WholeBodyControllerParameters wholeBodyControllerParameters, FullHumanoidRobotModel fullRobotModel)
   {
      super("RRTExpandingStateMachineBehavior", RRTExpandingStates.class, yoTime, communicationBridge);

      this.nodes = nodes;
      
      PrintTools.info("RRTExpandingStateMachineBehavior "+nodes.size()+" test nodes");
      
      this.fullRobotModel = fullRobotModel;
      this.wholeBodyControllerParameters = wholeBodyControllerParameters;
      
      testValidityBehavior = new SolarPanelPoseValidityTester(wholeBodyControllerParameters, communicationBridge, fullRobotModel);
      waitingResultBehavior = new WaitingResultBehavior(communicationBridge);
      testDoneBehavior = new TestDoneBehavior(communicationBridge);
      
      setUpStateMachine();
   }
   
   public double getScore()
   {
      return nodesScore;
   }
   
   public boolean getValdity()
   {
      return nodesValidity;
   }
   
   public void setNodes(ArrayList<RRTNode> nodes)
   {
      this.nodes = nodes;
   }
   
   public void setSolarPanel(SolarPanel solarPanel)
   {
      testValidityBehavior.setSolarPanel(solarPanel); 
   }
   
   private void setUpStateMachine()
   {    
      BehaviorAction<RRTExpandingStates> intializePrivilegedConfigurationAction = new BehaviorAction<RRTExpandingStates>(RRTExpandingStates.INITIALIZE, testValidityBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            /*
             * override suitable node data for node.
             */        
            testValidityBehavior.setWholeBodyPose(TimeDomain1DNode.cleaningPath, 0, 0);            
            testValidityBehavior.setUpHasBeenDone();
         }
      };
      
      BehaviorAction<RRTExpandingStates> testValidityAction = new BehaviorAction<RRTExpandingStates>(RRTExpandingStates.VALIDITY_TEST, testValidityBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            /*
             * override suitable node data for node.
             */
            testValidityBehavior.setWholeBodyPose(TimeDomain1DNode.cleaningPath, nodes.get(indexOfCurrentNode).getNodeData(0), nodes.get(indexOfCurrentNode).getNodeData(1));            
            testValidityBehavior.setUpHasBeenDone();
            indexOfCurrentNode++; 
         }
      };
      
      BehaviorAction<RRTExpandingStates> waitingResultAction = new BehaviorAction<RRTExpandingStates>(RRTExpandingStates.WAITING_RESULT, waitingResultBehavior);
      
      BehaviorAction<RRTExpandingStates> testDoneAction = new BehaviorAction<RRTExpandingStates>(RRTExpandingStates.DONE, testDoneBehavior);
      
      StateTransitionCondition keepDoingCondition = new StateTransitionCondition()
      {
         @Override
         public boolean checkCondition()
         {            
            boolean b = waitingResultBehavior.isDone() && (indexOfCurrentNode < nodes.size());
            return b;
         }
      };

      StateTransitionCondition doneCondition = new StateTransitionCondition()
      {
         @Override
         public boolean checkCondition()
         {
            boolean b = waitingResultBehavior.isDone() && indexOfCurrentNode == nodes.size();
            return b;
         }
      };

      statemachine.addStateWithDoneTransition(intializePrivilegedConfigurationAction, RRTExpandingStates.VALIDITY_TEST);
      statemachine.addStateWithDoneTransition(testValidityAction, RRTExpandingStates.WAITING_RESULT);
      
      statemachine.addState(waitingResultAction);
      waitingResultAction.addStateTransition(RRTExpandingStates.VALIDITY_TEST, keepDoingCondition);

      waitingResultAction.addStateTransition(RRTExpandingStates.DONE, doneCondition);
      
      statemachine.addState(testDoneAction);            
      statemachine.setStartState(RRTExpandingStates.INITIALIZE);
   }
   
   @Override
   public void onBehaviorExited()
   {
      // TODO Auto-generated method stub      
   }
   
   private class WaitingResultBehavior extends AbstractBehavior
   {
      private boolean isDone = false;
      
      public WaitingResultBehavior(CommunicationBridgeInterface communicationBridge)
      {
         super(communicationBridge);
      }

      @Override
      public void doControl()
      {            
         isDone = true;
      }

      @Override
      public void onBehaviorEntered()
      {
         double curScore = testValidityBehavior.getScroe();
         nodesScore = nodesScore + curScore;
         
         PrintTools.info(""+(indexOfCurrentNode-1)+" result get "+ testValidityBehavior.isValid());
         if(testValidityBehavior.isValid() == false)
         {
            nodesValidity = false;
         }
      }

      @Override
      public void onBehaviorAborted()
      {
      }

      @Override
      public void onBehaviorPaused()
      {
      }

      @Override
      public void onBehaviorResumed()
      {
      }

      @Override
      public void onBehaviorExited()
      {         
      }

      @Override
      public boolean isDone()
      {
         return isDone;
      }
   }
   
   private class TestDoneBehavior extends AbstractBehavior
   {
      private boolean isDone = false;
      
      public TestDoneBehavior(CommunicationBridgeInterface communicationBridge)
      {
         super(communicationBridge);
      }

      @Override
      public void doControl()
      {            
         isDone = true;
      }

      @Override
      public void onBehaviorEntered()
      {   
      }

      @Override
      public void onBehaviorAborted()
      {
      }

      @Override
      public void onBehaviorPaused()
      {
      }

      @Override
      public void onBehaviorResumed()
      {
      }

      @Override
      public void onBehaviorExited()
      {         
      }

      @Override
      public boolean isDone()
      {
         return isDone;
      }
   }
}
