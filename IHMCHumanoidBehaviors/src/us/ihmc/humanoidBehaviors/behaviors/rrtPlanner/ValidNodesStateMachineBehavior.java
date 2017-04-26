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
import us.ihmc.manipulation.planning.rrt.RRTTree;
import us.ihmc.manipulation.planning.solarpanelmotion.SolarPanel;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.wholeBodyController.WholeBodyControllerParameters;

public class ValidNodesStateMachineBehavior extends StateMachineBehavior<RRTExpandingStates>
{      
   private FullHumanoidRobotModel fullRobotModel;
   private WholeBodyControllerParameters wholeBodyControllerParameters;
         
   private WaitingResultBehavior waitingResultBehavior;
   
   private SolarPanelPoseValidityTester testValidityBehavior;
   
   private int indexOfCurrentNode = -1;
   private ArrayList<RRTNode> nodes = new ArrayList<RRTNode>();
   
   public enum RRTExpandingStates
   {
      ASK_AND_SAVERESULT, VALIDITY_TEST
   }
   
   public ValidNodesStateMachineBehavior(ArrayList<RRTNode> nodes, CommunicationBridge communicationBridge, DoubleYoVariable yoTime,  
                                         WholeBodyControllerParameters wholeBodyControllerParameters, FullHumanoidRobotModel fullRobotModel)
   {
      super("RRTExpandingStateMachineBehavior", RRTExpandingStates.class, yoTime, communicationBridge);

      this.nodes = nodes;
      
      PrintTools.info("RRTExpandingStateMachineBehavior "+nodes+" test nodes");
      
      this.fullRobotModel = fullRobotModel;
      this.wholeBodyControllerParameters = wholeBodyControllerParameters;
      
      testValidityBehavior = new SolarPanelPoseValidityTester(wholeBodyControllerParameters, communicationBridge, fullRobotModel);
      waitingResultBehavior = new WaitingResultBehavior(communicationBridge);
      
      setUpStateMachine();
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
      BehaviorAction<RRTExpandingStates> waitingResultAction = new BehaviorAction<RRTExpandingStates>(RRTExpandingStates.ASK_AND_SAVERESULT, waitingResultBehavior);
      
      BehaviorAction<RRTExpandingStates> testValidityAction = new BehaviorAction<RRTExpandingStates>(RRTExpandingStates.VALIDITY_TEST, testValidityBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            PrintTools.info("testValidityAction "+indexOfCurrentNode +" " + nodes.get(indexOfCurrentNode).getNodeData(0) +" " +nodes.get(indexOfCurrentNode).getNodeData(1));   
          
            /*
             * override suitable node data for node.
             */            
            testValidityBehavior.setWholeBodyPose(TimeDomain1DNode.cleaningPath, nodes.get(indexOfCurrentNode).getNodeData(0), nodes.get(indexOfCurrentNode).getNodeData(1));
            
            testValidityBehavior.setUpHasBeenDone();
         }
      };
      
      
      
      statemachine.addStateWithDoneTransition(waitingResultAction, RRTExpandingStates.VALIDITY_TEST);
      statemachine.addStateWithDoneTransition(testValidityAction, RRTExpandingStates.ASK_AND_SAVERESULT);

      statemachine.setStartState(RRTExpandingStates.ASK_AND_SAVERESULT);
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
         //PrintTools.info("WaitingResultBehavior :: doControl");

            
         isDone = true;
      }

      @Override
      public void onBehaviorEntered()
      {
         //PrintTools.info("WaitingResultBehavior :: onBehaviorEntered");
         indexOfCurrentNode++;
         if(indexOfCurrentNode > nodes.size())
         {
            PrintTools.info("all nodes are tested. ");
            PrintTools.info("all nodes are tested. ");
            PrintTools.info("all nodes are tested. ");
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
         //PrintTools.info("WaitingResultBehavior :: onBehaviorExited");         
      }

      @Override
      public boolean isDone()
      {
         return isDone;
      }
   }
   
   
}
