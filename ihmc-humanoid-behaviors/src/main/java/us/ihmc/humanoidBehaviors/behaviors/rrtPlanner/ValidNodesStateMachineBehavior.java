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
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.manipulation.planning.rrt.RRTNode;
import us.ihmc.manipulation.planning.solarpanelmotion.SolarPanel;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullHumanoidRobotModelFactory;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.StateTransitionCondition;
import us.ihmc.yoVariables.variable.YoDouble;

public class ValidNodesStateMachineBehavior extends StateMachineBehavior<RRTExpandingStates>
{
   private WaitingResultBehavior waitingResultBehavior;
   private TestDoneBehavior testDoneBehavior;

   private SolarPanelPoseValidityTester testValidityBehavior;

   private int indexOfCurrentNode = 0;
   private ArrayList<RRTNode> nodes = new ArrayList<RRTNode>();

   private double nodesScore;
   private boolean nodesValidity;

   private boolean DEBUG = false;
   private boolean NONSTOP = false;

   public enum RRTExpandingStates
   {
      INITIALIZE, VALIDITY_TEST, WAITING_RESULT, DONE
   }

   public ValidNodesStateMachineBehavior(ArrayList<RRTNode> nodes, CommunicationBridge communicationBridge, YoDouble yoTime,
                                         FullHumanoidRobotModelFactory robotModelFactory, FullHumanoidRobotModel fullRobotModel, HumanoidReferenceFrames referenceFrames)
   {
      super("RRTExpandingStateMachineBehavior", RRTExpandingStates.class, yoTime, communicationBridge);

      if(DEBUG)
         PrintTools.info("number Of nodes "+nodes.size());
      this.nodes = nodes;

      testValidityBehavior = new SolarPanelPoseValidityTester(robotModelFactory, communicationBridge, fullRobotModel, referenceFrames);
      waitingResultBehavior = new WaitingResultBehavior(communicationBridge);
      testDoneBehavior = new TestDoneBehavior(communicationBridge);

      nodesValidity = true;
      nodesScore = 0;

      setUpStateMachine();
   }

   public double getScore()
   {
      return nodesScore;
   }

   public void resultInitialize()
   {
      indexOfCurrentNode = 0;
      nodesScore = 0;
      nodesValidity = true;
   }

   public boolean getNodesValdity()
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
            resultInitialize();
            /*
             * override suitable node data for node.
             */
            testValidityBehavior.setWholeBodyPose(SolarPanelCleaningInfo.getCleaningPath(), SolarPanelCleaningInfo.getNode());
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
            if(DEBUG)
               PrintTools.info("Check :: Tester Set Behavior Input ");

            if(indexOfCurrentNode < nodes.size())
            {
               testValidityBehavior.setWholeBodyPose(SolarPanelCleaningInfo.getCleaningPath(), nodes.get(indexOfCurrentNode));
               testValidityBehavior.setUpHasBeenDone();
               indexOfCurrentNode++;
            }
            else
            {
               testValidityBehavior.setIsDone(true);
            }
         }
      };

      BehaviorAction<RRTExpandingStates> waitingResultAction = new BehaviorAction<RRTExpandingStates>(RRTExpandingStates.WAITING_RESULT, waitingResultBehavior);

      StateTransitionCondition keepDoingCondition = new StateTransitionCondition()
      {
         @Override
         public boolean checkCondition()
         {
            boolean b;
            if(NONSTOP)
               b = waitingResultBehavior.isDone() && (indexOfCurrentNode < nodes.size());
            else
               b = (waitingResultBehavior.isDone() && (indexOfCurrentNode < nodes.size()) && nodesValidity == true);

            if(DEBUG && b)
               PrintTools.info("Check :: keepDoingCondition " + b);
            return b;
         }
      };

      StateTransitionCondition doneCondition = new StateTransitionCondition()
      {
         @Override
         public boolean checkCondition()
         {
            boolean b;
            if(NONSTOP)
               b = waitingResultBehavior.isDone() && (indexOfCurrentNode == nodes.size());
            else
               b = ((waitingResultBehavior.isDone() && (indexOfCurrentNode == nodes.size())) || nodesValidity == false);


            if(DEBUG && b)
               PrintTools.info("Check :: doneCondition " + b);
            return b;
         }
      };

      BehaviorAction<RRTExpandingStates> testDoneAction = new BehaviorAction<RRTExpandingStates>(RRTExpandingStates.DONE, testDoneBehavior);

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

         if(DEBUG)
            PrintTools.info(" "+ nodesScore +" "+curScore +" "+testValidityBehavior.isValid() +" cur nodes "+nodesValidity);
         if(DEBUG)
            PrintTools.info("Check :: Waiting Behavior ");
         if(DEBUG)
            PrintTools.info(" "+ nodes.size() +" "+(indexOfCurrentNode-1)+" result get "+ testValidityBehavior.isValid());

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
         if(DEBUG)
            PrintTools.info("Check :: Done Behavior ");
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
         if(true)
            PrintTools.info("Check :: Exit Behavior ");
      }

      @Override
      public boolean isDone()
      {
         return isDone;
      }
   }
}
