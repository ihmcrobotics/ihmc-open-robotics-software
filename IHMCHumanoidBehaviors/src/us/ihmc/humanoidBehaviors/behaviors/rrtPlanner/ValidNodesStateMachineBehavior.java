package us.ihmc.humanoidBehaviors.behaviors.rrtPlanner;

import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
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
import us.ihmc.manipulation.planning.solarpanelmotion.SolarPanelPath;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.transformables.Pose;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.StateTransitionCondition;
import us.ihmc.wholeBodyController.WholeBodyControllerParameters;

public class ValidNodesStateMachineBehavior extends StateMachineBehavior<RRTExpandingStates>
{      
   private FullHumanoidRobotModel fullRobotModel;
   private WholeBodyControllerParameters wholeBodyControllerParameters;
   
   private boolean validityTestResult = false;
   
   private RRTNode rootNode;
   private RRTTree rrtTree;
   
   private RRTNode newNode;
   
   private GetRandomNodeBehavior getRandomNodeBehavior;
   private ExpandingTreeBehavior expandingBehavior;
   private SolarPanelPoseValidityTester testValidityBehavior;
   private CloseBehavior closeBehavior;
   
   
   
   public enum RRTExpandingStates
   {
      GET_RANDOMNODE, VALIDITY_TEST, EXPANDING_TREE, CLOSE
   }
   
   public ValidNodesStateMachineBehavior(RRTNode rootNode, RRTTree rrtTree, CommunicationBridge communicationBridge, DoubleYoVariable yoTime,  
                                         WholeBodyControllerParameters wholeBodyControllerParameters, FullHumanoidRobotModel fullRobotModel)
   {
      super("RRTExpandingStateMachineBehavior", RRTExpandingStates.class, yoTime, communicationBridge);

      PrintTools.info("RRTExpandingStateMachineBehavior");
      
      this.rootNode = rootNode;
      this.rrtTree = rrtTree;
      
      this.fullRobotModel = fullRobotModel;
      this.wholeBodyControllerParameters = wholeBodyControllerParameters;
      
      testValidityBehavior = new SolarPanelPoseValidityTester(wholeBodyControllerParameters, communicationBridge, fullRobotModel);
      
      
      getRandomNodeBehavior = new GetRandomNodeBehavior(communicationBridge);  
      expandingBehavior = new ExpandingTreeBehavior(communicationBridge);  
      closeBehavior = new CloseBehavior(communicationBridge);
      
      
      
      setUpStateMachine();
      PrintTools.info("SetUp State Machine Done");
   }
   
   public void setSolarPanel(SolarPanel solarPanel)
   {
      testValidityBehavior.setSolarPanel(solarPanel); 
   }
   
   public RRTTree getRRTTree()
   {
      return rrtTree;
   }
   
   private void setUpStateMachine()
   {      
      BehaviorAction<RRTExpandingStates> getRandomNodeAction = new BehaviorAction<RRTExpandingStates>(RRTExpandingStates.GET_RANDOMNODE, getRandomNodeBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            PrintTools.info("getRandomNodeAction");            
         }
      };
      
      BehaviorAction<RRTExpandingStates> testValidityAction = new BehaviorAction<RRTExpandingStates>(RRTExpandingStates.VALIDITY_TEST, testValidityBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            PrintTools.info("testValidityAction");   
          
            /*
             * override suitable node data for node.
             */            
            testValidityBehavior.setWholeBodyPose(TimeDomain1DNode.cleaningPath, newNode.getNodeData(0), newNode.getNodeData(1));
            testValidityBehavior.onBehaviorEntered();
            testValidityBehavior.setUpHasBeenDone();
         }
      };
      
      BehaviorAction<RRTExpandingStates> expandingTreeAction = new BehaviorAction<RRTExpandingStates>(RRTExpandingStates.EXPANDING_TREE, expandingBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            PrintTools.info("testValidityAction");            
         }
      };
      
      BehaviorAction<RRTExpandingStates> closeAction = new BehaviorAction<RRTExpandingStates>(RRTExpandingStates.CLOSE, closeBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            PrintTools.info("closeAction");            
         }
      };
      
      StateTransitionCondition closeExpanding = new StateTransitionCondition()
      {
         @Override
         public boolean checkCondition()
         {
            if(expandingBehavior.nextState == RRTExpandingStates.CLOSE)
               return true;
            else
               return false;
         }
      };
      StateTransitionCondition repeatExpanding = new StateTransitionCondition()
      {
         @Override
         public boolean checkCondition()
         {
            if(expandingBehavior.nextState == RRTExpandingStates.GET_RANDOMNODE)
               return true;
            else
               return false;
         }
      };
      
      
      statemachine.addStateWithDoneTransition(getRandomNodeAction, RRTExpandingStates.VALIDITY_TEST);
      statemachine.addStateWithDoneTransition(testValidityAction, RRTExpandingStates.EXPANDING_TREE);
      statemachine.addState(expandingTreeAction);
      
      expandingTreeAction.addStateTransition(RRTExpandingStates.CLOSE, closeExpanding);
      expandingTreeAction.addStateTransition(RRTExpandingStates.GET_RANDOMNODE, repeatExpanding);
      
      
      statemachine.addState(closeAction);
      statemachine.setStartState(RRTExpandingStates.GET_RANDOMNODE);
   }
   
   @Override
   public void onBehaviorExited()
   {
      // TODO Auto-generated method stub
      
   }
   
   private class GetRandomNodeBehavior extends AbstractBehavior
   {
      private boolean isDone = false;
      
      public GetRandomNodeBehavior(CommunicationBridgeInterface communicationBridge)
      {
         super(communicationBridge);
      }

      @Override
      public void doControl()
      {
         PrintTools.info("GetRandomNodeBehavior :: doControl");
         isDone = true;
      }

      @Override
      public void onBehaviorEntered()
      {
         PrintTools.info("GetRandomNodeBehavior :: onBehaviorEntered");
         RRTNode node = rrtTree.getRandomNode();
         PrintTools.info("New Node is "+ node.getNodeData(0)+" "+ node.getNodeData(1));
         rrtTree.updateNearNodeForTargetNode(node);
         newNode = rrtTree.getNewNode(node);
         PrintTools.info("New Node is "+ newNode.getNodeData(0)+" "+ newNode.getNodeData(1));
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
         PrintTools.info("GetRandomNodeBehavior :: onBehaviorExited");
      }

      @Override
      public boolean isDone()
      {
         return isDone;
      }
   }
   
   private class ExpandingTreeBehavior extends AbstractBehavior
   {
      private boolean isDone = false;
      private int cnt = 0;
      public RRTExpandingStates nextState;
      
      public ExpandingTreeBehavior(CommunicationBridgeInterface communicationBridge)
      {
         super(communicationBridge);
      }

      @Override
      public void doControl()
      {
         PrintTools.info("ExpandingTreeBehavior :: doControl");
         isDone = true;         
         if(cnt < 3)
         {
            nextState = RRTExpandingStates.GET_RANDOMNODE;
            PrintTools.info("ExpandingTreeBehavior :: NextState is "+nextState);
         }
         else
         {
            nextState = RRTExpandingStates.CLOSE;
            PrintTools.info("ExpandingTreeBehavior :: NextState is "+nextState);
         }
         cnt++;
      }

      @Override
      public void onBehaviorEntered()
      {
         PrintTools.info("ExpandingTreeBehavior :: onBehaviorEntered");
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
         PrintTools.info("ExpandingTreeBehavior :: onBehaviorExited");
      }

      @Override
      public boolean isDone()
      {
         return isDone;
      }
   }
   
   private class CloseBehavior extends AbstractBehavior
   {
      private boolean isDone = false;
      
      public CloseBehavior(CommunicationBridgeInterface communicationBridge)
      {
         super(communicationBridge);
      }

      @Override
      public void doControl()
      {
//         PrintTools.info("CloseBehavior :: doControl");
//         onBehaviorExited();
      }

      @Override
      public void onBehaviorEntered()
      {
         PrintTools.info("CloseBehavior :: onBehaviorEntered");
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
         PrintTools.info("CloseBehavior :: onBehaviorExited");
      }

      @Override
      public boolean isDone()
      {
         return isDone;
      }
   }
   
}
