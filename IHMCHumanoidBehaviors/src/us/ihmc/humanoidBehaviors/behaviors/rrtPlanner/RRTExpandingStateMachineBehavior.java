package us.ihmc.humanoidBehaviors.behaviors.rrtPlanner;

import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidBehaviors.behaviors.AbstractBehavior;
import us.ihmc.humanoidBehaviors.behaviors.rrtPlanner.RRTExpandingStateMachineBehavior.RRTExpandingStates;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.BehaviorAction;
import us.ihmc.humanoidBehaviors.behaviors.wholebodyValidityTester.SolarPanelPoseValidityTester;
import us.ihmc.humanoidBehaviors.communication.CommunicationBridge;
import us.ihmc.humanoidBehaviors.communication.CommunicationBridgeInterface;
import us.ihmc.humanoidBehaviors.stateMachine.StateMachineBehavior;
import us.ihmc.manipulation.planning.rrt.RRTNode;
import us.ihmc.manipulation.planning.rrt.RRTTree;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.transformables.Pose;
import us.ihmc.wholeBodyController.WholeBodyControllerParameters;

public class RRTExpandingStateMachineBehavior extends StateMachineBehavior<RRTExpandingStates>
{      
   private FullHumanoidRobotModel fullRobotModel;
   private WholeBodyControllerParameters wholeBodyControllerParameters;
   
   private boolean validityTestResult = false;
   
   private RRTNode rootNode;
   private RRTTree rrtTree;
   
   private GetRandomNodeBehavior getRandomNodeBehavior;
   private ExpandingTreeBehavior expandingBehavior;
   private SolarPanelPoseValidityTester testValidityBehavior;
   
   public enum RRTExpandingStates
   {
      GET_RANDOMNODE, VALIDITY_TEST, EXPANDING_TREE
   }
   
   public RRTExpandingStateMachineBehavior(RRTNode rootNode, RRTTree rrtTree, CommunicationBridge communicationBridge, DoubleYoVariable yoTime,  
                                         WholeBodyControllerParameters wholeBodyControllerParameters, FullHumanoidRobotModel fullRobotModel)
   {
      super("RRTExpandingStateMachineBehavior", RRTExpandingStates.class, yoTime, communicationBridge);

      this.rootNode = rootNode;
      this.rrtTree = rrtTree;
      
      this.fullRobotModel = fullRobotModel;
      this.wholeBodyControllerParameters = wholeBodyControllerParameters;
      
      testValidityBehavior = new SolarPanelPoseValidityTester(wholeBodyControllerParameters, communicationBridge, fullRobotModel);
      
      
      
      
      
      
      setUpStateMachine();
      PrintTools.info("SetUp State Machine Done");
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
          
            Pose aPose = new Pose(new Point3D(0.8, -0.35, 1.1), new Quaternion());
            testValidityBehavior.setWholeBodyPose(aPose, Math.PI*0.2);
            
            PrintTools.info("Result is "+testValidityBehavior.isValid());
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
      
      
      
      statemachine.addStateWithDoneTransition(getRandomNodeAction, RRTExpandingStates.VALIDITY_TEST);
      statemachine.addStateWithDoneTransition(testValidityAction, RRTExpandingStates.EXPANDING_TREE);
      statemachine.addState(expandingTreeAction);
      
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
      
      public ExpandingTreeBehavior(CommunicationBridgeInterface communicationBridge)
      {
         super(communicationBridge);
      }

      @Override
      public void doControl()
      {
         PrintTools.info("ExpandingTreeBehavior :: doControl");
         isDone = true;
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
}
