package us.ihmc.humanoidBehaviors.behaviors.rrtPlanner;

import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidBehaviors.behaviors.AbstractBehavior;
import us.ihmc.humanoidBehaviors.behaviors.primitives.WholeBodyTrajectoryBehavior;
import us.ihmc.humanoidBehaviors.behaviors.rrtPlanner.CleaningMotionStateMachineBehavior.CleaningMotionState;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.BehaviorAction;
import us.ihmc.humanoidBehaviors.behaviors.solarPanel.SolarPanelWholeBodyTrajectoryMessageFacotry;
import us.ihmc.humanoidBehaviors.communication.CommunicationBridge;
import us.ihmc.humanoidBehaviors.communication.CommunicationBridgeInterface;
import us.ihmc.humanoidBehaviors.stateMachine.StateMachineBehavior;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.wholebody.WholeBodyTrajectoryMessage;
import us.ihmc.manipulation.planning.solarpanelmotion.SolarPanel;
import us.ihmc.manipulation.planning.solarpanelmotion.SolarPanelCleaningPose;
import us.ihmc.manipulation.planning.solarpanelmotion.SolarPanelPath;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.transformables.Pose;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.StateTransitionCondition;
import us.ihmc.wholeBodyController.WholeBodyControllerParameters;

public class CleaningMotionStateMachineBehavior extends StateMachineBehavior<CleaningMotionState>
{
   private SolarPanel solarPanel = new SolarPanel();
   
   private SolarPanelCleaningPose readyPose;
   private SolarPanelPath cleaningPath;
   
   private GetSolarPanelBehavior getSolarPanelBehavior;
   private ControlPointOptimizationStateMachineBehavior controlPointOptimizationBehavior;
   
   private WholeBodyTrajectoryBehavior wholebodyTrajectoryBehavior;
   
   private TestDoneBehavior doneBehavior;
   
   SolarPanelWholeBodyTrajectoryMessageFacotry motionFactory = new SolarPanelWholeBodyTrajectoryMessageFacotry();
   
   public enum CleaningMotionState
   {
      GET_SOLARPANEL, CONTROLPOINT_OPTIMIZATION, GOTO_READYPOSE, CLEANING_MOTION, DONE
   }
   
   public CleaningMotionStateMachineBehavior(CommunicationBridge communicationBridge, DoubleYoVariable yoTime,  
                                         WholeBodyControllerParameters wholeBodyControllerParameters, FullHumanoidRobotModel fullRobotModel)
   {
      super("CleaningMotionStateMachineBehavior", CleaningMotionState.class, yoTime, communicationBridge);
      
      PrintTools.info("CleaningMotionStateMachineBehavior ");

      getSolarPanelBehavior = new GetSolarPanelBehavior(communicationBridge);
      wholebodyTrajectoryBehavior = new WholeBodyTrajectoryBehavior(communicationBridge, yoTime);
      doneBehavior = new TestDoneBehavior(communicationBridge);
      
      
      
      
      TimeDomain1DNode rootNode = new TimeDomain1DNode(0.0, 0);
      
      controlPointOptimizationBehavior
      = new ControlPointOptimizationStateMachineBehavior(rootNode, communicationBridge, yoTime, wholeBodyControllerParameters, fullRobotModel);
      
      setUpStateMachine();
   }
   
   private void setUpStateMachine()
   {    
      // condition check for the case that no solution is exist in the CONTROLPOINT_OPTIMIZATION state.
      // In that case, the state transition should be to the DONE state.
      
      // In CONTROLPOINT_OPTIMIZATION, manually selected cleaning motion is put for test.
      
      
      BehaviorAction<CleaningMotionState> getSolarPanelAction = new BehaviorAction<CleaningMotionState>(CleaningMotionState.GET_SOLARPANEL, getSolarPanelBehavior);
      
      BehaviorAction<CleaningMotionState> controlPointOptimizationAction = new BehaviorAction<CleaningMotionState>(CleaningMotionState.CONTROLPOINT_OPTIMIZATION, controlPointOptimizationBehavior);
      
      StateTransitionCondition yesSolutionCondition = new StateTransitionCondition()
      {
         @Override
         public boolean checkCondition()
         {            
            boolean b = controlPointOptimizationAction.isDone() && controlPointOptimizationBehavior.isSolved() == true;
            return b;
         }
      };
      
      StateTransitionCondition noSolutionCondition = new StateTransitionCondition()
      {
         @Override
         public boolean checkCondition()
         {            
            boolean b = controlPointOptimizationAction.isDone() && controlPointOptimizationBehavior.isSolved() != true;
            return b;
         }
      };
      
      BehaviorAction<CleaningMotionState> gotoReadyPoseAction = new BehaviorAction<CleaningMotionState>(CleaningMotionState.GOTO_READYPOSE, wholebodyTrajectoryBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            PrintTools.info("gotoReadyPoseAction");
            WholeBodyTrajectoryMessage wholebodyMessage = new WholeBodyTrajectoryMessage();
            
            readyPose = new SolarPanelCleaningPose(solarPanel, 0.5, 0.1, -0.15, -Math.PI*0.2);
            motionFactory.setMessage(readyPose, Math.PI*0.0, 0.0, 3.0);
            wholebodyMessage = motionFactory.getWholeBodyTrajectoryMessage();
//            HandTrajectoryMessage handMessage = new HandTrajectoryMessage(RobotSide.RIGHT, 3.0, new Point3D(0.7, -0.35, 1.2), new Quaternion(), ReferenceFrame.getWorldFrame(), ReferenceFrame.getWorldFrame());
//            wholebodyMessage.setHandTrajectoryMessage(handMessage);
            wholebodyTrajectoryBehavior.setInput(wholebodyMessage);
         }
      };
      
      BehaviorAction<CleaningMotionState> cleaningAction = new BehaviorAction<CleaningMotionState>(CleaningMotionState.CLEANING_MOTION, wholebodyTrajectoryBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            PrintTools.info("cleaningAction");
            WholeBodyTrajectoryMessage wholebodyMessage = new WholeBodyTrajectoryMessage();
            motionFactory.setCleaningPath(TimeDomain1DNode.cleaningPath);         
            motionFactory.setMessage(controlPointOptimizationBehavior.getOptimalControlPointNodePath());            
            wholebodyMessage = motionFactory.getWholeBodyTrajectoryMessage();
//            HandTrajectoryMessage handMessage = new HandTrajectoryMessage(RobotSide.RIGHT, 3.0, new Point3D(0.7, -0.35, 1.6), new Quaternion(), ReferenceFrame.getWorldFrame(), ReferenceFrame.getWorldFrame());
//            wholebodyMessage.setHandTrajectoryMessage(handMessage);
            wholebodyTrajectoryBehavior.setInput(wholebodyMessage);
            
            
         }
      };
      
      
      BehaviorAction<CleaningMotionState> doneAction = new BehaviorAction<CleaningMotionState>(CleaningMotionState.DONE, doneBehavior);
      
      
      statemachine.addStateWithDoneTransition(getSolarPanelAction, CleaningMotionState.CONTROLPOINT_OPTIMIZATION);
            
      statemachine.addStateWithDoneTransition(gotoReadyPoseAction, CleaningMotionState.CLEANING_MOTION);
      statemachine.addStateWithDoneTransition(cleaningAction, CleaningMotionState.DONE);
      
      statemachine.addState(controlPointOptimizationAction);            
      controlPointOptimizationAction.addStateTransition(CleaningMotionState.GOTO_READYPOSE, yesSolutionCondition);
      controlPointOptimizationAction.addStateTransition(CleaningMotionState.DONE, noSolutionCondition);
      
      statemachine.addState(doneAction);
      
      statemachine.setStartState(CleaningMotionState.GET_SOLARPANEL);
      PrintTools.info("setUpStateMachine done ");
   }

   @Override
   public void onBehaviorExited()
   {
      // TODO Auto-generated method stub
   }
   
   
   
   
   
   private void setUpSolarPanel()
   {
      Pose poseSolarPanel = new Pose();
      Quaternion quaternionSolarPanel = new Quaternion();
      poseSolarPanel.setPosition(0.7, -0.05, 1.0);
      quaternionSolarPanel.appendRollRotation(0.0);
      quaternionSolarPanel.appendPitchRotation(-Math.PI*0.25);
      poseSolarPanel.setOrientation(quaternionSolarPanel);
      
      double sizeWidth = 0.6;
      double sizeLength = 0.6;
      
      solarPanel = new SolarPanel(poseSolarPanel, sizeWidth, sizeLength);
   }
   
   private class GetSolarPanelBehavior extends AbstractBehavior
   {
      private boolean isDone = false;
      
      public GetSolarPanelBehavior(CommunicationBridgeInterface communicationBridge)
      {
         super(communicationBridge);
      }

      @Override
      public void doControl()
      {            
         setUpSolarPanel();
         isDone = true;
      }

      @Override
      public void onBehaviorEntered()
      {   
         PrintTools.info("GetSolarPanelBehavior");
         
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
         PrintTools.info("TestDoneBehavior");
         
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
