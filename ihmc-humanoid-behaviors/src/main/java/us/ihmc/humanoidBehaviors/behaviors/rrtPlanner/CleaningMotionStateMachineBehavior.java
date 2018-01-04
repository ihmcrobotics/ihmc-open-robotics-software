package us.ihmc.humanoidBehaviors.behaviors.rrtPlanner;

import us.ihmc.commons.PrintTools;
import us.ihmc.communication.packets.TextToSpeechPacket;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidBehaviors.behaviors.AbstractBehavior;
import us.ihmc.humanoidBehaviors.behaviors.primitives.WholeBodyTrajectoryBehavior;
import us.ihmc.humanoidBehaviors.behaviors.rrtPlanner.CleaningMotionStateMachineBehavior.CleaningMotionState;
import us.ihmc.humanoidBehaviors.behaviors.rrtPlanner.SolarPanelCleaningInfo.DegreesOfRedundancy;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.BehaviorAction;
import us.ihmc.humanoidBehaviors.behaviors.solarPanel.SolarPanelWholeBodyTrajectoryMessageFacotry;
import us.ihmc.humanoidBehaviors.communication.CommunicationBridge;
import us.ihmc.humanoidBehaviors.communication.CommunicationBridgeInterface;
import us.ihmc.humanoidBehaviors.stateMachine.StateMachineBehavior;
import us.ihmc.humanoidRobotics.communication.packets.wholebody.WholeBodyTrajectoryMessage;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.manipulation.planning.rrt.RRTNode;
import us.ihmc.manipulation.planning.solarpanelmotion.SolarPanel;
import us.ihmc.manipulation.planning.solarpanelmotion.SolarPanelCleaningPose;
import us.ihmc.manipulation.planning.solarpanelmotion.SolarPanelPath;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullHumanoidRobotModelFactory;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.StateTransitionCondition;
import us.ihmc.yoVariables.variable.YoDouble;

public class CleaningMotionStateMachineBehavior extends StateMachineBehavior<CleaningMotionState>
{
   private GetSolarPanelBehavior getSolarPanelBehavior;
   private ControlPointOptimizationStateMachineBehavior controlPointOptimizationBehavior;

   private WholeBodyTrajectoryBehavior wholebodyTrajectoryBehavior;

   private TestDoneBehavior doneBehavior;

   private SolarPanelWholeBodyTrajectoryMessageFacotry motionFactory;

   YoDouble yoTime;
   FullHumanoidRobotModel fullRobotModel;

   public enum CleaningMotionState
   {
      GET_SOLARPANEL, CONTROLPOINT_OPTIMIZATION, GOTO_READYPOSE, CLEANING_MOTION, DONE
   }

   public CleaningMotionStateMachineBehavior(CommunicationBridge communicationBridge, YoDouble yoTime, FullHumanoidRobotModelFactory robotModelFactory,
                                             FullHumanoidRobotModel fullRobotModel, HumanoidReferenceFrames referenceFrames)
   {
      super("CleaningMotionStateMachineBehavior", CleaningMotionState.class, yoTime, communicationBridge);

      PrintTools.info("CleaningMotionStateMachineBehavior ");

      getSolarPanelBehavior = new GetSolarPanelBehavior(communicationBridge);
      wholebodyTrajectoryBehavior = new WholeBodyTrajectoryBehavior(communicationBridge, yoTime);
      doneBehavior = new TestDoneBehavior(communicationBridge);

      motionFactory = new SolarPanelWholeBodyTrajectoryMessageFacotry(fullRobotModel);


      // ********************************** get SolarPanel Info ********************************** //

      Pose3D poseSolarPanel = new Pose3D();
      Quaternion quaternionSolarPanel = new Quaternion();
      poseSolarPanel.setPosition(0.7, -0.05, 1.0);
      quaternionSolarPanel.appendRollRotation(0.0);
      quaternionSolarPanel.appendPitchRotation(-Math.PI*0.25);
      poseSolarPanel.setOrientation(quaternionSolarPanel);

      SolarPanel solarPanel = new SolarPanel(poseSolarPanel, 0.6, 0.6);

      // ********************************** get SolarPanel Info ********************************** //
      // *********************************** get Cleaning Path *********************************** //

      SolarPanelCleaningPose readyPose = new SolarPanelCleaningPose(solarPanel, 0.5, 0.1, -0.15, -Math.PI*0.1);
      SolarPanelPath cleaningPath = new SolarPanelPath(readyPose);

      cleaningPath.addCleaningPose(new SolarPanelCleaningPose(solarPanel, 0.1, 0.1, -0.15, -Math.PI*0.3), 4.0);
      cleaningPath.addCleaningPose(new SolarPanelCleaningPose(solarPanel, 0.1, 0.2, -0.15, -Math.PI*0.3), 1.0);
      cleaningPath.addCleaningPose(new SolarPanelCleaningPose(solarPanel, 0.5, 0.2, -0.15, -Math.PI*0.1), 4.0);
      cleaningPath.addCleaningPose(new SolarPanelCleaningPose(solarPanel, 0.5, 0.3, -0.15, -Math.PI*0.1), 1.0);
      cleaningPath.addCleaningPose(new SolarPanelCleaningPose(solarPanel, 0.1, 0.3, -0.15, -Math.PI*0.3), 4.0);
      cleaningPath.addCleaningPose(new SolarPanelCleaningPose(solarPanel, 0.1, 0.4, -0.15, -Math.PI*0.3), 1.0);
      cleaningPath.addCleaningPose(new SolarPanelCleaningPose(solarPanel, 0.5, 0.4, -0.15, -Math.PI*0.1), 1.0);
      cleaningPath.addCleaningPose(new SolarPanelCleaningPose(solarPanel, 0.5, 0.5, -0.15, -Math.PI*0.1), 1.0);
      cleaningPath.addCleaningPose(new SolarPanelCleaningPose(solarPanel, 0.1, 0.5, -0.15, -Math.PI*0.3), 1.0);


      SolarPanelCleaningInfo.setCleaningPath(cleaningPath);
      SolarPanelCleaningInfo.setDegreesOfRedundancy(DegreesOfRedundancy.THREE);

      PrintTools.info("cur Height is "+ fullRobotModel.getPelvis().getBodyFixedFrame().getTransformToWorldFrame().getM23() +" "+ yoTime);
      TimeDomain3DNode.defaultPelvisHeight = fullRobotModel.getPelvis().getParentJoint().getFrameAfterJoint().getTransformToWorldFrame().getM23();
      // *********************************** get Cleaning Path *********************************** //

      // for re-initializing ControlPointOptimizationStateMachineBehavior after saving solarpanel information.
      // 170512
      this.yoTime = yoTime;
      this.fullRobotModel = fullRobotModel;

      RRTNode rootNode = SolarPanelCleaningInfo.getNode();

      controlPointOptimizationBehavior
      = new ControlPointOptimizationStateMachineBehavior(rootNode, communicationBridge, yoTime, robotModelFactory, fullRobotModel, referenceFrames);

      setUpStateMachine();
   }

   private void setUpStateMachine()
   {
      // condition check for the case that no solution is exist in the CONTROLPOINT_OPTIMIZATION state.
      // In that case, the state transition should be to the DONE state.

      // In CONTROLPOINT_OPTIMIZATION, manually selected cleaning motion is put for test.


      BehaviorAction<CleaningMotionState> getSolarPanelAction = new BehaviorAction<CleaningMotionState>(CleaningMotionState.GET_SOLARPANEL, getSolarPanelBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            TextToSpeechPacket p1 = new TextToSpeechPacket("getSolarPanelAction");
            sendPacket(p1);
         }
      };

      BehaviorAction<CleaningMotionState> controlPointOptimizationAction = new BehaviorAction<CleaningMotionState>(CleaningMotionState.CONTROLPOINT_OPTIMIZATION, controlPointOptimizationBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            TextToSpeechPacket p1 = new TextToSpeechPacket("controlPointOptimizationAction");
            sendPacket(p1);
         }
      };

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
            TextToSpeechPacket p1 = new TextToSpeechPacket("gotoReadyPoseAction");
            sendPacket(p1);

            PrintTools.info("gotoReadyPoseAction");
            WholeBodyTrajectoryMessage wholebodyMessage = new WholeBodyTrajectoryMessage();

            SolarPanelCleaningPose readyPose = SolarPanelCleaningInfo.getCleaningPath().getStartPose();
            motionFactory.setMessage(readyPose, Math.PI*0.0, 0.0, 3.0);
            wholebodyMessage = motionFactory.getWholeBodyTrajectoryMessage();
            wholebodyTrajectoryBehavior.setInput(wholebodyMessage);
         }
      };

      BehaviorAction<CleaningMotionState> cleaningAction = new BehaviorAction<CleaningMotionState>(CleaningMotionState.CLEANING_MOTION, wholebodyTrajectoryBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            TextToSpeechPacket p1 = new TextToSpeechPacket("cleaningAction");
            sendPacket(p1);

            PrintTools.info("cleaningAction");
            WholeBodyTrajectoryMessage wholebodyMessage = new WholeBodyTrajectoryMessage();
            motionFactory.setCleaningPath(SolarPanelCleaningInfo.getCleaningPath());
            motionFactory.setMessage(controlPointOptimizationBehavior.getOptimalControlPointNodePath());
            wholebodyMessage = motionFactory.getWholeBodyTrajectoryMessage();
            wholebodyTrajectoryBehavior.setInput(wholebodyMessage);


         }
      };

      BehaviorAction<CleaningMotionState> doneAction = new BehaviorAction<CleaningMotionState>(CleaningMotionState.DONE, doneBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            TextToSpeechPacket p1 = new TextToSpeechPacket("doneAction");
            sendPacket(p1);
         }
      };

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
   }

   private void setUpSolarPanel()
   {
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
