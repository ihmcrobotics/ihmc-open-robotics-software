package us.ihmc.humanoidBehaviors.behaviors.roughTerrain;

import javax.vecmath.AxisAngle4d;
import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Tuple3d;

import us.ihmc.communication.packets.TextToSpeechPacket;
import us.ihmc.humanoidBehaviors.behaviors.AbstractBehavior;
import us.ihmc.humanoidBehaviors.behaviors.behaviorServices.FiducialDetectorBehaviorService;
import us.ihmc.humanoidBehaviors.behaviors.examples.UserValidationExampleBehavior;
import us.ihmc.humanoidBehaviors.behaviors.fiducialLocation.FindFiducialBehavior;
import us.ihmc.humanoidBehaviors.behaviors.primitives.AtlasPrimitiveActions;
import us.ihmc.humanoidBehaviors.behaviors.roughTerrain.WalkOverTerrainStateMachineBehavior.WalkOverTerrainState;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.BehaviorAction;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.SimpleDoNothingBehavior;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.SleepBehavior;
import us.ihmc.humanoidBehaviors.communication.CommunicationBridge;
import us.ihmc.humanoidBehaviors.stateMachine.StateMachineBehavior;
import us.ihmc.humanoidRobotics.communication.packets.sensing.DepthDataFilterParameters;
import us.ihmc.humanoidRobotics.communication.packets.sensing.DepthDataStateCommand.LidarState;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataListMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.HeadTrajectoryMessage;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.tools.taskExecutor.PipeLine;

public class WalkOverTerrainStateMachineBehavior extends StateMachineBehavior<WalkOverTerrainState>
{
   public enum WalkOverTerrainState
   {
      LOOK_FOR_GOAL, LOOK_DOWN_AT_TERRAIN, PLAN_TO_GOAL, TAKE_SOME_STEPS, REACHED_GOAL
   }

   private final CommunicationBridge coactiveBehaviorsNetworkManager;

   private final AtlasPrimitiveActions atlasPrimitiveActions;

   private final FindFiducialBehavior lookForGoalBehavior;
   private final LookDownBehavior lookDownAtTerrainBehavior;
   private final PlanHumanoidFootstepsBehavior planHumanoidFootstepsBehavior;
   private final TakeSomeStepsBehavior takeSomeStepsBehavior;
   private final SimpleDoNothingBehavior reachedGoalBehavior;

   private final UserValidationExampleBehavior userValidationExampleBehavior;
   private final ReferenceFrame midZupFrame;

   private final DoubleYoVariable yoTime;

   private final EnumYoVariable<RobotSide> nextSideToSwing;

   public WalkOverTerrainStateMachineBehavior(CommunicationBridge communicationBridge, DoubleYoVariable yoTime, AtlasPrimitiveActions atlasPrimitiveActions, FullHumanoidRobotModel fullRobotModel,
         HumanoidReferenceFrames referenceFrames, FiducialDetectorBehaviorService fiducialDetectorBehaviorService, int fiducialToTrack)
   {
      super("WalkOverTerrain", WalkOverTerrainState.class, yoTime, communicationBridge);

      this.yoTime = yoTime;

      nextSideToSwing = new EnumYoVariable<>("nextSideToSwing", registry, RobotSide.class);
      nextSideToSwing.set(RobotSide.LEFT);
      midZupFrame = atlasPrimitiveActions.referenceFrames.getMidFeetZUpFrame();
      coactiveBehaviorsNetworkManager = communicationBridge;
      //      coactiveBehaviorsNetworkManager.registerYovaribleForAutoSendToUI(statemachine.getStateYoVariable());

      this.atlasPrimitiveActions = atlasPrimitiveActions;

      //create your behaviors

      lookForGoalBehavior = new FindFiducialBehavior(yoTime, communicationBridge, fullRobotModel, referenceFrames, fiducialDetectorBehaviorService, fiducialToTrack);
      lookDownAtTerrainBehavior = new LookDownBehavior(communicationBridge);
      planHumanoidFootstepsBehavior = new PlanHumanoidFootstepsBehavior(yoTime, communicationBridge, fullRobotModel, referenceFrames, fiducialDetectorBehaviorService);
      takeSomeStepsBehavior = new TakeSomeStepsBehavior(yoTime, communicationBridge, fullRobotModel, referenceFrames);
      reachedGoalBehavior = new SimpleDoNothingBehavior(communicationBridge);

      userValidationExampleBehavior = new UserValidationExampleBehavior(communicationBridge);

      this.registry.addChild(lookForGoalBehavior.getYoVariableRegistry());
      this.registry.addChild(lookDownAtTerrainBehavior.getYoVariableRegistry());
      this.registry.addChild(planHumanoidFootstepsBehavior.getYoVariableRegistry());
      this.registry.addChild(takeSomeStepsBehavior.getYoVariableRegistry());
      //      this.registry.addChild(reachedGoalBehavior.getYoVariableRegistry());
      this.registry.addChild(userValidationExampleBehavior.getYoVariableRegistry());

      setupStateMachine();
   }

   @Override
   public void onBehaviorEntered()
   {
      TextToSpeechPacket p1 = new TextToSpeechPacket("Starting Walk Over Terrain Behavior");
      sendPacket(p1);
      statemachine.setCurrentState(WalkOverTerrainState.LOOK_FOR_GOAL);
   }

   private void setupStateMachine()
   {

      BehaviorAction<WalkOverTerrainState> lookForGoalAction = new BehaviorAction<WalkOverTerrainState>(WalkOverTerrainState.LOOK_FOR_GOAL, lookForGoalBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            TextToSpeechPacket p1 = new TextToSpeechPacket("Looking for Goal");
            sendPacket(p1);
         }
      };

      BehaviorAction<WalkOverTerrainState> lookDownAtTerrainAction = new BehaviorAction<WalkOverTerrainState>(WalkOverTerrainState.LOOK_DOWN_AT_TERRAIN, lookDownAtTerrainBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            TextToSpeechPacket p1 = new TextToSpeechPacket("Looking Down at Terrain");
            sendPacket(p1);

            lookDownAtTerrainBehavior.setupPipeLine();
         }
      };

      BehaviorAction<WalkOverTerrainState> planHumanoidFootstepsAction = new BehaviorAction<WalkOverTerrainState>(WalkOverTerrainState.PLAN_TO_GOAL, planHumanoidFootstepsBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            FramePose goalPose = new FramePose();
            lookForGoalBehavior.getFiducialPose(goalPose);
            Tuple3d goalPosition = new Point3d();
            goalPose.getPosition(goalPosition);

            TextToSpeechPacket p1 = new TextToSpeechPacket("Plannning Footsteps to " + goalPosition);
            sendPacket(p1);

            planHumanoidFootstepsBehavior.setGoalPoseAndFirstSwingSide(goalPose, nextSideToSwing.getEnumValue());
         }
      };

      BehaviorAction<WalkOverTerrainState> takeSomeStepsAction = new BehaviorAction<WalkOverTerrainState>(WalkOverTerrainState.TAKE_SOME_STEPS, takeSomeStepsBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            TextToSpeechPacket p1 = new TextToSpeechPacket("Taking some Footsteps");
            sendPacket(p1);

            int maxNumberOfStepsToTake = 4;
            double swingTime = 1.2;
            double transferTime = 0.5;
            FootstepDataListMessage footstepDataListMessageForPlan = planHumanoidFootstepsBehavior.getFootstepDataListMessageForPlan(maxNumberOfStepsToTake, swingTime, transferTime);
            takeSomeStepsBehavior.setFootstepsToTake(footstepDataListMessageForPlan);

            nextSideToSwing.set(footstepDataListMessageForPlan.footstepDataList.get(footstepDataListMessageForPlan.footstepDataList.size() - 1).getRobotSide().getOppositeSide());
         }
      };

      BehaviorAction<WalkOverTerrainState> reachedGoalAction = new BehaviorAction<WalkOverTerrainState>(WalkOverTerrainState.REACHED_GOAL, reachedGoalBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            TextToSpeechPacket p1 = new TextToSpeechPacket("Reached Goal.");
            sendPacket(p1);
         }
      };

      //setup the state machine

      statemachine.addStateWithDoneTransition(lookForGoalAction, WalkOverTerrainState.LOOK_DOWN_AT_TERRAIN);
      statemachine.addStateWithDoneTransition(lookDownAtTerrainAction, WalkOverTerrainState.PLAN_TO_GOAL);
      statemachine.addStateWithDoneTransition(planHumanoidFootstepsAction, WalkOverTerrainState.TAKE_SOME_STEPS);
      statemachine.addStateWithDoneTransition(takeSomeStepsAction, WalkOverTerrainState.LOOK_DOWN_AT_TERRAIN); //REACHED_GOAL);
      //      statemachine.addStateWithDoneTransition(takeSomeStepsAction, WalkOverTerrainState.LOOK_FOR_GOAL);
      statemachine.addStateWithDoneTransition(reachedGoalAction, WalkOverTerrainState.LOOK_DOWN_AT_TERRAIN);
   }

   private class LookDownBehavior extends AbstractBehavior
   {
      private PipeLine<BehaviorAction> pipeLine = new PipeLine<>();

      public LookDownBehavior(CommunicationBridge communicationBridge)
      {
         super(communicationBridge);
      }

      public void setupPipeLine()
      {
         BehaviorAction lookUpAction = new BehaviorAction(atlasPrimitiveActions.headTrajectoryBehavior)
         {
            @Override
            protected void setBehaviorInput()
            {
               AxisAngle4d orientationAxisAngle = new AxisAngle4d(0.0, 1.0, 0.0, Math.PI / 4.0);
               Quat4d headOrientation = new Quat4d();
               headOrientation.set(orientationAxisAngle);
               HeadTrajectoryMessage headTrajectoryMessage = new HeadTrajectoryMessage(0.5, headOrientation);
               atlasPrimitiveActions.headTrajectoryBehavior.setInput(headTrajectoryMessage);
            }
         };

         BehaviorAction lookDownAction = new BehaviorAction(atlasPrimitiveActions.headTrajectoryBehavior)
         {
            @Override
            protected void setBehaviorInput()
            {
               AxisAngle4d orientationAxisAngle = new AxisAngle4d(0.0, 1.0, 0.0, Math.PI / 2.0);
               Quat4d headOrientation = new Quat4d();
               headOrientation.set(orientationAxisAngle);
               HeadTrajectoryMessage headTrajectoryMessage = new HeadTrajectoryMessage(1.0, headOrientation);
               atlasPrimitiveActions.headTrajectoryBehavior.setInput(headTrajectoryMessage);
            }
         };

         //ENABLE LIDAR
         BehaviorAction enableLidarTask = new BehaviorAction(atlasPrimitiveActions.enableLidarBehavior)
         {
            @Override
            protected void setBehaviorInput()
            {
               atlasPrimitiveActions.enableLidarBehavior.setLidarState(LidarState.ENABLE);
            }
         };

         //REDUCE LIDAR RANGE *******************************************

         BehaviorAction setLidarMediumRangeTask = new BehaviorAction(atlasPrimitiveActions.setLidarParametersBehavior)
         {
            @Override
            protected void setBehaviorInput()
            {

               DepthDataFilterParameters param = new DepthDataFilterParameters();
               param.nearScanRadius = 4.0f;
               atlasPrimitiveActions.setLidarParametersBehavior.setInput(param);
            }
         };

         //CLEAR LIDAR POINTS FOR CLEAN SCAN *******************************************
         BehaviorAction clearLidarTask = new BehaviorAction(atlasPrimitiveActions.clearLidarBehavior);

         final SleepBehavior sleepBehavior = new SleepBehavior(communicationBridge, yoTime);
         BehaviorAction sleepTask = new BehaviorAction(sleepBehavior)
         {
            @Override
            protected void setBehaviorInput()
            {
               sleepBehavior.setSleepTime(2.0);
            }
         };

         pipeLine.clearAll();
         //         pipeLine.submitSingleTaskStage(lookUpAction);
         pipeLine.submitSingleTaskStage(lookDownAction);
         pipeLine.submitSingleTaskStage(enableLidarTask);
         pipeLine.submitSingleTaskStage(setLidarMediumRangeTask);
         pipeLine.submitSingleTaskStage(clearLidarTask);
         pipeLine.submitSingleTaskStage(sleepTask);
      }

      @Override
      public void doControl()
      {
         pipeLine.doControl();
      }

      @Override
      public boolean isDone()
      {
         return pipeLine.isDone();
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
   }

   @Override
   public void onBehaviorExited()
   {
   }

}
