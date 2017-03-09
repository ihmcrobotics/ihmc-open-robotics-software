package us.ihmc.humanoidBehaviors.behaviors.roughTerrain;

import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.communication.packets.RequestPlanarRegionsListMessage;
import us.ihmc.communication.packets.RequestPlanarRegionsListMessage.RequestType;
import us.ihmc.communication.packets.TextToSpeechPacket;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidBehaviors.behaviors.AbstractBehavior;
import us.ihmc.humanoidBehaviors.behaviors.examples.GetUserValidationBehavior;
import us.ihmc.humanoidBehaviors.behaviors.goalLocation.FindGoalBehavior;
import us.ihmc.humanoidBehaviors.behaviors.goalLocation.GoalDetectorBehaviorService;
import us.ihmc.humanoidBehaviors.behaviors.primitives.AtlasPrimitiveActions;
import us.ihmc.humanoidBehaviors.behaviors.roughTerrain.WalkOverTerrainStateMachineBehavior.WalkOverTerrainState;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.BehaviorAction;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.SimpleDoNothingBehavior;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.SleepBehavior;
import us.ihmc.humanoidBehaviors.communication.CommunicationBridge;
import us.ihmc.humanoidBehaviors.communication.CommunicationBridgeInterface;
import us.ihmc.humanoidBehaviors.stateMachine.StateMachineBehavior;
import us.ihmc.humanoidRobotics.communication.packets.sensing.DepthDataFilterParameters;
import us.ihmc.humanoidRobotics.communication.packets.sensing.DepthDataStateCommand.LidarState;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataListMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.HeadTrajectoryMessage;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.multicastLogDataProtocol.modelLoaders.LogModelProvider;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.dataStructures.variable.IntegerYoVariable;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.tools.FormattingTools;
import us.ihmc.tools.taskExecutor.PipeLine;

public class WalkOverTerrainStateMachineBehavior extends StateMachineBehavior<WalkOverTerrainState>
{
   public enum WalkOverTerrainState
   {
      LOOK_FOR_GOAL, SLEEP, LOOK_DOWN_AT_TERRAIN, PLAN_TO_GOAL, CLEAR_PLANAR_REGIONS_LIST, TAKE_SOME_STEPS, REACHED_GOAL
   }

   private final CommunicationBridge coactiveBehaviorsNetworkManager;

   private final AtlasPrimitiveActions atlasPrimitiveActions;

   private final FindGoalBehavior lookForGoalBehavior;
   private final SleepBehavior sleepBehavior;
   private final LookDownBehavior lookDownAtTerrainBehavior;
   private final PlanHumanoidFootstepsBehavior planHumanoidFootstepsBehavior;
   private final TakeSomeStepsBehavior takeSomeStepsBehavior;
   private final ClearPlanarRegionsListBehavior clearPlanarRegionsListBehavior;
   private final SimpleDoNothingBehavior reachedGoalBehavior;

   private final GetUserValidationBehavior userValidationExampleBehavior;
   private final ReferenceFrame midZupFrame;

   private final DoubleYoVariable yoTime;

   private final EnumYoVariable<RobotSide> nextSideToSwing;

   private final String prefix = getClass().getSimpleName();
   private final DoubleYoVariable swingTime = new DoubleYoVariable(prefix + "SwingTime", registry);
   private final DoubleYoVariable transferTime = new DoubleYoVariable(prefix + "TransferTime", registry);
   private final IntegerYoVariable maxNumberOfStepsToTake = new IntegerYoVariable(prefix + "NumberOfStepsToTake", registry);

   private final FullRobotModel fullRobotModel;

   public WalkOverTerrainStateMachineBehavior(CommunicationBridge communicationBridge, DoubleYoVariable yoTime, AtlasPrimitiveActions atlasPrimitiveActions, LogModelProvider logModelProvider, FullHumanoidRobotModel fullRobotModel,
                                              HumanoidReferenceFrames referenceFrames, GoalDetectorBehaviorService goalDetectorBehaviorService)
   {
      super(goalDetectorBehaviorService.getClass().getSimpleName(), "WalkOverTerrain_" + goalDetectorBehaviorService.getClass().getSimpleName(), WalkOverTerrainState.class, yoTime, communicationBridge);

      this.yoTime = yoTime;
      this.fullRobotModel = fullRobotModel;

      nextSideToSwing = new EnumYoVariable<>("nextSideToSwing", registry, RobotSide.class);
      nextSideToSwing.set(RobotSide.LEFT);
      midZupFrame = atlasPrimitiveActions.referenceFrames.getMidFeetZUpFrame();
      coactiveBehaviorsNetworkManager = communicationBridge;
      //      coactiveBehaviorsNetworkManager.registerYovaribleForAutoSendToUI(statemachine.getStateYoVariable());

      this.atlasPrimitiveActions = atlasPrimitiveActions;

      //create your behaviors
      this.lookForGoalBehavior = new FindGoalBehavior(yoTime, communicationBridge, fullRobotModel, referenceFrames,
                                                      goalDetectorBehaviorService);

      sleepBehavior = new SleepBehavior(communicationBridge, yoTime);
      sleepBehavior.setSleepTime(2.0);
      lookDownAtTerrainBehavior = new LookDownBehavior(communicationBridge);

      planHumanoidFootstepsBehavior = new PlanHumanoidFootstepsBehavior(yoTime, communicationBridge, fullRobotModel, referenceFrames);
//      planHumanoidFootstepsBehavior.createAndAttachSCSListenerToPlanner();
      planHumanoidFootstepsBehavior.createAndAttachYoVariableServerListenerToPlanner(logModelProvider, fullRobotModel);

      clearPlanarRegionsListBehavior = new ClearPlanarRegionsListBehavior(communicationBridge);
      takeSomeStepsBehavior = new TakeSomeStepsBehavior(yoTime, communicationBridge, fullRobotModel, referenceFrames);
      reachedGoalBehavior = new SimpleDoNothingBehavior(communicationBridge);

      userValidationExampleBehavior = new GetUserValidationBehavior(communicationBridge);

      this.registry.addChild(lookForGoalBehavior.getYoVariableRegistry());
      this.registry.addChild(sleepBehavior.getYoVariableRegistry());
      this.registry.addChild(lookDownAtTerrainBehavior.getYoVariableRegistry());
      this.registry.addChild(planHumanoidFootstepsBehavior.getYoVariableRegistry());
      this.registry.addChild(takeSomeStepsBehavior.getYoVariableRegistry());
      //      this.registry.addChild(reachedGoalBehavior.getYoVariableRegistry());
      this.registry.addChild(userValidationExampleBehavior.getYoVariableRegistry());

      setupStateMachine();

      swingTime.set(1.5);
      transferTime.set(0.3);
      maxNumberOfStepsToTake.set(3);
   }

   @Override
   public void onBehaviorEntered()
   {
      TextToSpeechPacket p1 = new TextToSpeechPacket("Starting Walk Over Terrain Behavior");
      sendPacket(p1);
      super.onBehaviorEntered();
   }

   @Override
   public void onBehaviorExited()
   {
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

      BehaviorAction<WalkOverTerrainState> sleepAction = new BehaviorAction<WalkOverTerrainState>(WalkOverTerrainState.SLEEP, sleepBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            TextToSpeechPacket p1 = new TextToSpeechPacket("Sleeping");
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
            lookForGoalBehavior.getGoalPose(goalPose);
            Tuple3DBasics goalPosition = new Point3D();
            goalPose.getPosition(goalPosition);

            String xString = FormattingTools.getFormattedToSignificantFigures(goalPosition.getX(), 3);
            String yString = FormattingTools.getFormattedToSignificantFigures(goalPosition.getY(), 3);

            TextToSpeechPacket p1 = new TextToSpeechPacket("Plannning Footsteps to (" + xString + ", " + yString + ")");
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

            FootstepDataListMessage footstepDataListMessageForPlan = planHumanoidFootstepsBehavior.getFootstepDataListMessageForPlan(maxNumberOfStepsToTake.getIntegerValue(), swingTime.getDoubleValue(), transferTime.getDoubleValue());
            takeSomeStepsBehavior.setFootstepsToTake(footstepDataListMessageForPlan);

            nextSideToSwing.set(footstepDataListMessageForPlan.footstepDataList.get(footstepDataListMessageForPlan.footstepDataList.size() - 1).getRobotSide().getOppositeSide());
         }
      };

      BehaviorAction<WalkOverTerrainState> clearPlanarRegionsListAction = new BehaviorAction<WalkOverTerrainState>(WalkOverTerrainState.CLEAR_PLANAR_REGIONS_LIST, clearPlanarRegionsListBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            TextToSpeechPacket p1 = new TextToSpeechPacket("Clearing Planar Regions List.");
            sendPacket(p1);
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

      statemachine.addStateWithDoneTransition(lookForGoalAction, WalkOverTerrainState.SLEEP);
      statemachine.addStateWithDoneTransition(sleepAction, WalkOverTerrainState.LOOK_DOWN_AT_TERRAIN);
      statemachine.addStateWithDoneTransition(lookDownAtTerrainAction, WalkOverTerrainState.PLAN_TO_GOAL);
      statemachine.addStateWithDoneTransition(planHumanoidFootstepsAction, WalkOverTerrainState.CLEAR_PLANAR_REGIONS_LIST);
      statemachine.addStateWithDoneTransition(clearPlanarRegionsListAction, WalkOverTerrainState.TAKE_SOME_STEPS);
      statemachine.addStateWithDoneTransition(takeSomeStepsAction, WalkOverTerrainState.PLAN_TO_GOAL); //REACHED_GOAL);
//      statemachine.addStateWithDoneTransition(takeSomeStepsAction, WalkOverTerrainState.LOOK_DOWN_AT_TERRAIN); //REACHED_GOAL);
      //      statemachine.addStateWithDoneTransition(takeSomeStepsAction, WalkOverTerrainState.LOOK_FOR_GOAL);
      statemachine.addStateWithDoneTransition(reachedGoalAction, WalkOverTerrainState.LOOK_DOWN_AT_TERRAIN);
      
      //the state machine will transition into this state every time the behavior starts.
      statemachine.setStartState(WalkOverTerrainState.LOOK_FOR_GOAL);

   }

   private class ClearPlanarRegionsListBehavior extends AbstractBehavior
   {

      public ClearPlanarRegionsListBehavior(CommunicationBridgeInterface communicationBridge)
      {
         super(communicationBridge);
      }

      @Override
      public void doControl()
      {
      }

      @Override
      public void onBehaviorEntered()
      {
         clearPlanarRegionsList();
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
         return true;
      }

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
         ReferenceFrame chestCoMFrame = fullRobotModel.getChest().getBodyFixedFrame();
         
         BehaviorAction lookUpAction = new BehaviorAction(atlasPrimitiveActions.headTrajectoryBehavior)
         {
            @Override
            protected void setBehaviorInput()
            {
               AxisAngle orientationAxisAngle = new AxisAngle(0.0, 1.0, 0.0, Math.PI / 4.0);
               Quaternion headOrientation = new Quaternion();
               headOrientation.set(orientationAxisAngle);
               HeadTrajectoryMessage headTrajectoryMessage = new HeadTrajectoryMessage(0.5, headOrientation, ReferenceFrame.getWorldFrame(), chestCoMFrame);
               atlasPrimitiveActions.headTrajectoryBehavior.setInput(headTrajectoryMessage);
            }
         };

         BehaviorAction lookDownAction = new BehaviorAction(atlasPrimitiveActions.headTrajectoryBehavior)
         {
            @Override
            protected void setBehaviorInput()
            {
               AxisAngle orientationAxisAngle = new AxisAngle(0.0, 1.0, 0.0, Math.PI / 2.0);
               Quaternion headOrientation = new Quaternion();
               headOrientation.set(orientationAxisAngle);
               HeadTrajectoryMessage headTrajectoryMessage = new HeadTrajectoryMessage(2.0, headOrientation, ReferenceFrame.getWorldFrame(), chestCoMFrame);
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
               sleepBehavior.setSleepTime(4.0);
            }
         };

         pipeLine.clearAll();
         //         pipeLine.submitSingleTaskStage(lookUpAction);
         pipeLine.submitSingleTaskStage(enableLidarTask);
         pipeLine.submitSingleTaskStage(setLidarMediumRangeTask);
//         pipeLine.submitSingleTaskStage(clearLidarTask);
//         pipeLine.submitSingleTaskStage(clearPlanarRegionsListTask);
         pipeLine.submitSingleTaskStage(lookDownAction);
//         pipeLine.submitSingleTaskStage(sleepTask);
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


   private void clearPlanarRegionsList()
   {
      RequestPlanarRegionsListMessage requestPlanarRegionsListMessage = new RequestPlanarRegionsListMessage(RequestType.CLEAR);
      requestPlanarRegionsListMessage.setDestination(PacketDestination.REA_MODULE);
      sendPacket(requestPlanarRegionsListMessage);
   }


}
