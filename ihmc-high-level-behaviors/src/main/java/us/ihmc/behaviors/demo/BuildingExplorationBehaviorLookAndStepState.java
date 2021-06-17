package us.ihmc.behaviors.demo;

import controller_msgs.msg.dds.FootstepStatusMessage;
import controller_msgs.msg.dds.PlanarRegionsListMessage;
import controller_msgs.msg.dds.RobotConfigurationData;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.behaviors.lookAndStep.LookAndStepBehavior;
import us.ihmc.behaviors.lookAndStep.LookAndStepBehaviorAPI;
import us.ihmc.behaviors.tools.BehaviorHelper;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.footstepPlanning.graphSearch.collision.BodyCollisionData;
import us.ihmc.footstepPlanning.graphSearch.parameters.DefaultFootstepPlannerParameters;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersBasics;
import us.ihmc.footstepPlanning.tools.PlannerTools;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.stateMachine.core.State;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicReference;

class BuildingExplorationBehaviorLookAndStepState implements State
{
   private static final double horizonFromDebrisToStop = 0.8;
   private static final double horizonForStairs = 1.0;
   private static final double heightIncreaseForStairs = 0.55;

   private static final double debrisCheckBodyBoxWidth = 0.3;
   private static final double debrisCheckBodyBoxDepth = 0.8;
   private static final double debrisCheckBodyBoxHeight = 1.5;
   private static final double debrisCheckBodyBoxBaseZ = 0.5;
   private static final int numberOfStepsToIgnoreDebrisAfterClearing = 4;

   private final BuildingExplorationBehaviorOld buildingExplorationBehavior;
   private final BehaviorHelper helper;

   private final Pose3DReadOnly bombPose;

   private final FootstepPlannerParametersBasics footstepPlannerParameters;

   private final AtomicReference<List<Pose3D>> bodyPath;
   private final AtomicReference<PlanarRegionsListMessage> planarRegions = new AtomicReference<>();
   private final AtomicReference<RobotConfigurationData> robotConfigurationData = new AtomicReference<>();

   private final AtomicBoolean debrisDetected = new AtomicBoolean();
   private final AtomicBoolean stairsDetected = new AtomicBoolean();

   private final AtomicInteger stepCounter = new AtomicInteger();
   private final ROS2SyncedRobotModel syncedRobot;
   private int numberOfStepsToIgnoreDebris = 0;

   private Runnable debrisDetectedCallback = () ->
   {
   };
   private Runnable stairsDetectedCallback = () ->
   {
   };

   private LookAndStepBehavior.State currentState = LookAndStepBehavior.State.RESET;

   boolean lookAndStepStarted = false;

   public BuildingExplorationBehaviorLookAndStepState(BuildingExplorationBehaviorOld buildingExplorationBehavior, BehaviorHelper helper, Pose3DReadOnly bombPose)
   {
      this.buildingExplorationBehavior = buildingExplorationBehavior;
      this.helper = helper;
      this.bombPose = bombPose;
      String robotName = helper.getRobotModel().getSimpleRobotName();

      this.footstepPlannerParameters = new DefaultFootstepPlannerParameters();
      this.footstepPlannerParameters.setBodyBoxDepth(debrisCheckBodyBoxWidth);
      this.footstepPlannerParameters.setBodyBoxWidth(debrisCheckBodyBoxDepth);
      this.footstepPlannerParameters.setBodyBoxHeight(debrisCheckBodyBoxHeight);
      this.footstepPlannerParameters.setBodyBoxBaseZ(debrisCheckBodyBoxBaseZ);

      syncedRobot = helper.newSyncedRobot();

      bodyPath = helper.subscribeViaReference(LookAndStepBehaviorAPI.BodyPathPlanForUI, new ArrayList<>());
      helper.subscribeViaCallback(ROS2Tools.LIDAR_REA_REGIONS, planarRegions::set);
      helper.subscribeToRobotConfigurationDataViaCallback(robotConfigurationData::set);
      helper.subscribeToControllerViaCallback(FootstepStatusMessage.class, footstepStatusMessage ->
      {
         if (footstepStatusMessage.getFootstepStatus() == FootstepStatusMessage.FOOTSTEP_STATUS_COMPLETED)
         {
            stepCounter.incrementAndGet();
         }
      });
      helper.subscribeViaCallback(LookAndStepBehaviorAPI.CurrentState, state ->
      {
         currentState = LookAndStepBehavior.State.valueOf(state);
      });
   }

   @Override
   public void onEntry()
   {
      BuildingExplorationBehaviorOld.pitchChestToSeeDoor(syncedRobot, helper);

      LogTools.info("Entering " + getClass().getSimpleName());

      planarRegions.set(null);
      bodyPath.set(null);
      debrisDetected.set(false);
      stairsDetected.set(false);
      stepCounter.set(0);
      lookAndStepStarted = false;

      helper.publish(LookAndStepBehaviorAPI.RESET);

      LogTools.info("Enabling look and step behavior");
      buildingExplorationBehavior.getLookAndStepBehavior().setEnabled(true);
      ThreadTools.sleep(100);

      if (!currentState.equals(LookAndStepBehavior.State.BODY_PATH_PLANNING))
      {
         LogTools.info("Waiting for BODY_PATH_PLANNING state...");
      }
   }

   @Override
   public void doAction(double timeInState)
   {
      if (!lookAndStepStarted && currentState.equals(LookAndStepBehavior.State.BODY_PATH_PLANNING))
      {
         lookAndStepStarted = true;
         LogTools.info("Look and step is in BODY_PATH_PLANNING state. Proceeding...");

         boolean operatorReviewEnabled = false;
         LogTools.info("Sending operator review enabled: {}", operatorReviewEnabled);
         helper.publish(LookAndStepBehaviorAPI.OperatorReviewEnabled, operatorReviewEnabled);
         ThreadTools.sleep(100);

         LogTools.info("Publishing goal pose: {}", bombPose);

         helper.publish(LookAndStepBehaviorAPI.GOAL_INPUT, new Pose3D(bombPose));
      }
      else if (!lookAndStepStarted)
      {
         return;
      }

      if (!debrisDetected.get() && (stepCounter.get() > numberOfStepsToIgnoreDebris))
      {
         checkForDebris();
      }
      if (!stairsDetected.get())
      {
         checkForStairs();
      }
   }

   public void ignoreDebris()
   {
      numberOfStepsToIgnoreDebris = numberOfStepsToIgnoreDebrisAfterClearing;
   }

   private void checkForDebris()
   {
      List<Pose3D> bodyPath = this.bodyPath.get();
      if (bodyPath == null)
      {
         LogTools.info("No body path received");
         return;
      }

      PlanarRegionsListMessage planarRegionsMessage = this.planarRegions.get();
      if (planarRegionsMessage == null)
      {
         LogTools.info("No Lidar regions received");
         return;
      }

      RobotConfigurationData robotConfigurationData = this.robotConfigurationData.get();
      if (robotConfigurationData == null)
      {
         LogTools.info("No robot configuration data received");
         return;
      }

      PlanarRegionsList planarRegionsList = PlanarRegionMessageConverter.convertToPlanarRegionsList(planarRegionsMessage);
      Pose3D rootPose = new Pose3D(new Point3D(robotConfigurationData.getRootTranslation()), robotConfigurationData.getRootOrientation());

      BodyCollisionData collisionData = PlannerTools.detectCollisionsAlongBodyPath(rootPose,
                                                                                   bodyPath,
                                                                                   planarRegionsList,
                                                                                   footstepPlannerParameters,
                                                                                   horizonFromDebrisToStop);
      if (collisionData != null && collisionData.isCollisionDetected())
      {
         LogTools.debug("Debris detected");
         this.debrisDetected.set(true);
         debrisDetectedCallback.run();
      }
   }

   private void checkForStairs()
   {
      List<Pose3D> bodyPath = this.bodyPath.get();
      if (bodyPath == null)
         return;

      RobotConfigurationData robotConfigurationData = this.robotConfigurationData.get();
      if (robotConfigurationData == null)
         return;

      Pose3D rootPose = new Pose3D(new Point3D(robotConfigurationData.getRootTranslation()), robotConfigurationData.getRootOrientation());
      Pose3D currentPoseAlongBodyPath = new Pose3D();
      Pose3D extrapolatedPoseAlongBodyPath = new Pose3D();

      PlannerTools.extrapolatePose(rootPose, bodyPath, 0.0, currentPoseAlongBodyPath);
      PlannerTools.extrapolatePose(rootPose, bodyPath, horizonForStairs, extrapolatedPoseAlongBodyPath);

      boolean stairsDetected = Math.abs(currentPoseAlongBodyPath.getZ() - extrapolatedPoseAlongBodyPath.getZ()) >= heightIncreaseForStairs;
      if (stairsDetected)
      {
         LogTools.debug("Stairs detected");
         this.stairsDetected.set(true);
         stairsDetectedCallback.run();
      }
   }

   boolean getDebrisDetected()
   {
      return debrisDetected.get();
   }

   boolean getStairsDetected()
   {
      return stairsDetected.get();
   }

   @Override
   public void onExit(double timeInState)
   {
      LogTools.info("Exiting " + getClass().getSimpleName());

      buildingExplorationBehavior.getLookAndStepBehavior().setEnabled(false);

      lookAndStepStarted = false;
      numberOfStepsToIgnoreDebris = 0;
      helper.publish(LookAndStepBehaviorAPI.RESET);
   }

   public void setDebrisDetectedCallback(Runnable debrisDetectedCallback)
   {
      this.debrisDetectedCallback = debrisDetectedCallback;
   }

   public void setStairsDetectedCallback(Runnable stairsDetectedCallback)
   {
      this.stairsDetectedCallback = stairsDetectedCallback;
   }
}
