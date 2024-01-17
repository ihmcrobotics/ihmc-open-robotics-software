package us.ihmc.behaviors.activeMapping;

import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.WalkingStatusMessage;
import perception_msgs.msg.dds.FramePlanarRegionsListMessage;
import perception_msgs.msg.dds.ImageMessage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ControllerAPIDefinition;
import us.ihmc.communication.ros2.ROS2PublisherMap;
import us.ihmc.footstepPlanning.monteCarloPlanning.TerrainPlanningDebugger;
import us.ihmc.footstepPlanning.swing.SwingPlannerParametersBasics;
import us.ihmc.humanoidRobotics.communication.packets.walking.WalkingStatus;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.log.LogTools;
import us.ihmc.perception.headless.LocalizationAndMappingTask;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2Topic;

import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicReference;

public class ActivePlanarMappingRemoteTask extends LocalizationAndMappingTask
{
   private final static long UPDATE_PERIOD_MS = 500;
   private final static long PLANNING_PERIOD_MS = 500;

   private final AtomicReference<WalkingStatusMessage> walkingStatusMessage = new AtomicReference<>(new WalkingStatusMessage());
   private final ROS2PublisherMap publisherMap;
   private final ROS2Topic controllerFootstepDataTopic;

   private ContinuousPlannerForPlanarRegions continuousPlanner;
   private final ContinuousWalkingParameters continuousPlanningParameters;
   private final SwingPlannerParametersBasics swingFootPlannerParameters;
   private TerrainPlanningDebugger terrainPlanningDebugger;


   public ActivePlanarMappingRemoteTask(String simpleRobotName,
                                        DRCRobotModel robotModel,
                                        ContinuousWalkingParameters continuousPlanningParameters,
                                        ROS2Topic<FramePlanarRegionsListMessage> terrainRegionsTopic,
                                        ROS2Topic<FramePlanarRegionsListMessage> structuralRegionsTopic,
                                        ROS2Node ros2Node,
                                        HumanoidReferenceFrames referenceFrames,
                                        Runnable referenceFramesUpdater,
                                        boolean smoothing)
   {
      super(simpleRobotName, terrainRegionsTopic, structuralRegionsTopic, ros2Node, referenceFrames, referenceFramesUpdater, smoothing);

      this.walkingStatusMessage.get().setWalkingStatus(WalkingStatus.COMPLETED.toByte());
      this.terrainPlanningDebugger = new TerrainPlanningDebugger(ros2Node);
      this.controllerFootstepDataTopic = ControllerAPIDefinition.getTopic(FootstepDataListMessage.class, robotModel.getSimpleRobotName());
      this.continuousPlanningParameters = continuousPlanningParameters;
      swingFootPlannerParameters = robotModel.getSwingPlannerParameters();

      continuousPlanner = new ContinuousPlannerForPlanarRegions(referenceFrames);
      publisherMap = new ROS2PublisherMap(ros2Node);
      publisherMap.getOrCreatePublisher(controllerFootstepDataTopic);
      ros2Helper.subscribeViaCallback(terrainRegionsTopic, this::onPlanarRegionsReceived);
      //ros2Helper.subscribeViaCallback(PerceptionAPI.OUSTER_DEPTH_IMAGE, this::onOusterDepthReceived);

      ros2Helper.subscribeViaCallback(ControllerAPIDefinition.getTopic(WalkingStatusMessage.class, robotModel.getSimpleRobotName()), this::walkingStatusReceived);

      executorService.scheduleAtFixedRate(this::updateActiveMappingPlan, 0, PLANNING_PERIOD_MS, TimeUnit.MILLISECONDS);
      executorService.scheduleAtFixedRate(this::generalUpdate, 0, UPDATE_PERIOD_MS, TimeUnit.MILLISECONDS);
   }

   public void onOusterDepthReceived(ImageMessage imageMessage)
   {
      // Convert Ouster depth image to pointcloud




   }

   private void walkingStatusReceived(WalkingStatusMessage walkingStatusMessage)
   {
      LogTools.warn("Received Walking Status Message: {}", walkingStatusMessage);
      this.walkingStatusMessage.set(walkingStatusMessage);
   }

   /**
    * This method is called periodically to update auxiliary services and data
    */
   private void generalUpdate()
   {
//      activeMappingModule.setActive(configurationParameters.getSLAMEnabled());
      //activeMappingModule.getPlanarRegionMap().printStatistics(true);
   }

   /**
    * Scheduled on regular intervals to compute and send the active mapping plan to the controller.
    */
   private void updateActiveMappingPlan()
   {
      if (continuousPlanningParameters.getEnableContinuousWalking())
      {
         if (walkingStatusMessage.get() != null)
         {
            if (walkingStatusMessage.get().getWalkingStatus() == WalkingStatusMessage.COMPLETED && !continuousPlanner.isPlanAvailable())
            {
               continuousPlanner.planBodyPathWithPlanarRegionMap(planarRegionMap);
            }
         }

         if (continuousPlanner.isPlanAvailable())
         {
            // Publishing Plan Result
            FootstepDataListMessage footstepDataList = continuousPlanner.getFootstepDataListMessage();
            publisherMap.publish(controllerFootstepDataTopic, footstepDataList);

            continuousPlanner.setPlanAvailable(false);
         }
//         configurationParameters.setActiveMapping(false);
      }
   }

   public ContinuousPlannerForPlanarRegions getContinuousPlanner()
   {
      return continuousPlanner;
   }
}
