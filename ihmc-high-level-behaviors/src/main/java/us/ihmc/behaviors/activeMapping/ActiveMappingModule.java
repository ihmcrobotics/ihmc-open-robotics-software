package us.ihmc.behaviors.activeMapping;

import controller_msgs.msg.dds.FootstepDataListMessage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.networkProcessor.footstepPlanningModule.FootstepPlanningModuleLauncher;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.footstepPlanning.FootstepDataMessageConverter;
import us.ihmc.footstepPlanning.FootstepPlannerOutput;
import us.ihmc.footstepPlanning.FootstepPlannerRequest;
import us.ihmc.footstepPlanning.FootstepPlanningModule;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.log.LogTools;
import us.ihmc.perception.mapping.PlanarRegionMap;
import us.ihmc.robotics.geometry.FramePlanarRegionsList;
import us.ihmc.robotics.robotSide.RobotSide;

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;

public class ActiveMappingModule
{
   private final FootstepPlanningModule footstepPlanner;
   private final DRCRobotModel robotModel;
   private final HumanoidReferenceFrames referenceFrames;
   private final PlanarRegionMap planarRegionMap;

   private FootstepPlannerRequest request;
   private FootstepPlannerOutput plannerOutput;

   private boolean walkingEnabled = false;
   private boolean active = true;

   public ActiveMappingModule(DRCRobotModel robotModel, HumanoidReferenceFrames humanoidReferenceFrames)
   {
      this.referenceFrames = humanoidReferenceFrames;
      this.planarRegionMap = new PlanarRegionMap(true);
      this.robotModel = robotModel;

      footstepPlanner = FootstepPlanningModuleLauncher.createModule(robotModel);
   }

   public void updateMap(FramePlanarRegionsList regions)
   {
      planarRegionMap.registerRegions(regions.getPlanarRegionsList(), regions.getSensorToWorldFrameTransform());
   }

   public void updateFootstepPlan()
   {
      Pose3D leftSolePose = new Pose3D(referenceFrames.getSoleFrame(RobotSide.LEFT).getTransformToWorldFrame());
      Pose3D rightSolePose = new Pose3D(referenceFrames.getSoleFrame(RobotSide.RIGHT).getTransformToWorldFrame());

      Pose3D leftGoalPose = new Pose3D(leftSolePose);
      leftGoalPose.appendTranslation(1.0, 0.0, 0.0);

      Pose3D rightGoalPose = new Pose3D(rightSolePose);
      rightGoalPose.appendTranslation(1.0, 0.0, 0.0);

      LogTools.info("Start Pose: {}, Goal Pose: {}", leftSolePose, leftGoalPose);

      request = new FootstepPlannerRequest();
      request.setTimeout(0.25);
      request.setStartFootPoses(leftSolePose, rightSolePose);
      request.setPlanarRegionsList(planarRegionMap.getMapRegions());
      request.setPlanBodyPath(false);
      request.setGoalFootPoses(leftGoalPose, rightGoalPose);
      request.setPerformAStarSearch(true);

      plannerOutput = footstepPlanner.handleRequest(request);


      LogTools.info(String.format("Planar Regions: %d\t Plan Length: %d\n",
                                  planarRegionMap.getMapRegions().getNumberOfPlanarRegions(),
                                  footstepPlanner.getOutput().getFootstepPlan().getNumberOfSteps()));
   }

   public PlanarRegionMap getPlanarRegionMap()
   {
      return planarRegionMap;
   }

   public FootstepDataListMessage getFootstepDataListMessage()
   {
      return FootstepDataMessageConverter.createFootstepDataListFromPlan(plannerOutput.getFootstepPlan(), 1.3, 0.4);
   }

   public void reset()
   {
      planarRegionMap.reset();
      planarRegionMap.setModified(true);
   }

   public void setWalkingEnabled(boolean walkingEnabled)
   {
      this.walkingEnabled = walkingEnabled;
   }

   public boolean isWalkingEnabled()
   {
      return walkingEnabled;
   }

   public boolean isActive()
   {
      return active;
   }
}
