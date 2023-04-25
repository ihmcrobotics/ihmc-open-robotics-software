package us.ihmc.behaviors.activeMapping;

import controller_msgs.msg.dds.FootstepDataListMessage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.networkProcessor.footstepPlanningModule.FootstepPlanningModuleLauncher;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.footstepPlanning.*;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.log.LogTools;
import us.ihmc.perception.mapping.PlanarRegionMap;
import us.ihmc.perception.tools.ActiveMappingTools;
import us.ihmc.robotics.geometry.FramePlanarRegionsList;
import us.ihmc.robotics.geometry.PlanarRegionTools;
import us.ihmc.robotics.robotSide.RobotSide;

import java.util.ArrayList;

public class ActiveMappingModule
{
   public enum ActiveMappingMode
   {
      EXECUTE_AND_PAUSE, CONTINUOUS_MAPPING_STRAIGHT, CONTINUOUS_MAPPING_COVERAGE, CONTINUOUS_MAPPING_SEARCH
   }

   public ActiveMappingMode activeMappingMode = ActiveMappingMode.CONTINUOUS_MAPPING_STRAIGHT;

   private final FootstepPlanningModule footstepPlanner;
   private final DRCRobotModel robotModel;
   private final HumanoidReferenceFrames referenceFrames;
   private final PlanarRegionMap planarRegionMap;

   private FootstepPlannerRequest request;
   private FootstepPlannerOutput plannerOutput;

   private Pose3D leftGoalPose = new Pose3D();
   private Pose3D rightGoalPose = new Pose3D();
   private Pose3D leftSolePose = new Pose3D();
   private Pose3D rightSolePose = new Pose3D();

   private ArrayList<Point2D> frontierPoints = new ArrayList<>();

   private boolean planAvailable = false;
   private boolean active = false;

   public ActiveMappingModule(DRCRobotModel robotModel, HumanoidReferenceFrames humanoidReferenceFrames)
   {
      this.referenceFrames = humanoidReferenceFrames;
      this.planarRegionMap = new PlanarRegionMap(true);
      this.robotModel = robotModel;

      footstepPlanner = FootstepPlanningModuleLauncher.createModule(robotModel);

      active = true;
   }

   public void updateMap(FramePlanarRegionsList regions)
   {
      if(active)
      {
         planarRegionMap.registerRegions(regions.getPlanarRegionsList(), regions.getSensorToWorldFrameTransform());
      }
   }

   public void updateFootstepPlan()
   {
      if(active)
      {
         leftSolePose.set(referenceFrames.getSoleFrame(RobotSide.LEFT).getTransformToWorldFrame());
         rightSolePose.set(referenceFrames.getSoleFrame(RobotSide.RIGHT).getTransformToWorldFrame());
         leftGoalPose.setToZero();
         rightGoalPose.setToZero();

         ActiveMappingTools.setStraightGoalFootPoses(leftSolePose, rightSolePose, leftGoalPose, rightGoalPose, 0.6f);

         request = new FootstepPlannerRequest();
         request.setTimeout(0.25);
         request.setStartFootPoses(leftSolePose, rightSolePose);
         request.setPlanarRegionsList(planarRegionMap.getMapRegions());
         request.setPlanBodyPath(false);
         request.setGoalFootPoses(leftGoalPose, rightGoalPose);
         request.setPerformAStarSearch(true);

         plannerOutput = footstepPlanner.handleRequest(request);

         FootstepPlanningResult footstepPlanningResult = plannerOutput.getFootstepPlanningResult();

         LogTools.info("Footstep Planning Result: {}", footstepPlanningResult);

         LogTools.info(String.format("Planar Regions: %d\t, First Area: %.2f\t Plan Length: %d\n",
                                     planarRegionMap.getMapRegions().getNumberOfPlanarRegions(),
                                     planarRegionMap.getMapRegions().getPlanarRegion(0).getArea(),
                                     footstepPlanner.getOutput().getFootstepPlan().getNumberOfSteps()));

         planAvailable = footstepPlanner.getOutput().getFootstepPlan().getNumberOfSteps() > 0;
      }
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

   public void setPlanAvailable(boolean planAvailable)
   {
      this.planAvailable = planAvailable;
   }

   public boolean isPlanAvailable()
   {
      return planAvailable;
   }

   public boolean isActive()
   {
      return active;
   }

   public void setActive(boolean active)
   {
      this.active = active;
   }
}
