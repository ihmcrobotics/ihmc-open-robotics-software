package us.ihmc.behaviors.activeMapping;

import controller_msgs.msg.dds.FootstepDataListMessage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.networkProcessor.footstepPlanningModule.FootstepPlanningModuleLauncher;
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

public class ActiveMappingModule
{
   private final FootstepPlanningModule footstepPlanner;
   private final DRCRobotModel robotModel;
   private final HumanoidReferenceFrames referenceFrames;
   private final PlanarRegionMap planarRegionMap;

   private FootstepPlannerRequest request;
   private FootstepPlannerOutput plannerOutput;

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
         Pose3D leftSolePose = new Pose3D(referenceFrames.getSoleFrame(RobotSide.LEFT).getTransformToWorldFrame());
         Pose3D rightSolePose = new Pose3D(referenceFrames.getSoleFrame(RobotSide.RIGHT).getTransformToWorldFrame());
         Pose3D midFootPose = new Pose3D(referenceFrames.getMidFeetZUpFrame().getTransformToWorldFrame());

         Pose3D goalFeetPose = new Pose3D(midFootPose);
         goalFeetPose.appendTranslation(1.0, 0.0, 0.0);

         LogTools.info("Start Pose: {}, Goal Pose: {}", leftSolePose, goalFeetPose);

         request = new FootstepPlannerRequest();
         request.setTimeout(0.25);
         request.setStartFootPoses(leftSolePose, rightSolePose);
         request.setPlanarRegionsList(planarRegionMap.getMapRegions());
         //request.setAssumeFlatGround(true);
         request.setPlanBodyPath(false);
         request.setGoalFootPoses(0.22, goalFeetPose);
         request.setSnapGoalSteps(true);
         request.setPerformAStarSearch(true);

         plannerOutput = footstepPlanner.handleRequest(request);

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
