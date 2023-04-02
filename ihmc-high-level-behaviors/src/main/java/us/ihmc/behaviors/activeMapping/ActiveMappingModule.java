package us.ihmc.behaviors.activeMapping;

import controller_msgs.msg.dds.FootstepDataListMessage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.networkProcessor.footstepPlanningModule.FootstepPlanningModuleLauncher;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.footstepPlanning.FootstepDataMessageConverter;
import us.ihmc.footstepPlanning.FootstepPlannerOutput;
import us.ihmc.footstepPlanning.FootstepPlannerRequest;
import us.ihmc.footstepPlanning.FootstepPlanningModule;
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

   private final ScheduledExecutorService executorService = Executors.newSingleThreadScheduledExecutor(ThreadTools.createNamedThreadFactory(
         "ActiveMappingRunner"));

   private PlanarRegionMap planarRegionMap;
   private FootstepPlannerRequest request;
   private FootstepPlannerOutput plannerOutput;

   private Pose3D goalPose = new Pose3D(1.0, 0.0, 0.0, 0.0, 0.0, 0.0);

   private boolean walkingEnabled = false;
   private boolean active = true;
   ;

   public ActiveMappingModule(DRCRobotModel robotModel)
   {
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
      request = new FootstepPlannerRequest();
      request.setTimeout(3.5);
      Pose3D initialMidFootPose = new Pose3D(new Point3D(), new Quaternion());
      request.setStartFootPoses(robotModel.getFootstepPlannerParameters().getIdealFootstepWidth(), initialMidFootPose);
      request.setRequestedInitialStanceSide(RobotSide.LEFT);
      request.setPlanarRegionsList(planarRegionMap.getMapRegions());
      request.setPlanBodyPath(false);
      request.setGoalFootPoses(robotModel.getFootstepPlannerParameters().getIdealFootstepWidth(), goalPose);
      request.setPerformAStarSearch(true);

      plannerOutput = footstepPlanner.handleRequest(request);

      LogTools.info("------------------------ Run -----------------------------");
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

   public boolean isActive()
   {
      return active;
   }
}
