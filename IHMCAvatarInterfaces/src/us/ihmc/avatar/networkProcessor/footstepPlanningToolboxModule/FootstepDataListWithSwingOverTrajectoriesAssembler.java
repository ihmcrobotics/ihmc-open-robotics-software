package us.ihmc.avatar.networkProcessor.footstepPlanningToolboxModule;

import java.util.ArrayList;
import java.util.List;

import javax.vecmath.Point2d;
import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;

import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.trajectories.SwingOverPlanarRegionsTrajectoryExpander;
import us.ihmc.commonWalkingControlModules.trajectories.SwingOverPlanarRegionsTrajectoryExpander.SwingOverPlanarRegionsTrajectoryExpansionStatus;
import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.SimpleFootstep;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.communication.packets.ExecutionMode;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataListMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataMessage.FootstepOrigin;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.footstep.Footstep.FootstepType;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.ConvexPolygon2d;
import us.ihmc.robotics.geometry.ConvexPolygonTools;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.trajectories.TrajectoryType;

public class FootstepDataListWithSwingOverTrajectoriesAssembler
{
   private final SwingOverPlanarRegionsTrajectoryExpander swingOverPlanarRegionsTrajectoryExpander;
   private final HumanoidReferenceFrames humanoidReferenceFrames;

   private final FramePose stanceFootPose;
   private final FramePose swingStartPose;
   private final FramePose swingEndPose;
   private final ConvexPolygon2d partialFootholdPolygon;

   public FootstepDataListWithSwingOverTrajectoriesAssembler(HumanoidReferenceFrames humanoidReferenceFrames,
                                                             WalkingControllerParameters walkingControllerParameters, YoVariableRegistry parentRegistry,
                                                             YoGraphicsListRegistry graphicsListRegistry)
   {
      this.humanoidReferenceFrames = humanoidReferenceFrames;

      swingOverPlanarRegionsTrajectoryExpander = new SwingOverPlanarRegionsTrajectoryExpander(walkingControllerParameters, parentRegistry,
                                                                                              graphicsListRegistry);

      stanceFootPose = new FramePose();
      swingStartPose = new FramePose();
      swingEndPose = new FramePose();
      partialFootholdPolygon = new ConvexPolygon2d();
   }
   
   public SwingOverPlanarRegionsTrajectoryExpansionStatus getStatus()
   {
      return swingOverPlanarRegionsTrajectoryExpander.getStatus();
   }

   public FootstepDataListMessage assemble(FootstepPlan footstepPlan, double swingTime, double transferTime, ExecutionMode executionMode,
                                           PlanarRegionsList planarRegionsList)
   {
      FootstepDataListMessage footstepDataListMessage = new FootstepDataListMessage();
      footstepDataListMessage.setSwingTime(swingTime);
      footstepDataListMessage.setTransferTime(transferTime);

      for (int i = 0; i < footstepPlan.getNumberOfSteps(); i++)
      {
         if (i == 0)
         {
            swingStartPose.setToZero(humanoidReferenceFrames.getSoleFrame(footstepPlan.getFootstep(0).getRobotSide()));
            stanceFootPose.setToZero(humanoidReferenceFrames.getSoleFrame(footstepPlan.getFootstep(0).getRobotSide().getOppositeSide()));
         }
         
         SimpleFootstep simpleFootstep = footstepPlan.getFootstep(i);

         simpleFootstep.getSoleFramePose(swingEndPose);

         FootstepDataMessage footstepDataMessage = new FootstepDataMessage(simpleFootstep.getRobotSide(), new Point3d(swingEndPose.getPositionUnsafe()),
                                                                           new Quat4d(swingEndPose.getOrientationUnsafe()));
         footstepDataMessage.setOrigin(FootstepOrigin.AT_SOLE_FRAME);

         swingOverPlanarRegionsTrajectoryExpander.expandTrajectoryOverPlanarRegions(stanceFootPose, swingStartPose, swingEndPose, planarRegionsList);
         footstepDataMessage.setTrajectoryType(TrajectoryType.CUSTOM);
         Point3d[] waypoints = new Point3d[] {new Point3d(), new Point3d()};
         swingOverPlanarRegionsTrajectoryExpander.getExpandedWaypoints().get(0).get(waypoints[0]);
         swingOverPlanarRegionsTrajectoryExpander.getExpandedWaypoints().get(1).get(waypoints[1]);
         footstepDataMessage.setTrajectoryWaypoints(waypoints);

         if (simpleFootstep.hasFoothold())
         {
            simpleFootstep.getFoothold(partialFootholdPolygon);
            
            if (partialFootholdPolygon.getNumberOfVertices() != 4)
            {
               ConvexPolygonTools.limitVerticesConservative(partialFootholdPolygon, 4);
            }
            
            ArrayList<Point2d> fourPartialFootholdCorners = new ArrayList<>();
            for (int j = 0; j < 4; j++)
            {
               fourPartialFootholdCorners.add(new Point2d(partialFootholdPolygon.getVertex(j)));
            }
            
            footstepDataMessage.setPredictedContactPoints(fourPartialFootholdCorners);
         }

         footstepDataListMessage.add(footstepDataMessage);

         swingStartPose.setIncludingFrame(stanceFootPose);
         stanceFootPose.setIncludingFrame(swingEndPose);
      }

      footstepDataListMessage.setExecutionMode(executionMode);
      return footstepDataListMessage;
   }
   
   public FootstepDataListMessage assemble(List<Footstep> footstepList, double swingTime, double transferTime, ExecutionMode executionMode,
                                           PlanarRegionsList planarRegionsList)
   {
      FootstepDataListMessage footstepDataListMessage = new FootstepDataListMessage();
      footstepDataListMessage.setSwingTime(swingTime);
      footstepDataListMessage.setTransferTime(transferTime);

      for (int i = 0; i < footstepList.size(); i++)
      {
         if (i == 0)
         {
            swingStartPose.setToZero(humanoidReferenceFrames.getSoleFrame(footstepList.get(0).getRobotSide()));
            stanceFootPose.setToZero(humanoidReferenceFrames.getSoleFrame(footstepList.get(0).getRobotSide().getOppositeSide()));
         }
         
         Footstep footstep = footstepList.get(i);

         footstep.getSolePose(swingEndPose);

         FootstepDataMessage footstepDataMessage = new FootstepDataMessage(footstep.getRobotSide(), new Point3d(swingEndPose.getPositionUnsafe()),
                                                                           new Quat4d(swingEndPose.getOrientationUnsafe()));
         footstepDataMessage.setOrigin(FootstepOrigin.AT_SOLE_FRAME);

         swingOverPlanarRegionsTrajectoryExpander.expandTrajectoryOverPlanarRegions(stanceFootPose, swingStartPose, swingEndPose, planarRegionsList);
         footstepDataMessage.setTrajectoryType(TrajectoryType.CUSTOM);
         Point3d[] waypoints = new Point3d[] {new Point3d(), new Point3d()};
         swingOverPlanarRegionsTrajectoryExpander.getExpandedWaypoints().get(0).get(waypoints[0]);
         swingOverPlanarRegionsTrajectoryExpander.getExpandedWaypoints().get(1).get(waypoints[1]);
         footstepDataMessage.setTrajectoryWaypoints(waypoints);

         if (footstep.getFootstepType() == FootstepType.PARTIAL_FOOTSTEP)
         {
            partialFootholdPolygon.clear();
            partialFootholdPolygon.addVertices(footstep.getPredictedContactPoints(), footstep.getPredictedContactPoints().size());
            partialFootholdPolygon.update();
            
            if (partialFootholdPolygon.getNumberOfVertices() != 4)
            {
               ConvexPolygonTools.limitVerticesConservative(partialFootholdPolygon, 4);
            }
            
            ArrayList<Point2d> fourPartialFootholdCorners = new ArrayList<>();
            for (int j = 0; j < 4; j++)
            {
               fourPartialFootholdCorners.add(new Point2d(partialFootholdPolygon.getVertex(j)));
            }
            
            footstepDataMessage.setPredictedContactPoints(fourPartialFootholdCorners);
         }

         footstepDataListMessage.add(footstepDataMessage);

         swingStartPose.setIncludingFrame(stanceFootPose);
         stanceFootPose.setIncludingFrame(swingEndPose);
      }

      footstepDataListMessage.setExecutionMode(executionMode);
      return footstepDataListMessage;
   }
}
