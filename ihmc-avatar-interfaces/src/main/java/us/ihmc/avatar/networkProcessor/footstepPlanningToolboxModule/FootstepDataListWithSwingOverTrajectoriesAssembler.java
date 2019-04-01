package us.ihmc.avatar.networkProcessor.footstepPlanningToolboxModule;

import java.util.ArrayList;
import java.util.List;

import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepDataMessage;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.trajectories.SwingOverPlanarRegionsTrajectoryExpander;
import us.ihmc.commonWalkingControlModules.trajectories.SwingOverPlanarRegionsTrajectoryExpander.SwingOverPlanarRegionsTrajectoryExpansionStatus;
import us.ihmc.communication.packets.ExecutionMode;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.Vertex2DSupplier;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.footstep.SimpleFootstep;
import us.ihmc.humanoidRobotics.footstep.Footstep.FootstepType;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.robotics.geometry.ConvexPolygonTools;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.trajectories.TrajectoryType;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class FootstepDataListWithSwingOverTrajectoriesAssembler
{
   private final SwingOverPlanarRegionsTrajectoryExpander swingOverPlanarRegionsTrajectoryExpander;
   private final HumanoidReferenceFrames humanoidReferenceFrames;

   private final FramePose3D stanceFootPose;
   private final FramePose3D swingStartPose;
   private final FramePose3D swingEndPose;
   private final ConvexPolygon2D partialFootholdPolygon;

   private static final double maxSwingSpeed = 1.0;

   public FootstepDataListWithSwingOverTrajectoriesAssembler(HumanoidReferenceFrames humanoidReferenceFrames,
                                                             WalkingControllerParameters walkingControllerParameters, YoVariableRegistry parentRegistry,
                                                             YoGraphicsListRegistry graphicsListRegistry)
   {
      this.humanoidReferenceFrames = humanoidReferenceFrames;

      swingOverPlanarRegionsTrajectoryExpander = new SwingOverPlanarRegionsTrajectoryExpander(walkingControllerParameters, parentRegistry,
                                                                                              graphicsListRegistry);

      stanceFootPose = new FramePose3D();
      swingStartPose = new FramePose3D();
      swingEndPose = new FramePose3D();
      partialFootholdPolygon = new ConvexPolygon2D();
   }

   public SwingOverPlanarRegionsTrajectoryExpansionStatus getStatus()
   {
      return swingOverPlanarRegionsTrajectoryExpander.getStatus();
   }

   public FootstepDataListMessage assemble(FootstepPlan footstepPlan, double swingTime, double transferTime, ExecutionMode executionMode,
                                           PlanarRegionsList planarRegionsList)
   {
      FootstepDataListMessage footstepDataListMessage = new FootstepDataListMessage();
      footstepDataListMessage.setDefaultSwingDuration(swingTime);
      footstepDataListMessage.setDefaultTransferDuration(transferTime);

      for (int i = 0; i < footstepPlan.getNumberOfSteps(); i++)
      {
         if (i == 0)
         {
            swingStartPose.setToZero(humanoidReferenceFrames.getSoleFrame(footstepPlan.getFootstep(0).getRobotSide()));
            stanceFootPose.setToZero(humanoidReferenceFrames.getSoleFrame(footstepPlan.getFootstep(0).getRobotSide().getOppositeSide()));
         }

         SimpleFootstep simpleFootstep = footstepPlan.getFootstep(i);

         simpleFootstep.getSoleFramePose(swingEndPose);

         FootstepDataMessage footstepDataMessage = HumanoidMessageTools.createFootstepDataMessage(simpleFootstep.getRobotSide(), new Point3D(swingEndPose.getPosition()), new Quaternion(swingEndPose.getOrientation()));

         double maxSpeedDimensionless = swingOverPlanarRegionsTrajectoryExpander.expandTrajectoryOverPlanarRegions(stanceFootPose, swingStartPose, swingEndPose, planarRegionsList);
         footstepDataMessage.setTrajectoryType(TrajectoryType.CUSTOM.toByte());
         Point3D[] waypoints = new Point3D[] {new Point3D(), new Point3D()};
         waypoints[0].set(swingOverPlanarRegionsTrajectoryExpander.getExpandedWaypoints().get(0));
         waypoints[1].set(swingOverPlanarRegionsTrajectoryExpander.getExpandedWaypoints().get(1));
         MessageTools.copyData(waypoints, footstepDataMessage.getCustomPositionWaypoints());

         if (simpleFootstep.hasFoothold())
         {
            simpleFootstep.getFoothold(partialFootholdPolygon);

            if (partialFootholdPolygon.getNumberOfVertices() != 4)
            {
               ConvexPolygonTools.limitVerticesConservative(partialFootholdPolygon, 4);
            }

            ArrayList<Point2D> fourPartialFootholdCorners = new ArrayList<>();
            for (int j = 0; j < 4; j++)
            {
               fourPartialFootholdCorners.add(new Point2D(partialFootholdPolygon.getVertex(j)));
            }

            HumanoidMessageTools.packPredictedContactPoints(fourPartialFootholdCorners, footstepDataMessage);
         }

         double maxSpeed = maxSpeedDimensionless / swingTime;
         if (maxSpeed > maxSwingSpeed)
         {
            double adjustedSwingTime = maxSpeedDimensionless / maxSwingSpeed;
            footstepDataMessage.setSwingDuration(adjustedSwingTime);
            footstepDataMessage.setTransferDuration(transferTime);
         }

         footstepDataListMessage.getFootstepDataList().add().set(footstepDataMessage);

         swingStartPose.setIncludingFrame(stanceFootPose);
         stanceFootPose.setIncludingFrame(swingEndPose);
      }

      footstepDataListMessage.getQueueingProperties().setExecutionMode(executionMode.toByte());
      footstepDataListMessage.getQueueingProperties().setPreviousMessageId(FootstepDataListMessage.VALID_MESSAGE_DEFAULT_ID);
      return footstepDataListMessage;
   }

   public FootstepDataListMessage assemble(List<Footstep> footstepList, double swingTime, double transferTime, ExecutionMode executionMode,
                                           PlanarRegionsList planarRegionsList)
   {
      FootstepDataListMessage footstepDataListMessage = new FootstepDataListMessage();
      footstepDataListMessage.setDefaultSwingDuration(swingTime);
      footstepDataListMessage.setDefaultTransferDuration(transferTime);

      for (int i = 0; i < footstepList.size(); i++)
      {
         if (i == 0)
         {
            swingStartPose.setToZero(humanoidReferenceFrames.getSoleFrame(footstepList.get(0).getRobotSide()));
            stanceFootPose.setToZero(humanoidReferenceFrames.getSoleFrame(footstepList.get(0).getRobotSide().getOppositeSide()));
         }

         Footstep footstep = footstepList.get(i);
         footstep.getPose(swingEndPose);

         FootstepDataMessage footstepDataMessage = HumanoidMessageTools.createFootstepDataMessage(footstep.getRobotSide(), new Point3D(swingEndPose.getPosition()), new Quaternion(swingEndPose.getOrientation()));

         swingOverPlanarRegionsTrajectoryExpander.expandTrajectoryOverPlanarRegions(stanceFootPose, swingStartPose, swingEndPose, planarRegionsList);
         footstepDataMessage.setTrajectoryType(TrajectoryType.CUSTOM.toByte());
         Point3D[] waypoints = new Point3D[] {new Point3D(), new Point3D()};
         waypoints[0].set(swingOverPlanarRegionsTrajectoryExpander.getExpandedWaypoints().get(0));
         waypoints[1].set(swingOverPlanarRegionsTrajectoryExpander.getExpandedWaypoints().get(1));
         MessageTools.copyData(waypoints, footstepDataMessage.getCustomPositionWaypoints());

         if (footstep.getFootstepType() == FootstepType.PARTIAL_FOOTSTEP)
         {
            partialFootholdPolygon.clear();
            partialFootholdPolygon.addVertices(Vertex2DSupplier.asVertex2DSupplier(footstep.getPredictedContactPoints()));
            partialFootholdPolygon.update();

            if (partialFootholdPolygon.getNumberOfVertices() != 4)
            {
               ConvexPolygonTools.limitVerticesConservative(partialFootholdPolygon, 4);
            }

            ArrayList<Point2D> fourPartialFootholdCorners = new ArrayList<>();
            for (int j = 0; j < 4; j++)
            {
               fourPartialFootholdCorners.add(new Point2D(partialFootholdPolygon.getVertex(j)));
            }

            
            HumanoidMessageTools.packPredictedContactPoints(fourPartialFootholdCorners, footstepDataMessage);
         }

         footstepDataListMessage.getFootstepDataList().add().set(footstepDataMessage);

         swingStartPose.setIncludingFrame(stanceFootPose);
         stanceFootPose.setIncludingFrame(swingEndPose);
      }

      footstepDataListMessage.getQueueingProperties().setExecutionMode(executionMode.toByte());
      footstepDataListMessage.getQueueingProperties().setPreviousMessageId(FootstepDataListMessage.VALID_MESSAGE_DEFAULT_ID);
      return footstepDataListMessage;
   }

   public void setCollisionSphereRadius(double collisionSphereRadius)
   {
      this.swingOverPlanarRegionsTrajectoryExpander.setCollisionSphereRadius(collisionSphereRadius);
   }
}
