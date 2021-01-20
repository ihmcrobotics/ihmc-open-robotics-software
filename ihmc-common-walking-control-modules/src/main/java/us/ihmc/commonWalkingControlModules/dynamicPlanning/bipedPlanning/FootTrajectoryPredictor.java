package us.ihmc.commonWalkingControlModules.dynamicPlanning.bipedPlanning;

import us.ihmc.commons.InterpolationTools;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.robotics.math.trajectories.core.FramePolynomial3D;
import us.ihmc.robotics.math.trajectories.generators.MultipleSegmentPositionTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.generators.MultipleWaypointsPoseTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.generators.MultipleWaypointsPositionTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.interfaces.FixedFramePolynomial3DBasics;
import us.ihmc.robotics.math.trajectories.trajectorypoints.YoFrameEuclideanTrajectoryPoint;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.ArrayList;
import java.util.List;

public class FootTrajectoryPredictor
{
   private static final double swingHeight = 0.15;
   private static final double interpolationFactor = 0.25;

   private final MultipleWaypointsPositionTrajectoryGenerator leftFootTrajectory;
   private final MultipleWaypointsPositionTrajectoryGenerator rightFootTrajectory;
   private final SideDependentList<MultipleWaypointsPositionTrajectoryGenerator> footTrajectories;

   private MultipleWaypointsPoseTrajectoryGenerator swingTrajectory;

   public FootTrajectoryPredictor(YoRegistry parentRegistry)
   {
      YoRegistry registry = new YoRegistry(getClass().getSimpleName());

      leftFootTrajectory = new MultipleWaypointsPositionTrajectoryGenerator("leftFootPredictedTrajectory", ReferenceFrame.getWorldFrame(), registry);
      rightFootTrajectory = new MultipleWaypointsPositionTrajectoryGenerator("rightFootPredictedTrajectory", ReferenceFrame.getWorldFrame(), registry);

      footTrajectories = new SideDependentList<>(leftFootTrajectory, rightFootTrajectory);

      parentRegistry.addChild(registry);
   }

   public void setSwingTrajectory(MultipleWaypointsPoseTrajectoryGenerator swingTrajectory)
   {
      this.swingTrajectory = swingTrajectory;
   }

   public void compute(CoPTrajectoryGeneratorState state)
   {
      if (state.getNumberOfFootstep() > 0)
         computeWalking(state);
      else
         computeStanding(state);
   }

   private final FrameVector3DReadOnly zeroVector = new FrameVector3D();

   private void computeStanding(CoPTrajectoryGeneratorState state)
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         MultipleWaypointsPositionTrajectoryGenerator footTrajectory = footTrajectories.get(robotSide);
         footTrajectory.clear();
         footTrajectory.appendWaypoint(0.0, state.getFootPose(robotSide).getPosition(), zeroVector);
         footTrajectory.appendWaypoint(Double.POSITIVE_INFINITY, state.getFootPose(robotSide).getPosition(), zeroVector);
         footTrajectory.initialize();
      }
   }

   private void computeWalking(CoPTrajectoryGeneratorState state)
   {
      PlanningTiming timing = state.getTiming(0);
      DynamicPlanningFootstep footstep = state.getFootstep(0);

      for (RobotSide robotSide : RobotSide.values)
      {
         MultipleWaypointsPositionTrajectoryGenerator footTrajectory = footTrajectories.get(robotSide);
         footTrajectory.clear();
         footTrajectory.appendWaypoint(0.0, state.getFootPose(robotSide).getPosition(), zeroVector);
         footTrajectory.appendWaypoint(timing.getTransferTime(), state.getFootPose(robotSide).getPosition(), zeroVector);
      }

      RobotSide swingSide = footstep.getRobotSide();
      RobotSide stanceSide = swingSide.getOppositeSide();

      footTrajectories.get(stanceSide)
                      .appendWaypoint(timing.getTransferTime() + timing.getSwingTime(), state.getFootPose(stanceSide).getPosition(), zeroVector);

      if (swingTrajectory == null)
      {
         predictSwingFootTrajectory(timing.getTransferTime(),
                                    timing.getSwingTime() + timing.getTransferTime(),
                                    swingHeight,
                                    state.getFootPose(swingSide).getPosition(),
                                    footstep.getFootstepPose().getPosition(),
                                    footTrajectories.get(swingSide));
      }
      else
      {
         setSwingFootTrajectory(swingTrajectory.getPositionTrajectory(), footTrajectories.get(swingSide));
      }

      leftFootTrajectory.initialize();
      rightFootTrajectory.initialize();
   }

   public MultipleWaypointsPositionTrajectoryGenerator getPredictedLeftFootTrajectories()
   {
      return leftFootTrajectory;
   }

   public MultipleWaypointsPositionTrajectoryGenerator getPredictedRightFootTrajectories()
   {
      return rightFootTrajectory;
   }

   private final FramePoint3D midpoint1 = new FramePoint3D();
   private final FramePoint3D midpoint2 = new FramePoint3D();

   private final FrameVector3D velocityVector = new FrameVector3D();

   void predictSwingFootTrajectory(double startTime,
                                   double endTime,
                                   double swingHeight,
                                   FramePoint3DReadOnly startPosition,
                                   FramePoint3DReadOnly endPosition,
                                   MultipleWaypointsPositionTrajectoryGenerator trajectoryToPack)
   {
      double time1 = InterpolationTools.linearInterpolate(startTime, endTime, interpolationFactor);
      midpoint1.interpolate(startPosition, endPosition, interpolationFactor);
      midpoint1.addZ(swingHeight);

      double time2 = InterpolationTools.linearInterpolate(endTime, startTime, interpolationFactor);
      midpoint2.interpolate(endPosition, startPosition, interpolationFactor);
      midpoint2.addZ(swingHeight);

      velocityVector.sub(midpoint2, midpoint1);
      velocityVector.scale(1.0 / (time2 - time1));

      trajectoryToPack.appendWaypoint(time1, midpoint1, velocityVector);
      trajectoryToPack.appendWaypoint(time2, midpoint2, velocityVector);
      trajectoryToPack.appendWaypoint(endTime, endPosition, zeroVector);
   }

   private static void setSwingFootTrajectory(MultipleWaypointsPositionTrajectoryGenerator swingTrajectory,
                                              MultipleWaypointsPositionTrajectoryGenerator trajectoriesToPack)
   {

      double timeShift = trajectoriesToPack.getLastWaypointTime() - swingTrajectory.getWaypoint(0).getTime();
      for (int waypointIdx = 1; waypointIdx < swingTrajectory.getCurrentNumberOfWaypoints(); waypointIdx++)
      {
         YoFrameEuclideanTrajectoryPoint waypoint = swingTrajectory.getWaypoint(waypointIdx);
         trajectoriesToPack.appendWaypoint(waypoint.getTime() + timeShift, waypoint.getPosition(), waypoint.getLinearVelocity());
      }
   }
}
