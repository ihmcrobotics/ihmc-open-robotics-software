package us.ihmc.commonWalkingControlModules.dynamicPlanning.bipedPlanning;

import us.ihmc.commons.InterpolationTools;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
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

   private final RecyclingArrayList<FixedFramePolynomial3DBasics> leftFootTrajectoryPool = new RecyclingArrayList<>(() -> new FramePolynomial3D(4,
                                                                                                                                                ReferenceFrame.getWorldFrame()));
   private final RecyclingArrayList<FixedFramePolynomial3DBasics> rightFootTrajectoryPool = new RecyclingArrayList<>(() -> new FramePolynomial3D(4,
                                                                                                                                                 ReferenceFrame.getWorldFrame()));

   private final SideDependentList<RecyclingArrayList<FixedFramePolynomial3DBasics>> footTrajectoryPool = new SideDependentList<>(leftFootTrajectoryPool,
                                                                                                                                  rightFootTrajectoryPool);

   private final MultipleSegmentPositionTrajectoryGenerator<FixedFramePolynomial3DBasics> leftFootTrajectory;
   private final MultipleSegmentPositionTrajectoryGenerator<FixedFramePolynomial3DBasics> rightFootTrajectory;
   private final SideDependentList<MultipleSegmentPositionTrajectoryGenerator<FixedFramePolynomial3DBasics>> footTrajectories;

   private MultipleWaypointsPoseTrajectoryGenerator swingTrajectory;

   public FootTrajectoryPredictor(YoRegistry parentRegistry)
   {
      YoRegistry registry = new YoRegistry(getClass().getSimpleName());

      leftFootTrajectory = new MultipleSegmentPositionTrajectoryGenerator<>("leftFootPredictedTrajectory", ReferenceFrame.getWorldFrame(), registry);
      rightFootTrajectory = new MultipleSegmentPositionTrajectoryGenerator<>("rightFootPredictedTrajectory", ReferenceFrame.getWorldFrame(), registry);

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

   private void computeStanding(CoPTrajectoryGeneratorState state)
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         footTrajectoryPool.get(robotSide).clear();
         footTrajectoryPool.get(robotSide).clear();
         footTrajectories.get(robotSide)
                         .appendSegment(setFootTrajectoryInContact(0.0,
                                                                   Double.POSITIVE_INFINITY,
                                                                   state.getFootPose(robotSide).getPosition(),
                                                                   footTrajectoryPool.get(robotSide).add()));
      }
   }

   private void computeWalking(CoPTrajectoryGeneratorState state)
   {
      PlanningTiming timing = state.getTiming(0);
      DynamicPlanningFootstep footstep = state.getFootstep(0);

      for (RobotSide robotSide : RobotSide.values)
      {
         footTrajectories.get(robotSide).clear();
         footTrajectoryPool.get(robotSide).clear();
         footTrajectories.get(robotSide)
                         .appendSegment(setFootTrajectoryInContact(0.0,
                                                                   timing.getTransferTime(),
                                                                   state.getFootPose(robotSide).getPosition(),
                                                                   footTrajectoryPool.get(robotSide).add()));
      }

      RobotSide swingSide = footstep.getRobotSide();
      RobotSide stanceSide = swingSide.getOppositeSide();

      setFootTrajectoryInContact(0.0, timing.getSwingTime(), state.getFootPose(stanceSide).getPosition(), footTrajectoryPool.get(stanceSide).add());

      if (swingTrajectory == null)
      {
         footTrajectories.get(swingSide)
                         .appendSegment(predictSwingFootTrajectory(0.0,
                                                                   timing.getSwingTime(),
                                                                   swingHeight,
                                                                   state.getFootPose(swingSide).getPosition(),
                                                                   footstep.getFootstepPose().getPosition(),
                                                                   footTrajectoryPool.get(swingSide).add()));
      }
      else
      {
         footTrajectories.get(swingSide).appendSegments(setSwingFootTrajectory(swingTrajectory.getPositionTrajectory(), footTrajectoryPool.get(swingSide)));
      }
   }

   public MultipleSegmentPositionTrajectoryGenerator<FixedFramePolynomial3DBasics> getPredictedLeftFootTrajectories()
   {
      return leftFootTrajectory;
   }

   public MultipleSegmentPositionTrajectoryGenerator<FixedFramePolynomial3DBasics> getPredictedRightFootTrajectories()
   {
      return rightFootTrajectory;
   }

   static FixedFramePolynomial3DBasics setFootTrajectoryInContact(double startTime,
                                                                  double endTime,
                                                                  FramePoint3DReadOnly position,
                                                                  FixedFramePolynomial3DBasics trajectoryToPack)
   {
      trajectoryToPack.setConstant(startTime, endTime, position);
      return trajectoryToPack;
   }

   private final FramePoint3D midpoint1 = new FramePoint3D();
   private final FramePoint3D midpoint2 = new FramePoint3D();

   FixedFramePolynomial3DBasics predictSwingFootTrajectory(double startTime,
                                                           double endTime,
                                                           double swingHeight,
                                                           FramePoint3DReadOnly startPosition,
                                                           FramePoint3DReadOnly endPosition,
                                                           FixedFramePolynomial3DBasics trajectoryToPack)
   {
      double time1 = InterpolationTools.linearInterpolate(startTime, endTime, interpolationFactor);
      midpoint1.interpolate(startPosition, endPosition, interpolationFactor);
      midpoint1.addZ(swingHeight);

      double time2 = InterpolationTools.linearInterpolate(endTime, startTime, interpolationFactor);
      midpoint2.interpolate(endPosition, startPosition, interpolationFactor);
      midpoint2.addZ(swingHeight);

      trajectoryToPack.setCubicUsingIntermediatePoints(startTime, time1, time2, endTime, startPosition, midpoint1, midpoint2, endPosition);
      return trajectoryToPack;
   }

   // FIXME generates garbage
   private static List<FixedFramePolynomial3DBasics> setSwingFootTrajectory(MultipleWaypointsPositionTrajectoryGenerator trajectory,
                                                                            RecyclingArrayList<FixedFramePolynomial3DBasics> trajectoriesToPack)
   {
      List<FixedFramePolynomial3DBasics> trajectories = new ArrayList<>();
      for (int waypointIdx = 0; waypointIdx < trajectory.getCurrentNumberOfWaypoints() - 1; waypointIdx++)
      {
         YoFrameEuclideanTrajectoryPoint start = trajectory.getWaypoint(waypointIdx);
         YoFrameEuclideanTrajectoryPoint end = trajectory.getWaypoint(waypointIdx + 1);

         trajectoriesToPack.add()
                           .setCubic(0.0,
                                     end.getTime() - start.getTime(),
                                     start.getPosition(),
                                     start.getLinearVelocity(),
                                     end.getPosition(),
                                     end.getLinearVelocity());
      }

      return trajectories;
   }
}
