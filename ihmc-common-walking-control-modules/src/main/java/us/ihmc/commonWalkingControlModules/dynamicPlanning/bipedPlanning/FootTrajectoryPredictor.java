package us.ihmc.commonWalkingControlModules.dynamicPlanning.bipedPlanning;

import us.ihmc.commons.InterpolationTools;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePose3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.footstep.FootstepTiming;
import us.ihmc.robotics.math.trajectories.Trajectory3D;
import us.ihmc.robotics.math.trajectories.Trajectory3DReadOnly;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

import java.util.List;

public class FootTrajectoryPredictor
{
   private static final double swingHeight = 0.15;
   private static final double interpolationFactor = 0.25;

   private final RecyclingArrayList<Trajectory3D> leftFootTrajectories = new RecyclingArrayList<>(() -> new Trajectory3D(4));
   private final RecyclingArrayList<Trajectory3D> rightFootTrajectories = new RecyclingArrayList<>(() -> new Trajectory3D(4));

   private final SideDependentList<RecyclingArrayList<Trajectory3D>> footTrajectories = new SideDependentList<>(leftFootTrajectories, rightFootTrajectories);

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
         footTrajectories.get(robotSide).clear();
         setFootTrajectoryInContact(0.0, Double.POSITIVE_INFINITY, state.getFootPose(robotSide).getPosition(), footTrajectories.get(robotSide).add());
      }
   }

   private void computeWalking(CoPTrajectoryGeneratorState state)
   {
      PlanningTiming timing = state.getTiming(0);
      DynamicPlanningFootstep footstep = state.getFootstep(0);

      for (RobotSide robotSide : RobotSide.values)
      {
         footTrajectories.get(robotSide).clear();
         setFootTrajectoryInContact(0.0, timing.getTransferTime(), state.getFootPose(robotSide).getPosition(), footTrajectories.get(robotSide).add());
      }

      RobotSide swingSide = footstep.getRobotSide();
      RobotSide stanceSide = swingSide.getOppositeSide();

      setFootTrajectoryInContact(0.0,
                                 timing.getSwingTime(),
                                 state.getFootPose(stanceSide).getPosition(),
                                 footTrajectories.get(stanceSide).add());

      predictSwingFootTrajectory(0.0,
                                 timing.getSwingTime(),
                                 swingHeight,
                                 state.getFootPose(swingSide).getPosition(),
                                 footstep.getFootstepPose().getPosition(),
                                 footTrajectories.get(swingSide).add());
   }

   public List<? extends Trajectory3DReadOnly> getPredictedLeftFootTrajectories()
   {
      return leftFootTrajectories;
   }

   public List<? extends Trajectory3DReadOnly> getPredictedRightFootTrajectories()
   {
      return rightFootTrajectories;
   }

   static void setFootTrajectoryInContact(double startTime, double endTime, FramePoint3DReadOnly position, Trajectory3D trajectoryToPack)
   {
      trajectoryToPack.setConstant(startTime, endTime, position);
   }

   private final FramePoint3D midpoint1 = new FramePoint3D();
   private final FramePoint3D midpoint2 = new FramePoint3D();

   void predictSwingFootTrajectory(double startTime,
                                          double endTime,
                                          double swingHeight,
                                          FramePoint3DReadOnly startPosition,
                                          FramePoint3DReadOnly endPosition,
                                          Trajectory3D trajectoryToPack)
   {
      double time1 = InterpolationTools.linearInterpolate(startTime, endTime, interpolationFactor);
      midpoint1.interpolate(startPosition, endPosition, interpolationFactor);
      midpoint1.addZ(swingHeight);

      double time2 = InterpolationTools.linearInterpolate(endTime, startTime, interpolationFactor);
      midpoint2.interpolate(endPosition, startPosition, interpolationFactor);
      midpoint2.addZ(swingHeight);

      trajectoryToPack.setInterval(startTime, endTime);
      trajectoryToPack.setCubicUsingIntermediatePoints(startTime, time1, time2, endTime, startPosition, midpoint1, midpoint2, endPosition);
   }
}
