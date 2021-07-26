package us.ihmc.commonWalkingControlModules.dynamicPlanning.bipedPlanning;

import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.jumpingController.JumpingCoPTrajectoryGeneratorState;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.jumpingController.JumpingGoalVariable;
import us.ihmc.commons.InterpolationTools;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.robotics.math.trajectories.generators.MultipleWaypointsPoseTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.generators.MultipleWaypointsPositionTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.trajectorypoints.SE3TrajectoryPoint;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.referenceFrames.ZUpFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.List;

import static us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.CoMTrajectoryPlannerTools.sufficientlyLongTime;

public class FootTrajectoryPredictor
{
   private static final double defaultSwingHeight = 0.15;
   private static final double defaultPredictorWaypointProportion = 0.25;

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final MultipleWaypointsPositionTrajectoryGenerator leftFootTrajectory = new MultipleWaypointsPositionTrajectoryGenerator("leftFootPredictedTrajectory", ReferenceFrame.getWorldFrame(), registry);
   private final MultipleWaypointsPositionTrajectoryGenerator rightFootTrajectory = new MultipleWaypointsPositionTrajectoryGenerator("rightFootPredictedTrajectory", ReferenceFrame.getWorldFrame(), registry);
   private final SideDependentList<MultipleWaypointsPositionTrajectoryGenerator> footTrajectories = new SideDependentList<>(leftFootTrajectory, rightFootTrajectory);

   private final DoubleProvider predictorSwingHeight = new DoubleParameter("predictorSwingHeight", registry, defaultSwingHeight);
   private final DoubleProvider predictorWaypointProportion = new DoubleParameter("predictorWaypointProportion", registry, defaultPredictorWaypointProportion);

   private final SideDependentList<RecyclingArrayList<SE3TrajectoryPoint>> swingWaypoints = new SideDependentList<>(new RecyclingArrayList<>(SE3TrajectoryPoint::new),
                                                                                                                    new RecyclingArrayList<>(SE3TrajectoryPoint::new));

   private final FramePose3D midFootPosition = new FramePose3D();
   private final FramePose3D midstancePose = new FramePose3D();
   private final PoseReferenceFrame midstanceFrame = new PoseReferenceFrame("midstanceFrame", ReferenceFrame.getWorldFrame());
   private final ZUpFrame midstanceZUpFrame = new ZUpFrame(ReferenceFrame.getWorldFrame(), midstanceFrame, "midstanceZUpFrame");
   private final FramePose3D goalPose = new FramePose3D();
   private final PoseReferenceFrame goalPoseFrame = new PoseReferenceFrame("goalPoseFrame", ReferenceFrame.getWorldFrame());
   private final FramePoint3D footGoalPosition = new FramePoint3D();

   public FootTrajectoryPredictor(YoRegistry parentRegistry)
   {
      parentRegistry.addChild(registry);
   }

   public void setSwingTrajectory(RobotSide swingSide, MultipleWaypointsPoseTrajectoryGenerator swingTrajectory)
   {
      if (swingTrajectory == null)
         return;

      RecyclingArrayList<SE3TrajectoryPoint> swingWaypointsToUse = swingWaypoints.get(swingSide);

      swingWaypointsToUse.clear();
      for (int i = 0; i < swingTrajectory.getPositionTrajectory().getCurrentNumberOfWaypoints(); i++)
         swingWaypointsToUse.add().set(swingTrajectory.getPositionTrajectory().getWaypoint(i));
   }

   public void clearSwingTrajectory()
   {
      for (RobotSide robotSide : RobotSide.values)
         swingWaypoints.get(robotSide).clear();
   }

   public void compute(CoPTrajectoryGeneratorState state)
   {
      if (state.getNumberOfFootstep() > 0)
         computeWalking(state);
      else
         computeStanding(state);
   }

   public void compute(JumpingCoPTrajectoryGeneratorState state)
   {
      if (Double.isNaN(state.getJumpingGoal().getGoalLength()))
         computeJumpingStanding(state);
      else
         computeJumping(state);
   }

   private final FrameVector3DReadOnly zeroVector = new FrameVector3D();

   private void computeStanding(CoPTrajectoryGeneratorState state)
   {
      clearSwingTrajectory();

      for (RobotSide robotSide : RobotSide.values)
      {
         MultipleWaypointsPositionTrajectoryGenerator footTrajectory = footTrajectories.get(robotSide);
         footTrajectory.clear();
         footTrajectory.appendWaypoint(0.0, state.getFootPose(robotSide).getPosition(), zeroVector);
         footTrajectory.appendWaypoint(sufficientlyLongTime, state.getFootPose(robotSide).getPosition(), zeroVector);
         footTrajectory.initialize();
      }
   }

   private void computeJumpingStanding(JumpingCoPTrajectoryGeneratorState state)
   {
      clearSwingTrajectory();

      for (RobotSide robotSide : RobotSide.values)
      {
         MultipleWaypointsPositionTrajectoryGenerator footTrajectory = footTrajectories.get(robotSide);
         footTrajectory.clear();
         footTrajectory.appendWaypoint(0.0, state.getFootPose(robotSide).getPosition(), zeroVector);
         footTrajectory.appendWaypoint(sufficientlyLongTime, state.getFootPose(robotSide).getPosition(), zeroVector);
         footTrajectory.initialize();
      }
   }

   private void computeWalking(CoPTrajectoryGeneratorState state)
   {
      PlanningTiming timing = state.getTiming(0);
      DynamicPlanningFootstep footstep = state.getFootstep(0);

      double transferDuration = Math.min(timing.getTransferTime(), sufficientlyLongTime);
      double swingDuration = Math.min(timing.getSwingTime(), sufficientlyLongTime);

      for (RobotSide robotSide : RobotSide.values)
      {
         MultipleWaypointsPositionTrajectoryGenerator footTrajectory = footTrajectories.get(robotSide);
         footTrajectory.clear();
         footTrajectory.appendWaypoint(0.0, state.getFootPose(robotSide).getPosition(), zeroVector);
         footTrajectory.appendWaypoint(transferDuration, state.getFootPose(robotSide).getPosition(), zeroVector);
      }

      RobotSide swingSide = footstep.getRobotSide();
      RobotSide stanceSide = swingSide.getOppositeSide();
      RecyclingArrayList<SE3TrajectoryPoint> swingWaypointsToUse = swingWaypoints.get(stanceSide.getOppositeSide());


      footTrajectories.get(stanceSide)
                      .appendWaypoint(transferDuration + swingDuration, state.getFootPose(stanceSide).getPosition(), zeroVector);


      if (swingWaypointsToUse.isEmpty())
      {
         predictSwingFootTrajectory(transferDuration,
                                    transferDuration + swingDuration,
                                    predictorSwingHeight.getValue(),
                                    state.getFootPose(swingSide).getPosition(),
                                    footstep.getFootstepPose().getPosition(),
                                    footTrajectories.get(swingSide));
      }
      else
      {
         setSwingFootTrajectory(swingWaypointsToUse, footTrajectories.get(swingSide));
      }

      leftFootTrajectory.initialize();
      rightFootTrajectory.initialize();
   }

   private void computeJumping(JumpingCoPTrajectoryGeneratorState state)
   {
      JumpingGoalVariable jumpingGoal = state.getJumpingGoal();

      double supportDuration = Math.min(jumpingGoal.getSupportDuration(), sufficientlyLongTime);
      double flightDuration = Math.min(jumpingGoal.getFlightDuration(), sufficientlyLongTime);

      for (RobotSide robotSide : RobotSide.values)
      {
         MultipleWaypointsPositionTrajectoryGenerator footTrajectory = footTrajectories.get(robotSide);
         footTrajectory.clear();
         footTrajectory.appendWaypoint(0.0, state.getFootPose(robotSide).getPosition(), zeroVector);
         footTrajectory.appendWaypoint(supportDuration, state.getFootPose(robotSide).getPosition(), zeroVector);
      }

      midstancePose.interpolate(state.getFootPose(RobotSide.LEFT), state.getFootPose(RobotSide.RIGHT), 0.5);
      midstanceFrame.setPoseAndUpdate(midstancePose);
      midstanceZUpFrame.update();

      midFootPosition.setToZero(midstanceZUpFrame);
      goalPose.setIncludingFrame(midFootPosition);
      goalPose.setX(jumpingGoal.getGoalLength());
      if (!Double.isNaN(state.getJumpingGoal().getGoalHeight()))
         goalPose.setZ(state.getJumpingGoal().getGoalHeight());
      if (!Double.isNaN(state.getJumpingGoal().getGoalRotation()))
         goalPose.getOrientation().setToYawOrientation(state.getJumpingGoal().getGoalRotation());
      goalPose.changeFrame(ReferenceFrame.getWorldFrame());
      goalPoseFrame.setPoseAndUpdate(goalPose);

      for (RobotSide robotSide : RobotSide.values)
      {
         RecyclingArrayList<SE3TrajectoryPoint> swingWaypointsToUse = swingWaypoints.get(robotSide);

         if (swingWaypointsToUse.isEmpty())
         {
            footGoalPosition.setToZero(goalPoseFrame);
            double width;
            if (!Double.isNaN(state.getJumpingGoal().getGoalFootWidth()))
               width = 0.5 * state.getJumpingGoal().getGoalFootWidth();
            else
               width = 0.5 * 0.3;//regularParameters.getDefaultFootWidth();
            width = robotSide.negateIfRightSide(width);

            footGoalPosition.setY(width);
            footGoalPosition.changeFrame(ReferenceFrame.getWorldFrame());

            predictSwingFootTrajectory(supportDuration,
                                       supportDuration + flightDuration,
                                       predictorSwingHeight.getValue(),
                                       state.getFootPose(robotSide).getPosition(),
                                       footGoalPosition,
                                       footTrajectories.get(robotSide));
         }
         else
         {
            setSwingFootTrajectory(swingWaypointsToUse, footTrajectories.get(robotSide));
         }
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

   private final FrameVector3D velocityVector1 = new FrameVector3D();
   private final FrameVector3D velocityVector2 = new FrameVector3D();

   void predictSwingFootTrajectory(double startTime,
                                   double endTime,
                                   double swingHeight,
                                   FramePoint3DReadOnly startPosition,
                                   FramePoint3DReadOnly endPosition,
                                   MultipleWaypointsPositionTrajectoryGenerator trajectoryToPack)
   {
      double time1 = InterpolationTools.linearInterpolate(startTime, endTime, predictorWaypointProportion.getValue());
      midpoint1.interpolate(startPosition, endPosition, predictorWaypointProportion.getValue());
      midpoint1.addZ(swingHeight);

      double time2 = InterpolationTools.linearInterpolate(endTime, startTime, predictorWaypointProportion.getValue());
      midpoint2.interpolate(endPosition, startPosition, predictorWaypointProportion.getValue());
      midpoint2.addZ(swingHeight);

      velocityVector1.sub(midpoint2, startPosition);
      velocityVector1.scale(1.0 / (time2 - startTime));

      velocityVector2.sub(endPosition, midpoint1);
      velocityVector2.scale(1.0 / (endTime - time1));

      trajectoryToPack.appendWaypoint(time1, midpoint1, velocityVector1);
      trajectoryToPack.appendWaypoint(time2, midpoint2, velocityVector2);
      trajectoryToPack.appendWaypoint(endTime, endPosition, zeroVector);
   }

   private static void setSwingFootTrajectory(List<SE3TrajectoryPoint> swingWaypoints,
                                              MultipleWaypointsPositionTrajectoryGenerator trajectoriesToPack)
   {

      double timeShift = trajectoriesToPack.getLastWaypointTime() - swingWaypoints.get(0).getTime();
      for (int waypointIdx = 0; waypointIdx < swingWaypoints.size(); waypointIdx++)
      {
         SE3TrajectoryPoint waypoint = swingWaypoints.get(waypointIdx);
         trajectoriesToPack.appendWaypoint(waypoint.getTime() + timeShift, waypoint.getPosition(), waypoint.getLinearVelocity());
      }
   }
}
