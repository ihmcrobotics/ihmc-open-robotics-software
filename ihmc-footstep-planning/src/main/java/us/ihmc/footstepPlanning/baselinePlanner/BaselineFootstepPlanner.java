package us.ihmc.footstepPlanning.baselinePlanner;

import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.robotics.math.trajectories.interfaces.PoseTrajectoryGenerator;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.robotSide.*;

import java.util.List;

public class BaselineFootstepPlanner
{
   private final double PLANNER_TIME_RESOLUTION = 0.01;
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final BaselineFootstepPlannerParameters parameters;
   private final FramePose3D lastStepBaselinePose;
   private final FramePose3D midpointBaselinePose;
   private final FramePose3D thisStepBaselinePose;
   private final PoseReferenceFrame midpointBaselineFrame;
   private final FrameVector3D midpointBaselineLinearVelocity;
   private final FrameVector3D midpointBaselineAngularVelocity;
   private final SideDependentList<FramePose3D> stancePose;
   private final SideDependentList<FramePose3D> previousFootholds;
   private final FramePose3D stepGoalPose;

   public BaselineFootstepPlanner(BaselineFootstepPlannerParameters parameters)
   {
      this.parameters = parameters;
      this.lastStepBaselinePose = new FramePose3D(worldFrame);
      this.midpointBaselinePose = new FramePose3D(worldFrame);
      this.thisStepBaselinePose = new FramePose3D(worldFrame);
      this.midpointBaselineFrame = new PoseReferenceFrame("midpointBaselineFrame", worldFrame);
      this.midpointBaselineLinearVelocity = new FrameVector3D(worldFrame);
      this.midpointBaselineAngularVelocity = new FrameVector3D(worldFrame);
      this.stancePose = new SideDependentList<>();
      for (RobotSide robotSide : RobotSide.values)
      {
         this.stancePose.put(robotSide, new FramePose3D(worldFrame));
      }
      this.previousFootholds = new SideDependentList<>();
      for (RobotSide robotSide : RobotSide.values)
      {
         this.previousFootholds.put(robotSide, new FramePose3D(worldFrame));
      }
      this.stepGoalPose = new FramePose3D(worldFrame);
      computeStancePose();
   }

   public int compute(List<SimpleTimedFootstep> footstepPlan, PoseTrajectoryGenerator baselinePoseTrajectory, SideDependentList<FramePose3D> initialFootholds,
                      RobotSide initialStepSide, double initialTime, double finalTime)
   {
      computeStancePose();

      // Initialize footholds.
      for (RobotSide robotSide : RobotSide.values)
      {
         previousFootholds.get(robotSide).setIncludingFrame(initialFootholds.get(robotSide));
      }

      // Initialize last step baseline pose.
      RobotSide lastStepSide = initialStepSide.getOppositeSide();
      computeBaselinePoseFromFootholdPose(lastStepBaselinePose, initialFootholds.get(lastStepSide), lastStepSide);

      int numSteps = 0;
      RobotSide thisStepSide = initialStepSide;
      double thisStepEndTime = initialTime + parameters.getMinimumTransferDuration() + parameters.getSwingDuration();
      while (thisStepEndTime < finalTime)
      {
         if (numSteps > footstepPlan.size() - 1)
         {
            break;
         }

         // Update midpoint baseline pose and twist.
         midpointBaselinePose.changeFrame(worldFrame);
         midpointBaselineLinearVelocity.changeFrame(worldFrame);
         midpointBaselineAngularVelocity.changeFrame(worldFrame);
         baselinePoseTrajectory.compute(thisStepEndTime);
         midpointBaselinePose.set(baselinePoseTrajectory.getPose());
         midpointBaselineLinearVelocity.set(baselinePoseTrajectory.getVelocity());
         midpointBaselineAngularVelocity.set(baselinePoseTrajectory.getAngularVelocity());

         // Update midpoint baseline frame.
         midpointBaselineFrame.setPoseAndUpdate(midpointBaselinePose);
         lastStepBaselinePose.changeFrame(midpointBaselineFrame);

         // Search for a future baseline pose that maximizes the stance symmetry at touchdown.
         double minimumSymmetryCost = Double.POSITIVE_INFINITY;
         double thisStepBaselineTime = thisStepEndTime;
         while (thisStepBaselineTime < finalTime)
         {
            baselinePoseTrajectory.compute(thisStepBaselineTime);
            thisStepBaselinePose.setIncludingFrame(worldFrame, baselinePoseTrajectory.getPose());
            thisStepBaselinePose.changeFrame(midpointBaselineFrame);
            double symmetryCost = computePoseSymmetryCost(lastStepBaselinePose, thisStepBaselinePose);
            if (symmetryCost < minimumSymmetryCost)
            {
               minimumSymmetryCost = symmetryCost;
               thisStepBaselineTime += PLANNER_TIME_RESOLUTION;
            }
            else
            {
               break;
            }
         }

         // Compute stride length and velocity in midpoint baseline frame.
         midpointBaselineLinearVelocity.changeFrame(midpointBaselineFrame);
         midpointBaselineAngularVelocity.changeFrame(midpointBaselineFrame);
         double forwardStride = thisStepBaselinePose.getX() - lastStepBaselinePose.getX();
         double lateralStride = thisStepBaselinePose.getY() - lastStepBaselinePose.getY();
         double turningStride = thisStepBaselinePose.getYaw() - lastStepBaselinePose.getYaw();
         double forwardVelocity = midpointBaselineLinearVelocity.getX();
         double lateralVelocity = midpointBaselineLinearVelocity.getY();
         double turningVelocity = midpointBaselineAngularVelocity.getZ();
         double angularVelocityMagnitude = Math.abs(turningVelocity);
         double linearVelocityMagnitude = EuclidCoreTools.norm(forwardVelocity, lateralVelocity);
         double angularVelocityEpsilon = 0.05;
         double linearVelocityEpsilon = 0.01;

         // Determine whether to take a step.
         boolean triggerStep = false;
         if (Math.abs(forwardStride) > parameters.getMinimumForwardStride())
         {
            triggerStep = true;
         }
         else if (Math.abs(lateralStride) > parameters.getMinimumLateralStride())
         {
            triggerStep = true;
         }
         else if (Math.abs(turningStride) > parameters.getMinimumTurningStride())
         {
            triggerStep = true;
         }
         else if (angularVelocityMagnitude < angularVelocityEpsilon && linearVelocityMagnitude < linearVelocityEpsilon)
         {
            double stancePositionError = 0.0;
            double stanceRotationError = 0.0;
            for (RobotSide robotSide : RobotSide.values)
            {
               FramePose3D stepPose = previousFootholds.get(robotSide);
               stepPose.changeFrame(worldFrame);
               stepGoalPose.setIncludingFrame(stancePose.get(robotSide));
               midpointBaselinePose.transform(stepGoalPose);
               double xError = stepPose.getX() - stepGoalPose.getX();
               double yError = stepPose.getY() - stepGoalPose.getY();
               stancePositionError += Math.sqrt(xError * xError + yError * yError) / 2.0;
               stanceRotationError += AngleTools.computeAngleDifferenceMinusPiToPi(stepPose.getYaw(), stepGoalPose.getYaw()) / 2.0;
            }

            // Align feet with nominal stance when velocity approaches zero.
            if (stancePositionError > parameters.getStancePositionAlignmentThreshold())
            {
               triggerStep = true;
            }
            if (stanceRotationError > parameters.getStanceRotationAlignmentThreshold())
            {
               triggerStep = true;
            }
         }

         if (triggerStep)
         {
            SimpleTimedFootstep step = footstepPlan.get(numSteps++);

            // Compute step time-interval and robot side.
            step.setRobotSide(thisStepSide);
            step.getTimeInterval().setInterval(thisStepEndTime - parameters.getSwingDuration(), thisStepEndTime);

            // Compute step pose relative to baseline (adjust y translation and yaw based on velocity to avoid self-collision).
            double lateralStanceAdjustment = thisStepSide.negateIfRightSide(parameters.getLateralVelocityStanceScaling() * Math.abs(lateralVelocity));
            double turningStanceAdjustment = thisStepSide.negateIfRightSide(parameters.getTurningVelocityStanceScaling() * Math.abs(turningVelocity));
            stepGoalPose.setIncludingFrame(stancePose.get(thisStepSide));
            stepGoalPose.setY(stepGoalPose.getY() + lateralStanceAdjustment);
            stepGoalPose.getRotation().setToYawOrientation(stepGoalPose.getYaw() + turningStanceAdjustment);

            // Transform step pose to world frame.
            thisStepBaselinePose.changeFrame(worldFrame);
            thisStepBaselinePose.transform(stepGoalPose);
            step.setSoleFramePose(stepGoalPose);

            // Update step parameters
            lastStepBaselinePose.setIncludingFrame(thisStepBaselinePose);
            lastStepBaselinePose.changeFrame(worldFrame);
            thisStepSide = thisStepSide.getOppositeSide();
            thisStepEndTime = thisStepEndTime + parameters.getMinimumTransferDuration() + parameters.getSwingDuration();
            thisStepEndTime = Math.max(thisStepEndTime, thisStepBaselineTime);
            previousFootholds.get(thisStepSide).setIncludingFrame(stepGoalPose);
            previousFootholds.get(thisStepSide).changeFrame(worldFrame);
         }
         else
         {
            lastStepBaselinePose.changeFrame(worldFrame);
            thisStepEndTime += PLANNER_TIME_RESOLUTION;
         }
      }

      return numSteps;
   }

   public void computeBaselinePoseFromFootholdPose(FramePose3D baselinePose, FramePose3D footholdPose, RobotSide robotSide)
   {
      baselinePose.setToZero(worldFrame);
      stancePose.get(robotSide).inverseTransform(baselinePose);
      footholdPose.changeFrame(worldFrame);
      footholdPose.transform(baselinePose);
   }

   private double computePoseSymmetryCost(FramePose3DReadOnly poseA, FramePose3DReadOnly poseB)
   {
      double xSum = poseA.getX() + poseB.getX();
      double ySum = poseA.getY() + poseB.getY();
      double aSum = poseA.getYaw() + poseB.getYaw();
      return parameters.getForwardSymmetryWeight() * xSum * xSum + parameters.getLateralSymmetryWeight() * ySum * ySum
             + parameters.getTurningSymmetryWeight() * aSum * aSum;
   }

   private void computeStancePose()
   {
      // Initialize default stance configuration.
      for (RobotSide robotSide : RobotSide.values)
      {
         double stanceY = robotSide.negateIfRightSide(parameters.getStanceWidth() / 2.0);
         double stanceYaw = robotSide.negateIfRightSide(parameters.getStanceSplay());
         stancePose.get(robotSide).getPosition().set(0, stanceY, 0);
         stancePose.get(robotSide).getOrientation().setToYawOrientation(stanceYaw);
      }
   }
}