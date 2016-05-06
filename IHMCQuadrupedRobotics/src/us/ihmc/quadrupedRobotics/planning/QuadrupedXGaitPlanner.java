package us.ihmc.quadrupedRobotics.planning;

import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;

import javax.vecmath.Vector3d;
import java.util.ArrayList;

public class QuadrupedXGaitPlanner
{
   private final FramePoint goalPosition;
   private final QuadrantDependentList<FramePoint> xGaitRectangle;
   private final FramePose xGaitRectanglePose;
   private final FramePose xGaitRectanglePoseAtSoS;
   private final PoseReferenceFrame xGaitRectangleFrame;

   public QuadrupedXGaitPlanner()
   {
      goalPosition = new FramePoint();
      xGaitRectangle = new QuadrantDependentList<>();
      xGaitRectanglePose = new FramePose(ReferenceFrame.getWorldFrame());
      xGaitRectanglePoseAtSoS = new FramePose(ReferenceFrame.getWorldFrame());
      xGaitRectangleFrame = new PoseReferenceFrame("xGaitRectangleFrame", ReferenceFrame.getWorldFrame());
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         xGaitRectangle.set(robotQuadrant, new FramePoint(xGaitRectangleFrame));
      }
   }

   public void computeInitialPlan(ArrayList<QuadrupedTimedStep> plannedSteps, Vector3d planarVelocity, RobotQuadrant initialStepQuadrant, FramePoint supportCentroidAtSoS, double timeAtSoS, double yawAtSoS, QuadrupedXGaitSettings xGaitSettings)
   {
      // initialize nominal support rectangle
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         xGaitRectangle.get(robotQuadrant).changeFrame(xGaitRectangleFrame);
         xGaitRectangle.get(robotQuadrant).setX(robotQuadrant.getEnd().negateIfHindEnd(xGaitSettings.getStanceLength() / 2.0));
         xGaitRectangle.get(robotQuadrant).setY(robotQuadrant.getSide().negateIfRightSide(xGaitSettings.getStanceWidth() / 2.0));
         xGaitRectangle.get(robotQuadrant).setZ(0);
      }
      ReferenceFrame supportCentroidFrame = supportCentroidAtSoS.getReferenceFrame();
      supportCentroidAtSoS.changeFrame(ReferenceFrame.getWorldFrame());
      supportCentroidAtSoS.changeFrame(supportCentroidFrame);
      xGaitRectanglePoseAtSoS.setPosition(supportCentroidAtSoS);
      xGaitRectanglePoseAtSoS.setYawPitchRoll(yawAtSoS, 0, 0);

      // plan steps
      double lastStepStartTime = timeAtSoS;
      RobotQuadrant lastStepQuadrant = initialStepQuadrant.getNextReversedRegularGaitSwingQuadrant();

      for (int i = 0; i < plannedSteps.size(); i++)
      {
         QuadrupedTimedStep step = plannedSteps.get(i);

         // compute step quadrant
         RobotQuadrant thisStepQuadrant = lastStepQuadrant.getNextRegularGaitSwingQuadrant();
         step.setRobotQuadrant(thisStepQuadrant);

         // compute step timing
         double thisStepStartTime;
         if (i == 0)
            thisStepStartTime = timeAtSoS;
         else
            thisStepStartTime = computeStepStartTime(thisStepQuadrant, lastStepStartTime, xGaitSettings);
         double thisStepEndTime = computeStepEndTime(thisStepQuadrant, thisStepStartTime, lastStepStartTime, xGaitSettings);
         step.getTimeInterval().setStartTime(thisStepStartTime);
         step.getTimeInterval().setEndTime(thisStepEndTime);

         // compute xGait rectangle pose at end of step
         double deltaTime = thisStepEndTime - timeAtSoS;
         extrapolatePose(xGaitRectanglePose, xGaitRectanglePoseAtSoS, planarVelocity, deltaTime);
         xGaitRectangleFrame.setPoseAndUpdate(xGaitRectanglePose);

         // compute step goal position by sampling the corner position of the xGait rectangle at touch down
         RobotQuadrant robotQuadrant = step.getRobotQuadrant();
         goalPosition.setIncludingFrame(xGaitRectangle.get(robotQuadrant));
         step.setGoalPosition(goalPosition);

         // compute step ground clearance
         step.setGroundClearance(xGaitSettings.getStepGroundClearance());

         // update book-keeping
         lastStepStartTime = thisStepStartTime;
         lastStepQuadrant = thisStepQuadrant;
      }
   }

   public void computeMidStepPlan(ArrayList<QuadrupedTimedStep> plannedSteps, QuadrupedTimedStep ongoingStep, Vector3d planarVelocity, double timeAtSoS, double yawAtSoS, QuadrupedXGaitSettings xGaitSettings)
   {
      if (timeAtSoS < ongoingStep.getTimeInterval().getStartTime())
      {
         throw new RuntimeException("Time at start of next step must be greater than the last step");
      }

      // initialize nominal support rectangle
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         xGaitRectangle.get(robotQuadrant).changeFrame(xGaitRectangleFrame);
         xGaitRectangle.get(robotQuadrant).setX(robotQuadrant.getEnd().negateIfHindEnd(xGaitSettings.getStanceLength() / 2.0));
         xGaitRectangle.get(robotQuadrant).setY(robotQuadrant.getSide().negateIfRightSide(xGaitSettings.getStanceWidth() / 2.0));
         xGaitRectangle.get(robotQuadrant).setZ(0);
      }
      xGaitRectanglePoseAtSoS.setPosition(0, 0, 0);
      xGaitRectanglePoseAtSoS.setYawPitchRoll(yawAtSoS, 0, 0);

      // compute step quadrants and time intervals
      {
         double thisStepStartTime = timeAtSoS;
         double lastStepEndTime = ongoingStep.getTimeInterval().getEndTime();
         RobotQuadrant thisStepQuadrant = ongoingStep.getRobotQuadrant().getNextRegularGaitSwingQuadrant();

         for (int i = 0; i < plannedSteps.size(); i++)
         {
            // compute step quadrant
            plannedSteps.get(i).setRobotQuadrant(thisStepQuadrant);
            thisStepQuadrant = thisStepQuadrant.getNextRegularGaitSwingQuadrant();

            // compute step timing
            double thisStepEndTime = computeStepEndTime(thisStepQuadrant, thisStepStartTime, lastStepEndTime, xGaitSettings);
            plannedSteps.get(i).getTimeInterval().setStartTime(thisStepStartTime);
            plannedSteps.get(i).getTimeInterval().setEndTime(thisStepEndTime);
            thisStepStartTime = lastStepEndTime + xGaitSettings.getEndDoubleSupportDuration();
            lastStepEndTime = thisStepEndTime;
         }
      }

      // compute step goal positions and ground clearances
      {
         for (int i = 0; i < plannedSteps.size(); i++)
         {
            // compute xGait rectangle pose at end of step
            double deltaTime = plannedSteps.get(i).getTimeInterval().getEndTime() - timeAtSoS;
            extrapolatePose(xGaitRectanglePose, xGaitRectanglePoseAtSoS, planarVelocity, deltaTime);
            xGaitRectangleFrame.setPoseAndUpdate(xGaitRectanglePose);

            // compute step goal position by sampling the corner position of the xGait rectangle at touchdown
            RobotQuadrant stepQuadrant = plannedSteps.get(i).getRobotQuadrant();
            goalPosition.setIncludingFrame(xGaitRectangle.get(stepQuadrant));
            plannedSteps.get(i).setGoalPosition(goalPosition);

            // compute step ground clearance
            plannedSteps.get(i).setGroundClearance(xGaitSettings.getStepGroundClearance());
         }
      }

      // translate step goal positions to match ongoing step constraint
      {
         // compute xGait rectangle pose at end of step
         double deltaTime = ongoingStep.getTimeInterval().getEndTime() - timeAtSoS;
         extrapolatePose(xGaitRectanglePose, xGaitRectanglePoseAtSoS, planarVelocity, deltaTime);
         xGaitRectangleFrame.setPoseAndUpdate(xGaitRectanglePose);

         // compute step goal position
         RobotQuadrant stepQuadrant = ongoingStep.getRobotQuadrant();
         goalPosition.setIncludingFrame(xGaitRectangle.get(stepQuadrant));
         goalPosition.changeFrame(ReferenceFrame.getWorldFrame());

         // compensate for position error
         for (int i = 0; i < plannedSteps.size(); i++)
         {
            plannedSteps.get(i).getGoalPosition().add(ongoingStep.getGoalPosition());
            plannedSteps.get(i).getGoalPosition().sub(goalPosition.getPoint());
         }
      }
   }

   private void extrapolatePose(FramePose finalPose, FramePose initialPose, Vector3d planarVelocity, double deltaTime)
   {
      // initialize planar position and orientation
      double a0 = initialPose.getYaw();
      double x0 = initialPose.getX();
      double y0 = initialPose.getY();

      // initialize forward, lateral, and rotational velocity in pose frame
      double u = planarVelocity.getX();
      double v = planarVelocity.getY();
      double phi = planarVelocity.getZ();

      // compute extrapolated pose assuming a constant planar velocity
      double a, x, y;
      double epsilon = 0.001;
      if (Math.abs(phi) > epsilon)
      {
         a = a0 + phi * deltaTime;
         x = x0 + u / phi * (Math.sin(a) - Math.sin(a0)) + v / phi * (Math.cos(a) - Math.cos(a0));
         y = y0 - u / phi * (Math.cos(a) - Math.cos(a0)) + v / phi * (Math.sin(a) - Math.sin(a0));
      }
      else
      {
         a = a0;
         x = x0 + (u * Math.cos(a) - v * Math.sin(a)) * deltaTime;
         y = y0 + (u * Math.sin(a) + v * Math.cos(a)) * deltaTime;
      }

      finalPose.setX(x);
      finalPose.setY(y);
      finalPose.setZ(initialPose.getZ());
      finalPose.setYawPitchRoll(a, initialPose.getPitch(), initialPose.getRoll());
   }

   private double computeStepEndTime(RobotQuadrant stepQuadrant, double stepStartTime, double lastStepEndTime, QuadrupedXGaitSettings xGaitSettings)
   {
      // compute support durations and end phase shift
      double stepDuration = xGaitSettings.getStepDuration();
      double endDoubleSupportDuration = xGaitSettings.getEndDoubleSupportDuration();
      double endPhaseShift = stepQuadrant.isQuadrantInHind() ? 180.0 - xGaitSettings.getEndPhaseShift() : xGaitSettings.getEndPhaseShift();

      // compute step end time assuming the nominal duration can only be scaled in the range (1.0, 1.5)
      double currentStepEndTime = lastStepEndTime + Math.max((stepDuration + endDoubleSupportDuration) * endPhaseShift / 180.0, 0.0);
      double currentStepDuration = MathTools.clipToMinMax(currentStepEndTime - stepStartTime, stepDuration, 1.5 * stepDuration);
      return stepStartTime + currentStepDuration;
   }

   private double computeStepStartTime(RobotQuadrant stepQuadrant, double lastStepStartTime, QuadrupedXGaitSettings xGaitSettings)
   {
      double stepDuration = xGaitSettings.getStepDuration();
      double endDoubleSupportDuration = xGaitSettings.getEndDoubleSupportDuration();
      double endPhaseShift = stepQuadrant.isQuadrantInHind() ? 180.0 - xGaitSettings.getEndPhaseShift() : xGaitSettings.getEndPhaseShift();
      return lastStepStartTime + (endDoubleSupportDuration + stepDuration) * Math.max(Math.min(endPhaseShift, 180.0), 0.0) / 180.0;
   }
}