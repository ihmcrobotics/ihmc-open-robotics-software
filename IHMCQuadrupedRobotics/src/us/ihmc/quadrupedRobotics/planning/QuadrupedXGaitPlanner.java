package us.ihmc.quadrupedRobotics.planning;

import us.ihmc.quadrupedRobotics.util.TimeInterval;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.*;

import javax.vecmath.Vector3d;
import java.util.ArrayList;

public class QuadrupedXGaitPlanner
{
   private final FramePoint goalPosition;
   private final QuadrantDependentList<FramePoint> xGaitRectangle;
   private final FramePose xGaitRectanglePose;
   private final FramePose xGaitRectanglePoseAtSoS;
   private final PoseReferenceFrame xGaitRectangleFrame;
   private final EndDependentList<QuadrupedTimedStep> pastSteps;

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
      pastSteps = new EndDependentList<>();
      pastSteps.put(RobotEnd.FRONT, new QuadrupedTimedStep());
      pastSteps.put(RobotEnd.HIND, new QuadrupedTimedStep());
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
      xGaitRectanglePoseAtSoS.setPosition(supportCentroidAtSoS);
      xGaitRectanglePoseAtSoS.setYawPitchRoll(yawAtSoS, 0, 0);
      supportCentroidAtSoS.changeFrame(supportCentroidFrame);

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
         double thisStepEndTime;
         if (i == 0)
         {
            thisStepStartTime = timeAtSoS;
            thisStepEndTime = timeAtSoS + xGaitSettings.getStepDuration();
         }
         else
         {
            double endPhaseShift = thisStepQuadrant.isQuadrantInHind() ? 180.0 - xGaitSettings.getEndPhaseShift() : xGaitSettings.getEndPhaseShift();
            double endTimeShift = (xGaitSettings.getEndDoubleSupportDuration() + xGaitSettings.getStepDuration()) * Math.max(Math.min(endPhaseShift, 180.0), 0.0) / 180.0;
            thisStepStartTime = lastStepStartTime + endTimeShift;
            thisStepEndTime = thisStepStartTime + xGaitSettings.getStepDuration();
         }
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

         // update state for next step
         lastStepStartTime = thisStepStartTime;
         lastStepQuadrant = thisStepQuadrant;
      }
   }
   public void computeOnlinePlan(ArrayList<QuadrupedTimedStep> plannedSteps, EndDependentList<QuadrupedTimedStep> latestSteps, Vector3d planarVelocity, double currentTime, double currentYaw, QuadrupedXGaitSettings xGaitSettings)
   {
      // initialize latest step
      QuadrupedTimedStep latestStep;
      if (latestSteps.get(RobotEnd.HIND).getTimeInterval().getEndTime() > latestSteps.get(RobotEnd.FRONT).getTimeInterval().getEndTime())
         latestStep = latestSteps.get(RobotEnd.HIND);
      else
         latestStep = latestSteps.get(RobotEnd.FRONT);

      // initialize nominal support rectangle
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         xGaitRectangle.get(robotQuadrant).changeFrame(xGaitRectangleFrame);
         xGaitRectangle.get(robotQuadrant).setX(robotQuadrant.getEnd().negateIfHindEnd(xGaitSettings.getStanceLength() / 2.0));
         xGaitRectangle.get(robotQuadrant).setY(robotQuadrant.getSide().negateIfRightSide(xGaitSettings.getStanceWidth() / 2.0));
         xGaitRectangle.get(robotQuadrant).setZ(0);
      }
      xGaitRectanglePoseAtSoS.setPosition(0, 0, 0);
      xGaitRectanglePoseAtSoS.setYawPitchRoll(currentYaw, 0, 0);

      // compute step quadrants and time intervals
      {
         RobotEnd thisStepRobotEnd = latestStep.getRobotQuadrant().getOppositeEnd();
         pastSteps.set(RobotEnd.FRONT, latestSteps.get(RobotEnd.FRONT));
         pastSteps.set(RobotEnd.HIND, latestSteps.get(RobotEnd.HIND));

         for (int i = 0; i < plannedSteps.size(); i++)
         {
            QuadrupedTimedStep thisStep = plannedSteps.get(i);
            thisStep.setRobotQuadrant(pastSteps.get(thisStepRobotEnd).getRobotQuadrant().getAcrossBodyQuadrant());
            double pastStepEndTimeForSameEnd = pastSteps.get(thisStepRobotEnd).getTimeInterval().getEndTime();
            double pastStepEndTimeForOppositeEnd = pastSteps.get(thisStepRobotEnd.getOppositeEnd()).getTimeInterval().getEndTime();
            computeStepTimeInterval(thisStep.getTimeInterval(), thisStepRobotEnd, pastStepEndTimeForSameEnd, pastStepEndTimeForOppositeEnd, xGaitSettings);

            pastSteps.set(thisStepRobotEnd, thisStep);
            thisStepRobotEnd = thisStepRobotEnd.getOppositeEnd();
         }
      }

      // compute step goal positions and ground clearances
      {
         for (int i = 0; i < plannedSteps.size(); i++)
         {
            // compute xGait rectangle pose at end of step
            double deltaTime = plannedSteps.get(i).getTimeInterval().getEndTime() - currentTime;
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

   // translate step goal positions based on latest step position
   {
      // compute xGait rectangle pose at end of step
      double deltaTime = latestStep.getTimeInterval().getEndTime() - currentTime;
      extrapolatePose(xGaitRectanglePose, xGaitRectanglePoseAtSoS, planarVelocity, deltaTime);
      xGaitRectangleFrame.setPoseAndUpdate(xGaitRectanglePose);

      // compute step goal position
      RobotQuadrant stepQuadrant = latestStep.getRobotQuadrant();
      goalPosition.setIncludingFrame(xGaitRectangle.get(stepQuadrant));
      goalPosition.changeFrame(ReferenceFrame.getWorldFrame());

      // compensate for position error
      for (int i = 0; i < plannedSteps.size(); i++)
      {
         plannedSteps.get(i).getGoalPosition().add(latestStep.getGoalPosition());
         plannedSteps.get(i).getGoalPosition().sub(goalPosition.getPoint());
      }
   }
}

   private void extrapolatePose(FramePose finalPose, FramePose initialPose, Vector3d planarVelocity, double deltaTime)
   {

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

   private void computeStepTimeInterval(TimeInterval thisStepTimeInterval, RobotEnd thisStepRobotEnd, double pastStepEndTimeForSameEnd, double pastStepEndTimeForOppositeEnd, QuadrupedXGaitSettings xGaitSettings)
   {
      // Compute support durations and end phase shift.
      double nominalStepDuration = xGaitSettings.getStepDuration();
      double endDoubleSupportDuration = xGaitSettings.getEndDoubleSupportDuration();
      double endPhaseShift = (thisStepRobotEnd == RobotEnd.HIND) ? 180.0 - xGaitSettings.getEndPhaseShift() : xGaitSettings.getEndPhaseShift();

      // Compute step time interval. Step duration is scaled in the range (1.0, 1.1667) to account for end phase shifts.
      double thisStepStartTime = pastStepEndTimeForSameEnd + endDoubleSupportDuration;
      double thisStepEndTime = pastStepEndTimeForOppositeEnd + Math.max((nominalStepDuration + endDoubleSupportDuration) * endPhaseShift / 180.0, 0.0);
      double thisStepDuration = MathTools.clipToMinMax(thisStepEndTime - thisStepStartTime, nominalStepDuration, 1.25 * nominalStepDuration);

      thisStepTimeInterval.setStartTime(thisStepStartTime);
      thisStepTimeInterval.setEndTime(thisStepStartTime + thisStepDuration);
   }
}