package us.ihmc.quadrupedRobotics.planning;

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
   private final QuadrupedXGaitSettings xGaitSettings;
   private final QuadrantDependentList<FramePoint> xGaitRectangle;
   private final FramePose xGaitRectanglePose;
   private final PoseReferenceFrame xGaitRectangleFrame;

   public QuadrupedXGaitPlanner()
   {
      xGaitSettings = new QuadrupedXGaitSettings();
      xGaitRectangle = new QuadrantDependentList<>();
      xGaitRectanglePose = new FramePose();
      xGaitRectangleFrame = new PoseReferenceFrame("xGaitRectangleFrame", ReferenceFrame.getWorldFrame());
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         xGaitRectangle.set(robotQuadrant, new FramePoint(xGaitRectangleFrame));
      }
   }

   public QuadrupedXGaitSettings getXGaitSettings()
   {
      return xGaitSettings;
   }

   public void plan(ArrayList<QuadrupedTimedStep> plannedSteps, Vector3d planarVelocity, RobotQuadrant initialStepQuadrant, FramePoint initialStepGoalPosition, double initialStepStartTime, double initialYaw)
   {
      // initialize nominal support rectangle
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         xGaitRectangle.get(robotQuadrant).changeFrame(xGaitRectangleFrame);
         xGaitRectangle.get(robotQuadrant).setX(robotQuadrant.getEnd().negateIfHindEnd(xGaitSettings.getStanceLength() / 2.0));
         xGaitRectangle.get(robotQuadrant).setY(robotQuadrant.getSide().negateIfRightSide(xGaitSettings.getStanceWidth() / 2.0));
         xGaitRectangle.get(robotQuadrant).setZ(initialStepGoalPosition.getZ());
      }

      // initialize position and orientation of support rectangle in world frame
      initialStepGoalPosition.changeFrame(ReferenceFrame.getWorldFrame());
      double a0 = initialYaw;
      double x0 = 0.0;
      double y0 = 0.0;
      double z0 = 0.0;

      // initialize forward, lateral, and rotational velocity in rectangle frame
      double u = planarVelocity.getX();
      double v = planarVelocity.getY();
      double phi = planarVelocity.getZ();

      // plan steps
      double previousStepStartTime = initialStepStartTime;
      RobotQuadrant previousStepQuadrant = initialStepQuadrant.getNextReversedRegularGaitSwingQuadrant();

      for (int i = 0; i < plannedSteps.size(); i++)
      {
         QuadrupedTimedStep step = plannedSteps.get(i);

         // compute step quadrant
         RobotQuadrant stepQuadrant = previousStepQuadrant.getNextRegularGaitSwingQuadrant();
         step.setRobotQuadrant(stepQuadrant);

         // compute step timing
         step.getTimeInterval().setInterval(0.0, xGaitSettings.getStepDuration());
         step.getTimeInterval().shiftInterval(previousStepStartTime);
         if (i > 0)
         {
            double endPhaseShift = xGaitSettings.getEndPhaseShift();
            double supportTime = xGaitSettings.getStepDuration() + xGaitSettings.getEndPairSupportDuration();
            double phaseShift = stepQuadrant.isQuadrantInHind() ? (180.0 - endPhaseShift) : endPhaseShift;
            double timeShift = supportTime * Math.max(Math.min(phaseShift, 180.0), 0.0) / 180.0;
            step.getTimeInterval().shiftInterval(timeShift);
         }
         double stepStartTime = step.getTimeInterval().getStartTime();
         double stepEndTime = step.getTimeInterval().getEndTime();

         // compute xGait rectangle pose at end of step
         double a, x, y, z;
         double epsilon = 0.001;
         if (Math.abs(phi) > epsilon)
         {
            a = a0 + phi * (stepEndTime - initialStepStartTime);
            x = x0 + u / phi * (Math.sin(a) - Math.sin(a0)) + v / phi * (Math.cos(a) - Math.cos(a0));
            y = y0 - u / phi * (Math.cos(a) - Math.cos(a0)) + v / phi * (Math.sin(a) - Math.sin(a0));
            z = z0;
         }
         else
         {
            a = a0;
            x = x0 + (u * Math.cos(a) - v * Math.sin(a)) * (stepEndTime - initialStepStartTime);
            y = y0 + (u * Math.sin(a) + v * Math.cos(a)) * (stepEndTime - initialStepStartTime);
            z = z0;
         }
         xGaitRectanglePose.setYawPitchRoll(a, 0.0, 0.0);
         xGaitRectanglePose.setX(x);
         xGaitRectanglePose.setY(y);
         xGaitRectanglePose.setZ(z);
         xGaitRectangleFrame.setPoseAndUpdate(xGaitRectanglePose);

         // compute step goal position by sampling the corner position of the xGait rectangle at touch down
         RobotQuadrant robotQuadrant = step.getRobotQuadrant();
         FramePoint goalPosition = step.getGoalPosition();
         goalPosition.changeFrame(ReferenceFrame.getWorldFrame());
         goalPosition.setIncludingFrame(xGaitRectangle.get(robotQuadrant));
         goalPosition.changeFrame(ReferenceFrame.getWorldFrame());

         // compute step ground clearance
         step.setGroundClearance(xGaitSettings.getStepGroundClearance());

         // update book-keeping
         previousStepStartTime = stepStartTime;
         previousStepQuadrant = stepQuadrant;
      }

      // translate step goal positions to match initial constraint
      for (int i = plannedSteps.size(); i > 0; i--)
      {
         plannedSteps.get(i - 1).getGoalPosition().sub(plannedSteps.get(0).getGoalPosition());
         plannedSteps.get(i - 1).getGoalPosition().add(initialStepGoalPosition);
      }
   }
}
