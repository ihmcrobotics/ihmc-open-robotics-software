package us.ihmc.quadrupedPlanning.stepStream;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose2D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.quadrupedBasics.gait.QuadrupedTimedOrientedStep;
import us.ihmc.quadrupedBasics.gait.QuadrupedTimedStep;
import us.ihmc.commons.lists.PreallocatedList;
import us.ihmc.quadrupedPlanning.QuadrupedXGaitSettingsReadOnly;
import us.ihmc.quadrupedPlanning.stepStream.bodyPath.QuadrupedPlanarBodyPathProvider;
import us.ihmc.quadrupedPlanning.footstepChooser.PointFootSnapper;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.robotSide.*;

public class QuadrupedXGaitPlanner
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final double maximumStepDown = 0.2;

   private final FramePoint3D goalPosition = new FramePoint3D();
   private final QuadrantDependentList<FramePoint3D> xGaitRectangle = new QuadrantDependentList<>();
   private final FramePose3D xGaitRectanglePose = new FramePose3D();
   private final PoseReferenceFrame xGaitRectangleFrame = new PoseReferenceFrame("xGaitRectangleFrame", worldFrame);
   private final EndDependentList<QuadrupedTimedStep> pastSteps;

   private final QuadrupedXGaitSettingsReadOnly xGaitSettings;
   private final QuadrupedPlanarBodyPathProvider bodyPathProvider;
   private final FramePose2D bodyPathPose = new FramePose2D();
   private PointFootSnapper snapper = null;

   public QuadrupedXGaitPlanner(QuadrupedPlanarBodyPathProvider bodyPathProvider, QuadrupedXGaitSettingsReadOnly xGaitSettings)
   {
      this.bodyPathProvider = bodyPathProvider;
      this.xGaitSettings = xGaitSettings;

      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         xGaitRectangle.set(robotQuadrant, new FramePoint3D(xGaitRectangleFrame));
      }
      pastSteps = new EndDependentList<>();
      pastSteps.put(RobotEnd.FRONT, new QuadrupedTimedStep());
      pastSteps.put(RobotEnd.HIND, new QuadrupedTimedStep());
   }

   public void computeInitialPlan(QuadrupedPlanarFootstepPlan footstepPlan, RobotQuadrant initialStepQuadrant, double timeAtSoS)
   {
//      bodyPathProvider.initialize();

      // initialize nominal support rectangle
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         xGaitRectangle.get(robotQuadrant).changeFrame(xGaitRectangleFrame);
         xGaitRectangle.get(robotQuadrant).setX(robotQuadrant.getEnd().negateIfHindEnd(xGaitSettings.getStanceLength() / 2.0));
         xGaitRectangle.get(robotQuadrant).setY(robotQuadrant.getSide().negateIfRightSide(xGaitSettings.getStanceWidth() / 2.0));
         xGaitRectangle.get(robotQuadrant).setZ(0);
      }

      // plan steps
      double lastStepStartTime = timeAtSoS;
      RobotQuadrant lastStepQuadrant = initialStepQuadrant.getNextReversedRegularGaitSwingQuadrant();
      PreallocatedList<QuadrupedTimedOrientedStep> plannedSteps = footstepPlan.getPlannedSteps();
      plannedSteps.clear();
      for (int i = 0; i < plannedSteps.capacity(); i++)
      {
         plannedSteps.add();
         QuadrupedTimedOrientedStep step = plannedSteps.get(plannedSteps.size() - 1);

         // compute step quadrant
         RobotQuadrant thisStepQuadrant = lastStepQuadrant.getNextRegularGaitSwingQuadrant();
         step.setRobotQuadrant(thisStepQuadrant);

         // compute step timing
         double endTimeShift;
         if (i == 0)
         {
            endTimeShift = 0.0;
         }
         else
         {
            endTimeShift = QuadrupedXGaitTools.computeTimeDeltaBetweenSteps(lastStepQuadrant, xGaitSettings);
         }
         double thisStepStartTime = lastStepStartTime + endTimeShift;
         double thisStepEndTime = thisStepStartTime + xGaitSettings.getStepDuration();

         step.getTimeInterval().setInterval(thisStepStartTime, thisStepEndTime);

         // compute xGait rectangle pose at end of step
         extrapolatePose(xGaitRectanglePose, thisStepEndTime);

         xGaitRectangleFrame.setPoseAndUpdate(xGaitRectanglePose);
         step.setStepYaw(xGaitRectanglePose.getYaw());

         // compute step goal position by sampling the corner position of the xGait rectangle at touch down
         RobotQuadrant robotQuadrant = step.getRobotQuadrant();
         this.goalPosition.setIncludingFrame(xGaitRectangle.get(robotQuadrant));
         step.setGoalPosition(this.goalPosition);
         snapStep(step);

         // compute step ground clearance
         step.setGroundClearance(xGaitSettings.getStepGroundClearance());

         // update state for next step
         lastStepStartTime = thisStepStartTime;
         lastStepQuadrant = thisStepQuadrant;
      }
   }

   public void computeOnlinePlan(QuadrupedPlanarFootstepPlan footstepPlan, double currentTime)
   {
      // initialize latest step
      QuadrupedTimedStep latestStep;
      EndDependentList<QuadrupedTimedOrientedStep> currentSteps = footstepPlan.getCurrentSteps();

      if (currentSteps.get(RobotEnd.HIND).getTimeInterval().getEndTime() > currentSteps.get(RobotEnd.FRONT).getTimeInterval().getEndTime())
         latestStep = currentSteps.get(RobotEnd.HIND);
      else
         latestStep = currentSteps.get(RobotEnd.FRONT);

      // initialize nominal support rectangle
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         xGaitRectangle.get(robotQuadrant).changeFrame(xGaitRectangleFrame);
         xGaitRectangle.get(robotQuadrant).setX(robotQuadrant.getEnd().negateIfHindEnd(xGaitSettings.getStanceLength() / 2.0));
         xGaitRectangle.get(robotQuadrant).setY(robotQuadrant.getSide().negateIfRightSide(xGaitSettings.getStanceWidth() / 2.0));
         xGaitRectangle.get(robotQuadrant).setZ(0);
      }

      PreallocatedList<QuadrupedTimedOrientedStep> plannedSteps = footstepPlan.getPlannedSteps();
      plannedSteps.clear();
      // compute step quadrants and time intervals
      RobotEnd thisStepEnd = latestStep.getRobotQuadrant().getOppositeEnd();
      pastSteps.set(RobotEnd.FRONT, currentSteps.get(RobotEnd.FRONT));
      pastSteps.set(RobotEnd.HIND, currentSteps.get(RobotEnd.HIND));

      for (int i = 0; i < plannedSteps.capacity(); i++)
      {
         plannedSteps.add();
         QuadrupedTimedStep thisStep = plannedSteps.get(i);
         QuadrupedTimedStep pastStepOnSameEnd = pastSteps.get(thisStepEnd);
         QuadrupedTimedStep pastStepOnOppositeEnd = pastSteps.get(thisStepEnd.getOppositeEnd());

         thisStep.setRobotQuadrant(pastStepOnSameEnd.getRobotQuadrant().getAcrossBodyQuadrant());
         QuadrupedXGaitTools.computeStepTimeInterval(thisStep, pastStepOnSameEnd, pastStepOnOppositeEnd, xGaitSettings);
         if (currentTime > thisStep.getTimeInterval().getStartTime())
            thisStep.getTimeInterval().shiftInterval(currentTime - thisStep.getTimeInterval().getStartTime());

         pastSteps.set(thisStepEnd, thisStep);
         thisStepEnd = thisStepEnd.getOppositeEnd();
      }

      // compute step goal positions and ground clearances
      for (int i = 0; i < plannedSteps.size(); i++)
      {
         // compute xGait rectangle pose at end of step
         double time = plannedSteps.get(i).getTimeInterval().getEndTime();
         extrapolatePose(xGaitRectanglePose, time);
         xGaitRectangleFrame.setPoseAndUpdate(xGaitRectanglePose);
         plannedSteps.get(i).setStepYaw(xGaitRectanglePose.getYaw());

         // compute step goal position by sampling the corner position of the xGait rectangle at touchdown
         RobotQuadrant stepQuadrant = plannedSteps.get(i).getRobotQuadrant();
         goalPosition.setIncludingFrame(xGaitRectangle.get(stepQuadrant));
         plannedSteps.get(i).setGoalPosition(goalPosition);

         // compute step ground clearance
         plannedSteps.get(i).setGroundClearance(xGaitSettings.getStepGroundClearance());
      }

      // snap the desired footsteps to a height map, if provided
      for (int i = 0; i < plannedSteps.size(); i++)
      {
         snapStep(plannedSteps.get(i));
      }
   }

   private void snapStep(QuadrupedTimedOrientedStep step)
   {
      snapStep(step, Double.NEGATIVE_INFINITY);
   }

   private void snapStep(QuadrupedTimedOrientedStep step, double previousStepZValue)
   {
      if (snapper != null)
      {
         goalPosition.setIncludingFrame(worldFrame, step.getGoalPosition());
         step.setGoalPosition(snapper.snapStep(goalPosition.getX(), goalPosition.getY(), previousStepZValue - maximumStepDown));
      }
   }

   private void extrapolatePose(FramePose3D finalPose, double time)
   {
      bodyPathProvider.getPlanarPose(time, bodyPathPose);
      finalPose.setX(bodyPathPose.getX());
      finalPose.setY(bodyPathPose.getY());
      finalPose.setOrientationYawPitchRoll(bodyPathPose.getYaw(), finalPose.getPitch(), finalPose.getRoll());
   }

   public void setStepSnapper(PointFootSnapper snapper)
   {
      this.snapper = snapper;
   }
}