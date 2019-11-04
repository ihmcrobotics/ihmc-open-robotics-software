package us.ihmc.quadrupedFootstepPlanning.pawPlanning.turnWalkTurn;

import us.ihmc.commons.MathTools;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose2D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.quadrupedBasics.gait.QuadrupedTimedOrientedStep;
import us.ihmc.quadrupedBasics.gait.QuadrupedTimedStep;
import us.ihmc.quadrupedBasics.referenceFrames.QuadrupedReferenceFrames;
import us.ihmc.quadrupedPlanning.QuadrupedXGaitSettingsBasics;
import us.ihmc.quadrupedPlanning.QuadrupedXGaitSettingsReadOnly;
import us.ihmc.quadrupedPlanning.stepStream.bodyPath.QuadrupedWaypointBasedBodyPathProvider;
import us.ihmc.quadrupedPlanning.footstepChooser.PointFootSnapper;
import us.ihmc.quadrupedPlanning.stepStream.bodyPath.QuadrupedBodyPathPlan;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.robotSide.*;
import us.ihmc.robotics.time.TimeIntervalBasics;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.List;

public class QuadrupedXGaitStepCalculator
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final YoDouble minimumStepClearance = new YoDouble("minimumStepClearance", registry);
   private final YoDouble timestamp;

   private final QuadrupedXGaitSettingsBasics xGaitSettings;
   private final Vector3D desiredPlanarVelocity = new Vector3D();
   private final DoubleProvider firstStepDelay;

   private final QuadrupedWaypointBasedBodyPathProvider waypointBasedPath;

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final double maximumStepDown = 0.2;

   private final FramePose3D goalPose = new FramePose3D();
   private final FramePoint3D stepGoalPosition = new FramePoint3D();
   private final QuadrantDependentList<FramePoint3D> xGaitRectangle = new QuadrantDependentList<>();
   private final FramePose3D xGaitRectanglePose = new FramePose3D();
   private final PoseReferenceFrame xGaitRectangleFrame = new PoseReferenceFrame("xGaitRectangleFrame", worldFrame);
   private final EndDependentList<QuadrupedTimedStep> pastSteps;

   private final FramePose2D bodyPathPose = new FramePose2D();
   private PointFootSnapper snapper = null;

   private final EndDependentList<QuadrupedTimedOrientedStep> currentSteps = new EndDependentList<>(new QuadrupedTimedOrientedStep(), new QuadrupedTimedOrientedStep());
   private final RecyclingArrayList<QuadrupedTimedOrientedStep> stepPlan = new RecyclingArrayList<>(QuadrupedTimedOrientedStep::new);

   private final RecyclingArrayList<QuadrupedTimedOrientedStep> completeStepSequence = new RecyclingArrayList<>(QuadrupedTimedOrientedStep::new);

   public QuadrupedXGaitStepCalculator(QuadrupedXGaitSettingsBasics xGaitSettings, YoDouble timestamp, QuadrupedReferenceFrames referenceFrames,
                                       DoubleProvider firstStepDelay, YoGraphicsListRegistry graphicsListRegistry, YoVariableRegistry parentRegistry)
   {
      this.xGaitSettings = xGaitSettings;
      this.timestamp = timestamp;
      waypointBasedPath = new QuadrupedWaypointBasedBodyPathProvider(referenceFrames, timestamp, graphicsListRegistry, registry);

      this.firstStepDelay = firstStepDelay;
      minimumStepClearance.set(0.075);

      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         xGaitRectangle.set(robotQuadrant, new FramePoint3D(xGaitRectangleFrame));
      }
      pastSteps = new EndDependentList<>();
      pastSteps.put(RobotEnd.FRONT, new QuadrupedTimedStep());
      pastSteps.put(RobotEnd.HIND, new QuadrupedTimedStep());

      if (parentRegistry != null)
      {
         parentRegistry.addChild(registry);
      }
   }

   private void updateXGaitSettings()
   {
      // increase stance dimensions as a function of velocity to prevent self collisions
      double strideRotation = desiredPlanarVelocity.getZ() * xGaitSettings.getStepDuration();
      double strideLength = Math.abs(2 * desiredPlanarVelocity.getX() * xGaitSettings.getStepDuration());
      double strideWidth = Math.abs(2 * desiredPlanarVelocity.getY() * xGaitSettings.getStepDuration());
      strideLength += Math.abs(xGaitSettings.getStanceWidth() / 2 * Math.sin(2 * strideRotation));
      strideWidth += Math.abs(xGaitSettings.getStanceLength() / 2 * Math.sin(2 * strideRotation));
      xGaitSettings.setStanceLength(Math.max(xGaitSettings.getStanceLength(), strideLength / 2 + minimumStepClearance.getValue()));
      xGaitSettings.setStanceWidth(Math.max(xGaitSettings.getStanceWidth(), strideWidth / 2 + minimumStepClearance.getValue()));
   }

   public void setStepSnapper(PointFootSnapper snapper)
   {
      this.snapper = snapper;
   }

   public void setGoalPose(FramePose3DReadOnly goalPose)
   {
      this.goalPose.set(goalPose);
   }

   public void setBodyPathPlan(QuadrupedBodyPathPlan pathPlan)
   {
      waypointBasedPath.setBodyPathPlan(pathPlan);
   }

   public void onEntry()
   {
      // initialize step queue
      updateXGaitSettings();
      double initialTime = timestamp.getDoubleValue() + firstStepDelay.getValue();
      RobotQuadrant initialQuadrant = (xGaitSettings.getEndPhaseShift() < 90) ? RobotQuadrant.HIND_LEFT : RobotQuadrant.FRONT_LEFT;
      waypointBasedPath.initialize();
      computePlan(initialQuadrant, initialTime);

      initializeCurrentStepsFromPlannedSteps();

      updateOnline();
   }

   private void initializeCurrentStepsFromPlannedSteps()
   {
      for (int i = 0; i < 2; i++)
      {
         RobotEnd robotEnd = stepPlan.get(i).getRobotQuadrant().getEnd();
         currentSteps.get(robotEnd).set(stepPlan.get(i));
      }
   }

   public void updateOnline()
   {
      double currentTime = timestamp.getDoubleValue();

      // update xgait current steps
      updateCurrentSteps(timestamp.getDoubleValue());

      updateXGaitSettings();
      computeOnlinePlan(currentTime);
   }

   public List<QuadrupedTimedOrientedStep> getSteps()
   {
      return getCompleteStepSequence(timestamp.getDoubleValue());
   }



   public void computePlan(RobotQuadrant initialStepQuadrant, double timeAtStartOfStep)
   {
      // initialize nominal support rectangle
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         xGaitRectangle.get(robotQuadrant).changeFrame(xGaitRectangleFrame);
         xGaitRectangle.get(robotQuadrant).setX(robotQuadrant.getEnd().negateIfHindEnd(xGaitSettings.getStanceLength() / 2.0));
         xGaitRectangle.get(robotQuadrant).setY(robotQuadrant.getSide().negateIfRightSide(xGaitSettings.getStanceWidth() / 2.0));
         xGaitRectangle.get(robotQuadrant).setZ(0);
      }

      // plan steps
      double lastStepStartTime = timeAtStartOfStep;
      RobotQuadrant lastStepQuadrant = initialStepQuadrant.getNextReversedRegularGaitSwingQuadrant();
      QuadrantDependentList<QuadrupedTimedOrientedStep> lastSteps = new QuadrantDependentList<>();

      stepPlan.clear();
      while (!bodyAtGoal() || !stepsAreSquare(lastSteps))
      {
         QuadrupedTimedOrientedStep step = stepPlan.add();

         // compute step quadrant
         RobotQuadrant thisStepQuadrant = lastStepQuadrant.getNextRegularGaitSwingQuadrant();
         step.setRobotQuadrant(thisStepQuadrant);

         // compute step timing
         double thisStepStartTime;
         double thisStepEndTime;
         if (stepPlan.size() == 1)
         {
            thisStepStartTime = timeAtStartOfStep;
            thisStepEndTime = timeAtStartOfStep + xGaitSettings.getStepDuration();
         }
         else
         {
            double endPhaseShift = thisStepQuadrant.isQuadrantInHind() ? 180.0 - xGaitSettings.getEndPhaseShift() : xGaitSettings.getEndPhaseShift();
            double endTimeShift = xGaitSettings.getEndDoubleSupportDuration() + xGaitSettings.getStepDuration();
            endTimeShift *= Math.max(Math.min(endPhaseShift, 180.0), 0.0) / 180.0;
            thisStepStartTime = lastStepStartTime + endTimeShift;
            thisStepEndTime = thisStepStartTime + xGaitSettings.getStepDuration();
         }
         step.getTimeInterval().setStartTime(thisStepStartTime);
         step.getTimeInterval().setEndTime(thisStepEndTime);

         // compute xGait rectangle pose at end of step
         extrapolatePose(xGaitRectanglePose, thisStepEndTime);

         xGaitRectangleFrame.setPoseAndUpdate(xGaitRectanglePose);
         step.setStepYaw(xGaitRectanglePose.getYaw());

         // compute step goal position by sampling the corner position of the xGait rectangle at touch down
         RobotQuadrant robotQuadrant = step.getRobotQuadrant();
         stepGoalPosition.setIncludingFrame(xGaitRectangle.get(robotQuadrant));
         step.setGoalPosition(stepGoalPosition);
         snapStep(step);

         // compute step ground clearance
         step.setGroundClearance(xGaitSettings.getStepGroundClearance());

         lastSteps.put(thisStepQuadrant, step);

         // update state for next step
         lastStepStartTime = thisStepStartTime;
         lastStepQuadrant = thisStepQuadrant;
      }
   }

   public void computeOnlinePlan(double currentTime)
   {
      // initialize latest step
      QuadrupedTimedStep latestStep;

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

      QuadrantDependentList<QuadrupedTimedOrientedStep> lastSteps = new QuadrantDependentList<>();
      stepPlan.clear();

      RobotEnd thisStepEnd = latestStep.getRobotQuadrant().getOppositeEnd();
      pastSteps.set(RobotEnd.FRONT, currentSteps.get(RobotEnd.FRONT));
      pastSteps.set(RobotEnd.HIND, currentSteps.get(RobotEnd.HIND));

      while (!bodyAtGoal() || !stepsAreSquare(lastSteps))
      {
         // compute step quadrants and time intervals
         QuadrupedTimedOrientedStep thisStep = stepPlan.add();
         QuadrupedTimedStep pastStepOnSameEnd = pastSteps.get(thisStepEnd);
         QuadrupedTimedStep pastStepOnOppositeEnd = pastSteps.get(thisStepEnd.getOppositeEnd());

         RobotQuadrant thisStepQuadrant = pastStepOnSameEnd.getRobotQuadrant().getAcrossBodyQuadrant();
         thisStep.setRobotQuadrant(thisStepQuadrant);
         computeStepTimeInterval(thisStep, pastStepOnSameEnd, pastStepOnOppositeEnd, xGaitSettings);
         TimeIntervalBasics thisTimeInterval = thisStep.getTimeInterval();
         if (currentTime > thisTimeInterval.getStartTime())
            thisTimeInterval.shiftInterval(currentTime - thisTimeInterval.getStartTime());

         pastSteps.set(thisStepEnd, thisStep);
         thisStepEnd = thisStepEnd.getOppositeEnd();

         // compute step goal positions and ground clearances

         // compute xGait rectangle pose at end of step
         double endTime = thisTimeInterval.getEndTime();
         extrapolatePose(xGaitRectanglePose, endTime);
         xGaitRectangleFrame.setPoseAndUpdate(xGaitRectanglePose);
         thisStep.setStepYaw(xGaitRectanglePose.getYaw());

         // compute step goal position by sampling the corner position of the xGait rectangle at touchdown
         stepGoalPosition.setIncludingFrame(xGaitRectangle.get(thisStepQuadrant));
         thisStep.setGoalPosition(stepGoalPosition);

         // compute step ground clearance
         thisStep.setGroundClearance(xGaitSettings.getStepGroundClearance());

         lastSteps.put(thisStepQuadrant, thisStep);
      }

      // snap the desired footsteps to a height map, if provided
      for (int i = 0; i < stepPlan.size(); i++)
      {
         snapStep(stepPlan.get(i));
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
         stepGoalPosition.setIncludingFrame(worldFrame, step.getGoalPosition());
         step.setGoalPosition(snapper.snapStep(stepGoalPosition.getX(), stepGoalPosition.getY(), previousStepZValue - maximumStepDown));
      }
   }

   private void extrapolatePose(FramePose3D finalPose, double time)
   {
      waypointBasedPath.getPlanarPose(time, bodyPathPose);
      finalPose.setX(bodyPathPose.getX());
      finalPose.setY(bodyPathPose.getY());
      finalPose.setOrientationYawPitchRoll(bodyPathPose.getYaw(), finalPose.getPitch(), finalPose.getRoll());
   }

   private void computeStepTimeInterval(QuadrupedTimedStep thisStep, QuadrupedTimedStep pastStepOnSameEnd, QuadrupedTimedStep pastStepOnOppositeEnd,
                                        QuadrupedXGaitSettingsReadOnly xGaitSettings)
   {
      RobotEnd thisStepEnd = thisStep.getRobotQuadrant().getEnd();
      RobotSide thisStepSide = thisStep.getRobotQuadrant().getSide();
      RobotSide pastStepSide = pastStepOnOppositeEnd.getRobotQuadrant().getSide();

      double pastStepEndTimeForSameEnd = pastStepOnSameEnd.getTimeInterval().getEndTime();
      double pastStepEndTimeForOppositeEnd = pastStepOnOppositeEnd.getTimeInterval().getEndTime();

      // Compute support durations and end phase shift.
      double nominalStepDuration = xGaitSettings.getStepDuration();
      double endDoubleSupportDuration = xGaitSettings.getEndDoubleSupportDuration();
      double endPhaseShift = MathTools.clamp(xGaitSettings.getEndPhaseShift(), 0, 359);
      if (thisStepEnd == RobotEnd.HIND)
         endPhaseShift = 360 - endPhaseShift;
      if (pastStepSide != thisStepSide)
         endPhaseShift = endPhaseShift - 180;

      // Compute step time interval. Step duration is scaled in the range (1.0, 1.5) to account for end phase shifts.
      double thisStepStartTime = pastStepEndTimeForSameEnd + endDoubleSupportDuration;
      double thisStepEndTime = pastStepEndTimeForOppositeEnd + (nominalStepDuration + endDoubleSupportDuration) * endPhaseShift / 180.0;
      double thisStepDuration = MathTools.clamp(thisStepEndTime - thisStepStartTime, nominalStepDuration, 1.5 * nominalStepDuration);

      thisStep.getTimeInterval().setStartTime(thisStepStartTime);
      thisStep.getTimeInterval().setEndTime(thisStepStartTime + thisStepDuration);
   }


   private void updateCurrentSteps(double currentTime)
   {
      for (int i = 0; i < stepPlan.size(); i++)
      {
         QuadrupedTimedStep xGaitPreviewStep = stepPlan.get(i);
         if (xGaitPreviewStep.getTimeInterval().getStartTime() <= currentTime)
         {
            currentSteps.get(xGaitPreviewStep.getRobotQuadrant().getEnd()).set(xGaitPreviewStep);
         }
      }
   }

   private RecyclingArrayList<QuadrupedTimedOrientedStep> getCompleteStepSequence(double currentTime)
   {
      completeStepSequence.clear();
      for (RobotEnd robotEnd : RobotEnd.values)
      {
         if (currentSteps.get(robotEnd).getTimeInterval().getEndTime() >= currentTime)
         {
            completeStepSequence.add().set(currentSteps.get(robotEnd));
         }
      }
      for (int i = 0; i < stepPlan.size(); i++)
      {
         if (stepPlan.get(i).getTimeInterval().getEndTime() >= currentTime)
         {
            completeStepSequence.add().set(stepPlan.get(i));
         }
      }

      return completeStepSequence;
   }

   private static final double atGoalEpsilon = 2.0e-3;
   private final FramePoint3D stepPosition = new FramePoint3D();

   private boolean stepsAreSquare(QuadrantDependentList<QuadrupedTimedOrientedStep> lastSteps)
   {
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         QuadrupedTimedOrientedStep step = lastSteps.get(robotQuadrant);
         if (step == null)
            return false;

         stepPosition.changeFrame(worldFrame);
         stepPosition.set(step.getGoalPosition());
         stepPosition.changeFrame(xGaitRectangleFrame);

         RobotQuadrant quadrant = step.getRobotQuadrant();
         double xSquarePosition;
         double ySquarePosition;
         if (quadrant.isQuadrantInFront())
            xSquarePosition = 0.5 * xGaitSettings.getStanceLength();
         else
            xSquarePosition = -0.5 * xGaitSettings.getStanceLength();

         if (quadrant.isQuadrantOnLeftSide())
            ySquarePosition = 0.5 * xGaitSettings.getStanceWidth();
         else
            ySquarePosition = -0.5 * xGaitSettings.getStanceWidth();

         if (!MathTools.epsilonEquals(stepPosition.getX(), xSquarePosition, atGoalEpsilon))
            return false;
         if (!MathTools.epsilonEquals(stepPosition.getY(), ySquarePosition, atGoalEpsilon))
            return false;
      }

      return true;
   }

   private boolean bodyAtGoal()
   {
      return xGaitRectanglePose.epsilonEquals(goalPose, atGoalEpsilon);
   }
}
