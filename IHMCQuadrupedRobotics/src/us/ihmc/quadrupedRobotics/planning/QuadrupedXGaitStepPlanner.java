package us.ihmc.quadrupedRobotics.planning;

import us.ihmc.quadrupedRobotics.estimator.GroundPlaneEstimator;
import us.ihmc.quadrupedRobotics.util.PreallocatedQueue;
import us.ihmc.graphics3DAdapter.graphics.appearances.AppearanceDefinition;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.quadrupedRobotics.estimator.referenceFrames.QuadrupedReferenceFrames;
import us.ihmc.quadrupedRobotics.geometry.supportPolygon.QuadrupedSupportPolygon;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotEnd;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.BagOfBalls;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicsListRegistry;

/**
 * A footstep planner capable of planning pace, amble, and trot gaits.
 */
public class QuadrupedXGaitStepPlanner
{
   /**
    * The number of footsteps to generate for each foot. The total number of steps in a plan is 4 times this number.
    */
   private static final int FOOTSTEP_QUEUE_SIZE = 5;

   private static final RobotQuadrant[] FOOTSTEP_ORDER = new RobotQuadrant[] { RobotQuadrant.HIND_LEFT,
         RobotQuadrant.FRONT_LEFT, RobotQuadrant.HIND_RIGHT, RobotQuadrant.FRONT_RIGHT };

   /**
    * The time a foot should spend in the air during its swing state in s.
    */
   private double swingDuration = 0.3;

   /**
    * The time during which both feet of a front or back pair are in support in s.
    */
   private double endPairSupportDuration = 1.0;

   /**
    * The phase offset of the front and back feet pairs in deg.
    */
   private double phaseShift = 60.0;

   /**
    * The distance between same-side legs during stance.
    */
   private double stanceLength = 1.1;

   /**
    * The distance between same-end legs during stance.
    */
   private double stanceWidth = 0.35;

   /**
    * The forward distance at which a footstep is placed from the previous step.
    */
   private double strideLength = 0.2;

   /**
    * The sideways distance at which a footstep is placed from the previous step.
    */
   private double strideWidth = 0.0;

   /**
    * The swing foot apex height.
    */
   private double groundClearance = 0.1;

   // TODO: Compute conversion of arbitrary yaw rate units to rad/s.
   private double yawRate = 0.1;

   /**
    * The appearance of the footstep visualization markers for each robot quadrant.
    */
   private static final QuadrantDependentList<AppearanceDefinition> footstepAppearance = new QuadrantDependentList<>(YoAppearance.Red(), YoAppearance.Blue(),
         YoAppearance.Salmon(), YoAppearance.LightBlue());

   private final QuadrupedReferenceFrames referenceFrames;
   private final BagOfBalls footstepVisualization;

   /**
    * Maintain four footstep queues -- one for each quadrant. This way, they can be merge-sorted in order of start time
    * into a full plan. This is necessary because some combinations of parameters generate the steps out of order.
    */
   private final QuadrantDependentList<PreallocatedQueue<QuadrupedTimedStep>> queues = new QuadrantDependentList<>();

   /**
    * Estimator used to compute the ground plane at the beginning of each step plan.
    */
   private static final GroundPlaneEstimator groundPlaneEstimator = new GroundPlaneEstimator();
   private static final QuadrantDependentList<FramePoint> initialFootholds = new QuadrantDependentList();

   /**
    * Temporary variables used to compute the step plan.
    */
   private final RigidBodyTransform temporaryTransform = new RigidBodyTransform();
   private final FramePoint temporaryFramePoint = new FramePoint();
   private final FrameVector temporaryFrameVector = new FrameVector();

   public QuadrupedXGaitStepPlanner(YoVariableRegistry registry, YoGraphicsListRegistry graphicsListRegistry,
         QuadrupedReferenceFrames referenceFrames)
   {
      this.referenceFrames = referenceFrames;

      for (RobotQuadrant quadrant : RobotQuadrant.values)
      {
         queues.set(quadrant, new PreallocatedQueue<>(QuadrupedTimedStep.class, FOOTSTEP_QUEUE_SIZE));
      }

      this.footstepVisualization = BagOfBalls
            .createRainbowBag(FOOTSTEP_QUEUE_SIZE * 4, 0.03, "footsteps", registry, graphicsListRegistry);

      for (RobotQuadrant quadrant : RobotQuadrant.values)
      {
         initialFootholds.set(quadrant, new FramePoint());
      }
   }

   /**
    * Clear the provided queue and fill it with a new plan.
    *
    * @param steps              the queue to fill
    * @param startTime          the desired start time of the first footstep
    * @param needInitialization whether or not initialization steps need to be added to the beginning of the queue. If
    *                           planning from a cold start in which feet are in arbitrary positions, pass {@code true}.
    *                           Otherwise, if feet are already following a plan and this is just an incremental replan,
    *                           pass {@code false}.
    */
   public void plan(PreallocatedQueue<QuadrupedTimedStep> steps, double startTime, boolean needInitialization)
   {
      footstepVisualization.reset();

      // Estimate the ground plane.
      for (RobotQuadrant quadrant : RobotQuadrant.values)
      {
         initialFootholds.get(quadrant).setToZero(referenceFrames.getFootFrame(quadrant));
      }
      groundPlaneEstimator.compute(initialFootholds);

      // Add the initialization steps.
      double initializationPeriod = needInitialization ? addInitializationSteps(startTime) : 0.0;

      // Fill the queue with steps.
      final double gaitPeriod = 2.0 * swingDuration + 2.0 * endPairSupportDuration;
      for (int i = 0; i < FOOTSTEP_QUEUE_SIZE - 1; i++)
      {
         addFourSteps(initializationPeriod + startTime + i * gaitPeriod);
      }

      // Draw the footstep visualization.
      footstepVisualization.setVisible(true);
      for (RobotQuadrant quadrant : RobotQuadrant.values)
      {
         for (int i = 0; i < queues.get(quadrant).size(); i++)
         {
            QuadrupedTimedStep step = queues.get(quadrant).get(i);

            step.getGoalPosition().changeFrame(referenceFrames.getWorldFrame());
            footstepVisualization
                  .setBallLoop(step.getGoalPosition(), footstepAppearance.get(quadrant));
         }
      }

      // Clear the queue to make room for our plan.
//      while (steps.dequeue())
//         ;

      // Dequeue footsteps from each of the quadrant queues and add them to the plan ordered by their start time.
      mergeFootstepQueues(steps);
   }

   /**
    * Add some steps to bring the feet to their initial positions, from which the gait can begin.
    *
    * @param startTime the time at which the first initialization step should begin
    * @return the initialization period
    */
   private double addInitializationSteps(double startTime)
   {
      int stepIndex = 0;
      for (RobotQuadrant quadrant : FOOTSTEP_ORDER)
      {
         QuadrupedTimedStep step = addZeroStep(startTime + stepIndex * (swingDuration + endPairSupportDuration),
               quadrant);

         // Set initial stance position based on stanceWidth/stanceHeight in the body frame
         FramePoint stepGoalPosition = step.getGoalPosition();
         stepGoalPosition.setToZero(referenceFrames.getBodyZUpFrame());

         double initialX = quadrant.getEnd().negateIfHindEnd(stanceLength / 2.0);
         double initialY = quadrant.getSide().negateIfRightSide(stanceWidth / 2.0);

         stepGoalPosition.set(initialX, initialY, 0.0);

         // Project onto the ground.
         groundPlaneEstimator.projectZ(stepGoalPosition);

         // Shift right feet forward half a stride.
         if (quadrant.isQuadrantOnRightSide())
         {
            step.getGoalPosition().add(strideLength / 2.0, strideWidth / 2.0, 0.0);
         }

         stepIndex++;
      }

      return 4 * (swingDuration + endPairSupportDuration);
   }

   private void addFourSteps(double startTime)
   {
      // Compute the period of this end pair, or the time between steps of the same foot.
      double endPairPeriod = 2.0 * swingDuration + 2.0 * endPairSupportDuration;

      // Compute the time these steps should be delayed to meet the desired phase shift.
      double phaseShiftDt = endPairPeriod * phaseShift / 360.0;

      addStepsForEndPair(startTime, RobotEnd.HIND);
      addStepsForEndPair(startTime + phaseShiftDt, RobotEnd.FRONT);
   }

   private void addStepsForEndPair(double startTime, RobotEnd end)
   {
      addStepFromPrevious(startTime, RobotQuadrant.getQuadrant(end, RobotSide.LEFT));
      addStepFromPrevious(startTime + swingDuration + endPairSupportDuration,
            RobotQuadrant.getQuadrant(end, RobotSide.RIGHT));
   }

   /**
    * Add a step based on the position of the previous step of the same quadrant.
    *
    * @see #addZeroStep(double, RobotQuadrant)
    */
   private QuadrupedTimedStep addStepFromPrevious(double startTime, RobotQuadrant quadrant)
   {
      FramePoint previousStepGoalPosition = getPreviousFootstepPosition(quadrant);

      // Start with the new step positioned at the previous step for this quadrant.
      QuadrupedTimedStep step = addZeroStep(startTime, quadrant);
      FramePoint stepGoalPosition = step.getGoalPosition();
      stepGoalPosition.setIncludingFrame(previousStepGoalPosition);
      stepGoalPosition.changeFrame(referenceFrames.getWorldFrame());
      groundPlaneEstimator.projectZ(stepGoalPosition);

      // Compute the support polygon for the previous group of four footsteps.
      QuadrupedSupportPolygon polygon = new QuadrupedSupportPolygon();
      for (RobotQuadrant q : RobotQuadrant.values)
      {
         polygon.setFootstep(q, getPreviousFootstepPosition(q));
      }
      polygon.changeFrame(referenceFrames.getWorldFrame());

      // From the previous support polygon, yaw the next support polygon by the desired turn rate.
      double nominalYaw = polygon.getNominalYaw();
      polygon.yawAboutCentroid(yawRate);

      RigidBodyTransform tf = temporaryTransform;
      tf.rotZ(nominalYaw);

      FramePoint centroid = temporaryFramePoint;
      polygon.getCentroid(centroid);

      // Compute the stride vector and rotate it to the new yaw angle.
      FrameVector stride = temporaryFrameVector;
      stride.setToZero(referenceFrames.getWorldFrame());
      stride.add(quadrant.getEnd().negateIfHindEnd(stanceLength) / 2.0, quadrant.getSide().negateIfRightSide(stanceWidth) / 2.0, 0.0);
      stride.add(strideLength, strideWidth, 0.0);
      stride.applyTransform(tf);
      stride.add(centroid);

//    step.setGoalPosition(polygon.getFootstep(quadrant));
      step.getGoalPosition().set(stride);
      groundPlaneEstimator.projectZ(step.getGoalPosition());

      return step;
   }

   /**
    * Add a step with a goal position at the current foot position.
    *
    * @param startTime the swing phase start time for this step
    * @param quadrant  the quadrant for this step
    * @return the new step
    */
   private QuadrupedTimedStep addZeroStep(double startTime, RobotQuadrant quadrant)
   {
      queues.get(quadrant).enqueue();
      QuadrupedTimedStep step = queues.get(quadrant).getTail();

      step.setRobotQuadrant(quadrant);
      step.getTimeInterval().setInterval(0.0, swingDuration);
      step.getTimeInterval().shiftInterval(startTime);
      step.setGroundClearance(groundClearance);
      step.getGoalPosition().setToZero(referenceFrames.getFootFrame(quadrant));

      return step;
   }

   /**
    * Gets the previous in-quadrant footstep's target position. If there are no previous steps, then gets the current
    * sole position estimate.
    */
   private FramePoint getPreviousFootstepPosition(RobotQuadrant quadrant)
   {
      if (queues.get(quadrant).size() > 0)
      {
         return queues.get(quadrant).getTail().getGoalPosition();
      }
      return initialFootholds.get(quadrant);
   }

   /**
    * Drain the four quadrant footstep queues into the main queue ordered by start time.
    *
    * @param steps the output queue
    */
   private void mergeFootstepQueues(PreallocatedQueue<QuadrupedTimedStep> steps)
   {
      while (true)
      {
         RobotQuadrant smallestQuadrant = null;
         for (RobotQuadrant quadrant : RobotQuadrant.values)
         {
            if (queues.get(quadrant).size() == 0)
            {
               continue;
            }

            if (smallestQuadrant == null)
            {
               smallestQuadrant = quadrant;
            }

            double smallestTime = queues.get(smallestQuadrant).getHead().getTimeInterval().getStartTime();
            double thisTime = queues.get(quadrant).getHead().getTimeInterval().getStartTime();
            if (thisTime < smallestTime)
            {

               smallestQuadrant = quadrant;
            }
         }

         if (smallestQuadrant == null)
         {
            break;
         }

         steps.enqueue();
         steps.getTail().set(queues.get(smallestQuadrant).getHead());

         queues.get(smallestQuadrant).dequeue();
      }
   }

   public void setVisible(boolean visible)
   {
      footstepVisualization.setVisible(visible);
   }

   public double getSwingDuration()
   {
      return swingDuration;
   }

   public void setSwingDuration(double swingDuration)
   {
      this.swingDuration = swingDuration;
   }

   public double getEndPairSupportDuration()
   {
      return endPairSupportDuration;
   }

   public void setEndPairSupportDuration(double endPairSupportDuration)
   {
      this.endPairSupportDuration = endPairSupportDuration;
   }

   public double getPhaseShift()
   {
      return phaseShift;
   }

   public void setPhaseShift(double phaseShift)
   {
      this.phaseShift = phaseShift;
   }

   public double getStanceLength()
   {
      return stanceLength;
   }

   public void setStanceLength(double stanceLength)
   {
      this.stanceLength = stanceLength;
   }

   public double getStanceWidth()
   {
      return stanceWidth;
   }

   public void setStanceWidth(double stanceWidth)
   {
      this.stanceWidth = stanceWidth;
   }

   public double getStrideLength()
   {
      return strideLength;
   }

   public void setStrideLength(double strideLength)
   {
      this.strideLength = strideLength;
   }

   public double getStrideWidth()
   {
      return strideWidth;
   }

   public double getYawRate()
   {
      return yawRate;
   }

   public void setYawRate(double yawRate)
   {
      this.yawRate = yawRate;
   }

   public void setStrideWidth(double strideWidth)
   {
      this.strideWidth = strideWidth;
   }
}
