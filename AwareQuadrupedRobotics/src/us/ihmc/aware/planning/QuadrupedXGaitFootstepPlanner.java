package us.ihmc.aware.planning;

import us.ihmc.aware.util.PreallocatedQueue;
import us.ihmc.aware.util.QuadrupedTimedStep;
import us.ihmc.quadrupedRobotics.referenceFrames.QuadrupedReferenceFrames;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;

/**
 * A footstep planner capable of planning pace, amble, and trot gaits.
 */
public class QuadrupedXGaitFootstepPlanner
{
   private static final int FOOTSTEP_QUEUE_SIZE = 15;

   /**
    * The time a foot should spend in the air during its swing state in s.
    */
   private double swingDuration = 1.0;

   /**
    * The time during which both feet of a front or back pair are in support in s.
    */
   private double endPairSupportDuration = 1.0;

   /**
    * The phase offset of the front and back feet pairs in deg.
    */
   private double phaseShift = 90.0;

   private double strideLength = 0.3;
   private double strideWidth = 0.0;

   private final QuadrupedReferenceFrames referenceFrames;

   /**
    * Maintain four footstep queues -- one for each quadrant. This way, they can be merge-sorted in order of start time
    * into a full plan. This is necessary because some combinations of parameters generate the steps out of order.
    */
   private final QuadrantDependentList<PreallocatedQueue<QuadrupedTimedStep>> queues = new QuadrantDependentList<>();

   public QuadrupedXGaitFootstepPlanner(QuadrupedReferenceFrames referenceFrames)
   {
      this.referenceFrames = referenceFrames;

      for (RobotQuadrant quadrant : RobotQuadrant.values)
      {
         queues.set(quadrant, new PreallocatedQueue<>(QuadrupedTimedStep.class, FOOTSTEP_QUEUE_SIZE));
      }
   }

   public void plan(PreallocatedQueue<QuadrupedTimedStep> steps, double startTime)
   {
      addInitializationSteps(startTime);

      // Fill the queue with steps.
      final double gaitPeriod = 2.0 * swingDuration + 2.0 * endPairSupportDuration;
      for (int i = 1; i < FOOTSTEP_QUEUE_SIZE - 4; i++)
      {
         addFourSteps(startTime + i * gaitPeriod);
      }

      // Clear the queue to make room for our plan.
      while (steps.dequeue())
         ;

      // Dequeue footsteps from each of the quadrant queues and add them to the plan ordered by their start time.
      mergeFootstepQueues(steps);
   }

   private void addInitializationSteps(double startTime)
   {
      for (RobotQuadrant quadrant : RobotQuadrant.values)
      {
         queues.get(quadrant).enqueue();
         QuadrupedTimedStep step = queues.get(quadrant).getTail();

         step.setRobotQuadrant(quadrant);
         step.getTimeInterval().setInterval(0.0, swingDuration);
         step.getTimeInterval().shiftInterval(startTime + swingDuration * quadrant.ordinal());

         step.getGoalPosition().setToZero(referenceFrames.getFootFrame(quadrant));
         step.getGoalPosition().changeFrame(referenceFrames.getBodyZUpFrame());

         // Shift the right side forward by half the stride length to make the gait symmetric.
         if (quadrant.isQuadrantOnRightSide())
         {
            step.getGoalPosition().add(strideLength / 2.0, 0.0, 0.0);
         }
      }
   }

   private void addFourSteps(double startTime)
   {
      // Compute the period of this end pair, or the time between steps of the same foot.
      double endPairPeriod = 2 * swingDuration + 2 * endPairSupportDuration;

      // Compute the time these steps should be delayed to meet the desired phase shift.
      double phaseShiftDt = endPairPeriod * phaseShift / 360.0;

      addStepsForEndPair(startTime, RobotQuadrant.HIND_LEFT, RobotQuadrant.HIND_RIGHT);
      addStepsForEndPair(startTime + phaseShiftDt, RobotQuadrant.FRONT_LEFT, RobotQuadrant.FRONT_RIGHT);
   }

   private void addStepsForEndPair(double startTime, RobotQuadrant left, RobotQuadrant right)
   {
      addStep(startTime, left);
      addStep(startTime + swingDuration + endPairSupportDuration, right);
   }

   private void addStep(double startTime, RobotQuadrant quadrant)
   {
      QuadrupedTimedStep previous = queues.get(quadrant).getTail();

      queues.get(quadrant).enqueue();
      QuadrupedTimedStep step = queues.get(quadrant).getTail();

      step.setRobotQuadrant(quadrant);
      step.getTimeInterval().setInterval(0.0, swingDuration);
      step.getTimeInterval().shiftInterval(startTime);

      step.getGoalPosition().setIncludingFrame(previous.getGoalPosition());
      step.getGoalPosition().changeFrame(referenceFrames.getBodyZUpFrame());
      step.getGoalPosition().add(strideLength, strideWidth, 0.0);
   }

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

   public void setStrideWidth(double strideWidth)
   {
      this.strideWidth = strideWidth;
   }
}
