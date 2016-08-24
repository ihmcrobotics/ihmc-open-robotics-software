package us.ihmc.quadrupedRobotics.planning.stepInput;

import us.ihmc.quadrupedRobotics.estimator.referenceFrames.QuadrupedReferenceFrames;
import us.ihmc.quadrupedRobotics.planning.QuadrupedTimedStep;
import us.ihmc.quadrupedRobotics.providers.QuadrupedMultiStepInputProvider;
import us.ihmc.quadrupedRobotics.util.PreallocatedQueue;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FrameOrientation;

import java.util.ArrayList;

public class QuadrupedMultiStepInputPlanner implements QuadrupedStepInputPlanner
{
   private static int MAXIMUM_STEP_QUEUE_SIZE = 100;

   private final QuadrupedMultiStepInputProvider multiStepInputProvider;
   private final QuadrupedReferenceFrames referenceFrames;
   private final DoubleYoVariable timestamp;
   private final PreallocatedQueue<QuadrupedTimedStep> stepQueue;
   private final FrameOrientation bodyOrientation;

   public QuadrupedMultiStepInputPlanner(QuadrupedMultiStepInputProvider multiStepInputProvider, QuadrupedReferenceFrames referenceFrames, DoubleYoVariable timestamp)
   {
      this.multiStepInputProvider = multiStepInputProvider;
      this.referenceFrames = referenceFrames;
      this.timestamp = timestamp;
      this.stepQueue = new PreallocatedQueue<>(QuadrupedTimedStep.class, MAXIMUM_STEP_QUEUE_SIZE);
      this.bodyOrientation = new FrameOrientation();
   }

   @Override
   public void initialize()
   {
      ArrayList<QuadrupedTimedStep> steps = multiStepInputProvider.get();
      stepQueue.clear();
      for (int i = 0; i < steps.size(); i++)
      {
         stepQueue.enqueue();
         stepQueue.getTail().set(steps.get(i));
         if (!stepQueue.getTail().isAbsolute())
         {
            stepQueue.getTail().getTimeInterval().shiftInterval(timestamp.getDoubleValue());
            stepQueue.getTail().setAbsolute(true);
         }
      }
   }

   @Override
   public void compute()
   {
      bodyOrientation.setToZero(referenceFrames.getCenterOfFeetZUpFrameAveragingLowestZHeightsAcrossEnds());
   }

   @Override
   public void getBodyOrientation(FrameOrientation bodyOrientation)
   {
      bodyOrientation.setIncludingFrame(this.bodyOrientation);
   }

   @Override
   public PreallocatedQueue<QuadrupedTimedStep> getStepQueue()
   {
      return stepQueue;
   }
}
