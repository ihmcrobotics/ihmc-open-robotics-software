package us.ihmc.quadrupedRobotics.planning.stepStream;

import us.ihmc.quadrupedRobotics.estimator.referenceFrames.QuadrupedReferenceFrames;
import us.ihmc.quadrupedRobotics.planning.QuadrupedTimedStep;
import us.ihmc.quadrupedRobotics.providers.QuadrupedTimedStepInputProvider;
import us.ihmc.quadrupedRobotics.util.PreallocatedQueue;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FrameOrientation;

import java.util.ArrayList;

public class QuadrupedPreplannedStepStream implements QuadrupedStepStream
{
   private static int MAXIMUM_STEP_QUEUE_SIZE = 100;

   private final QuadrupedTimedStepInputProvider timedStepInputProvider;
   private final QuadrupedReferenceFrames referenceFrames;
   private final DoubleYoVariable timestamp;
   private final PreallocatedQueue<QuadrupedTimedStep> stepQueue;
   private final FrameOrientation bodyOrientation;

   public QuadrupedPreplannedStepStream(QuadrupedTimedStepInputProvider timedStepInputProvider, QuadrupedReferenceFrames referenceFrames, DoubleYoVariable timestamp)
   {
      this.timedStepInputProvider = timedStepInputProvider;
      this.referenceFrames = referenceFrames;
      this.timestamp = timestamp;
      this.stepQueue = new PreallocatedQueue<>(QuadrupedTimedStep.class, MAXIMUM_STEP_QUEUE_SIZE);
      this.bodyOrientation = new FrameOrientation();
   }

   @Override
   public void onEntry()
   {
      double currentTime = timestamp.getDoubleValue();
      ArrayList<QuadrupedTimedStep> steps = timedStepInputProvider.get();
      stepQueue.clear();
      for (int i = 0; i < steps.size(); i++)
      {
         if (steps.get(i).getTimeInterval().getEndTime() <= currentTime)
         {
            stepQueue.enqueue();
            stepQueue.getTail().set(steps.get(i));
            if (!stepQueue.getTail().isAbsolute())
            {
               stepQueue.getTail().getTimeInterval().shiftInterval(currentTime);
               stepQueue.getTail().setAbsolute(true);
            }
         }
      }
   }

   @Override
   public void process()
   {
      bodyOrientation.setToZero(referenceFrames.getCenterOfFeetZUpFrameAveragingLowestZHeightsAcrossEnds());
   }

   @Override
   public void getBodyOrientation(FrameOrientation bodyOrientation)
   {
      bodyOrientation.setIncludingFrame(this.bodyOrientation);
   }

   @Override
   public PreallocatedQueue<QuadrupedTimedStep> getSteps()
   {
      return stepQueue;
   }
}
