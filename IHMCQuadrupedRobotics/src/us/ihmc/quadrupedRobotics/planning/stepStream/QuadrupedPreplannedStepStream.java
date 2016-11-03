package us.ihmc.quadrupedRobotics.planning.stepStream;

import us.ihmc.quadrupedRobotics.estimator.referenceFrames.QuadrupedReferenceFrames;
import us.ihmc.quadrupedRobotics.planning.QuadrupedTimedStep;
import us.ihmc.quadrupedRobotics.planning.YoQuadrupedTimedStep;
import us.ihmc.quadrupedRobotics.providers.QuadrupedPreplannedStepInputProvider;
import us.ihmc.quadrupedRobotics.util.PreallocatedList;
import us.ihmc.quadrupedRobotics.util.TimeIntervalTools;
import us.ihmc.quadrupedRobotics.util.YoPreallocatedList;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.math.frames.YoFrameOrientation;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

import java.util.ArrayList;

public class QuadrupedPreplannedStepStream implements QuadrupedStepStream
{
   private static int MAXIMUM_STEP_QUEUE_SIZE = 100;
   private final YoVariableRegistry registry;
   private final QuadrupedPreplannedStepInputProvider preplannedStepProvider;
   private final QuadrupedReferenceFrames referenceFrames;
   private final DoubleYoVariable timestamp;
   private final YoFrameOrientation bodyOrientation;
   private final YoPreallocatedList<YoQuadrupedTimedStep> stepSequence;

   public QuadrupedPreplannedStepStream(QuadrupedPreplannedStepInputProvider preplannedStepProvider, QuadrupedReferenceFrames referenceFrames, DoubleYoVariable timestamp, YoVariableRegistry parentRegistry)
   {
      this.registry = new YoVariableRegistry(getClass().getSimpleName());
      this.preplannedStepProvider = preplannedStepProvider;
      this.referenceFrames = referenceFrames;
      this.timestamp = timestamp;
      this.bodyOrientation = new YoFrameOrientation("bodyOrientation", ReferenceFrame.getWorldFrame(), registry);
      this.stepSequence = new YoPreallocatedList<>("stepSequence", registry, MAXIMUM_STEP_QUEUE_SIZE,
            new YoPreallocatedList.DefaultElementFactory<YoQuadrupedTimedStep>()
            {
               @Override
               public YoQuadrupedTimedStep createDefaultElement(String prefix, YoVariableRegistry registry)
               {
                  return new YoQuadrupedTimedStep(prefix, registry);
               }
            });

      if (parentRegistry != null)
      {
         parentRegistry.addChild(registry);
      }
   }

   @Override
   public void onEntry()
   {
      double currentTime = timestamp.getDoubleValue();
      boolean isExpressedInAbsoluteTime = preplannedStepProvider.isStepPlanExpressedInAbsoluteTime();
      ArrayList<QuadrupedTimedStep> steps = preplannedStepProvider.getAndClearSteps();
      stepSequence.clear();
      for (int i = 0; i < steps.size(); i++)
      {
         double timeShift = isExpressedInAbsoluteTime ? 0.0 : currentTime;
         double touchdownTime = steps.get(i).getTimeInterval().getEndTime();
         if (touchdownTime + timeShift >= currentTime)
         {
            stepSequence.add();
            stepSequence.get(stepSequence.size() - 1).set(steps.get(i));
            stepSequence.get(stepSequence.size() - 1).getTimeInterval().shiftInterval(timeShift);
         }
      }
      TimeIntervalTools.sortByEndTime(stepSequence);

      bodyOrientation.setFromReferenceFrame(referenceFrames.getCenterOfFeetZUpFrameAveragingLowestZHeightsAcrossEnds());
   }

   @Override
   public void process()
   {
      // dequeue completed steps
      double currentTime = timestamp.getDoubleValue();
      TimeIntervalTools.removeEndTimesLessThan(currentTime, stepSequence);

      bodyOrientation.setFromReferenceFrame(referenceFrames.getCenterOfFeetZUpFrameAveragingLowestZHeightsAcrossEnds());
   }

   @Override
   public void onExit()
   {
      stepSequence.clear();
   }

   @Override
   public void getBodyOrientation(FrameOrientation bodyOrientation)
   {
      bodyOrientation.setIncludingFrame(this.bodyOrientation.getFrameOrientation());
   }

   @Override
   public PreallocatedList<? extends QuadrupedTimedStep> getSteps()
   {
      return stepSequence;
   }
}
