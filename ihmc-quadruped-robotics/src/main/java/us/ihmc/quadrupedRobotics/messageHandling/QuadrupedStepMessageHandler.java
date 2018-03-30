package us.ihmc.quadrupedRobotics.messageHandling;

import us.ihmc.commons.MathTools;
import us.ihmc.communication.streamingData.GlobalDataProducer;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameQuaternionReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.quadrupedRobotics.communication.packets.QuadrupedTimedStepPacket;
import us.ihmc.quadrupedRobotics.estimator.referenceFrames.QuadrupedReferenceFrames;
import us.ihmc.quadrupedRobotics.planning.QuadrupedTimedStep;
import us.ihmc.quadrupedRobotics.planning.YoQuadrupedTimedStep;
import us.ihmc.quadrupedRobotics.util.TimeIntervalTools;
import us.ihmc.quadrupedRobotics.util.YoPreallocatedList;
import us.ihmc.robotics.math.frames.YoFrameOrientation;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.ArrayList;
import java.util.concurrent.atomic.AtomicReference;

public class QuadrupedStepMessageHandler
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final int STEP_QUEUE_SIZE = 10;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final AtomicReference<QuadrupedTimedStepPacket> inputTimedStepPacket = new AtomicReference<>();
   private final QuadrupedReferenceFrames referenceFrames;
   private final YoFrameOrientation bodyOrientation;
   private final YoPreallocatedList<YoQuadrupedTimedStep> stepSequence;
   private final ArrayList<YoQuadrupedTimedStep> activeSteps = new ArrayList<>();
   private final YoDouble robotTimestamp;
   private final DoubleParameter haltTransitionDurationParameter = new DoubleParameter("haltTransitionDuration", registry, 1.0);

   private final YoDouble haltTime = new YoDouble("haltTime", registry);
   private final YoBoolean haltFlag = new YoBoolean("haltFlag", registry);

   private final FramePoint3D tempPoint = new FramePoint3D();

   public QuadrupedStepMessageHandler(YoDouble robotTimestamp, QuadrupedReferenceFrames referenceFrames, GlobalDataProducer globalDataProducer, YoVariableRegistry parentRegistry)
   {
      this.robotTimestamp = robotTimestamp;
      this.stepSequence = new YoPreallocatedList<>("stepSequence", registry, STEP_QUEUE_SIZE, YoQuadrupedTimedStep::new);
      this.referenceFrames = referenceFrames;
      this.bodyOrientation = new YoFrameOrientation("bodyOrientation", ReferenceFrame.getWorldFrame(), registry);

      if(globalDataProducer != null)
      {
         globalDataProducer.attachListener(QuadrupedTimedStepPacket.class, inputTimedStepPacket::set);
      }

      parentRegistry.addChild(registry);
   }

   public boolean isStepPlanAvailable()
   {
      return inputTimedStepPacket.get() != null;
   }

   public void getBodyOrientation(FrameQuaternion bodyOrientation)
   {
      bodyOrientation.setIncludingFrame(this.bodyOrientation.getFrameOrientation());
   }

   public FrameQuaternionReadOnly getBodyOrientation()
   {
      return bodyOrientation.getFrameOrientation();
   }

   /**
    * Consumes incoming footsteps and adjusts their position by the given vector
    */
   public void process(FrameVector3DReadOnly stepAdjustment)
   {
      if(isStepPlanAvailable())
      {
         updateStepSequence();
         adjustStepQueue(stepAdjustment);
      }
   }

   private void updateStepSequence()
   {
      double currentTime = robotTimestamp.getDoubleValue();
      QuadrupedTimedStepPacket stepPacket = this.inputTimedStepPacket.getAndSet(null);

      boolean isExpressedInAbsoluteTime = stepPacket.isExpressedInAbsoluteTime;
      ArrayList<QuadrupedTimedStep> steps = stepPacket.getSteps();

      stepSequence.clear();
      for (int i = 0; i < Math.min(steps.size(), STEP_QUEUE_SIZE) ; i++)
      {
         double timeShift = isExpressedInAbsoluteTime ? 0.0 : currentTime;
         double touchdownTime = steps.get(i).getTimeInterval().getEndTime();
         double shiftedTouchdownTime = touchdownTime + timeShift;

         boolean shouldNotBeHalted = !haltFlag.getBooleanValue() || shiftedTouchdownTime < haltTime.getDoubleValue();
         boolean isFutureStep = shiftedTouchdownTime >= currentTime;

         if (shouldNotBeHalted && isFutureStep)
         {
            stepSequence.add();
            YoQuadrupedTimedStep step = stepSequence.get(stepSequence.size() - 1);
            step.set(steps.get(i));
            step.getTimeInterval().shiftInterval(timeShift);
         }
      }

      TimeIntervalTools.sortByEndTime(stepSequence);
      bodyOrientation.setFromReferenceFrame(referenceFrames.getCenterOfFeetZUpFrameAveragingLowestZHeightsAcrossEnds());
      updateActiveSteps();
   }

   public void initialize()
   {
      haltFlag.set(false);
   }

   private void adjustStepQueue(FrameVector3DReadOnly stepAdjustment)
   {
      for (int i = 0; i < stepSequence.size(); i++)
      {
         YoQuadrupedTimedStep step = stepSequence.get(i);
         step.getGoalPosition(tempPoint);
         tempPoint.changeFrame(worldFrame);
         tempPoint.add(stepAdjustment);
         step.setGoalPosition(tempPoint);
      }
   }

   public boolean isDoneWithStepSequence()
   {
      return stepSequence.size() == 0 || stepSequence.get(stepSequence.size() - 1).getTimeInterval().getEndTime() < robotTimestamp.getDoubleValue();
   }

   public void halt()
   {
      if (haltFlag.getBooleanValue() == false)
      {
         haltFlag.set(true);
         haltTime.set(robotTimestamp.getDoubleValue() + haltTransitionDurationParameter.getValue());
      }
   }

   public YoPreallocatedList<YoQuadrupedTimedStep> getStepSequence()
   {
      return stepSequence;
   }

   public ArrayList<YoQuadrupedTimedStep> getActiveSteps()
   {
      return activeSteps;
   }

   private void updateActiveSteps()
   {
      activeSteps.clear();

      for (int i = 0; i < stepSequence.size(); i++)
      {
         double currentTime = robotTimestamp.getDoubleValue();
         double startTime = stepSequence.get(i).getTimeInterval().getStartTime();
         double endTime = stepSequence.get(i).getTimeInterval().getEndTime();

         if (MathTools.intervalContains(currentTime, startTime, endTime))
         {
            activeSteps.add(stepSequence.get(i));
         }
      }
   }

   public void reset()
   {
      inputTimedStepPacket.set(null);
      stepSequence.clear();
   }
}
