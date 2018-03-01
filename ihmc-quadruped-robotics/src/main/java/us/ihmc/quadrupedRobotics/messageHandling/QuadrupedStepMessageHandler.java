package us.ihmc.quadrupedRobotics.messageHandling;

import us.ihmc.commons.MathTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.quadrupedRobotics.planning.YoQuadrupedTimedStep;
import us.ihmc.quadrupedRobotics.planning.stepStream.QuadrupedStepStream;
import us.ihmc.robotics.dataStructures.parameter.DoubleParameter;
import us.ihmc.robotics.dataStructures.parameter.ParameterFactory;
import us.ihmc.robotics.lists.GenericTypeBuilder;
import us.ihmc.robotics.lists.RecyclingArrayList;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.ArrayList;

public class QuadrupedStepMessageHandler
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private static final int STEP_QUEUE_SIZE = 10;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final RecyclingArrayList<YoQuadrupedTimedStep> stepSequence;

   private final ArrayList<YoQuadrupedTimedStep> activeSteps = new ArrayList<>();

   private final QuadrupedStepStream stepStream;
   private final YoDouble robotTimestamp;

   private final ParameterFactory parameterFactory = ParameterFactory.createWithRegistry(getClass(), registry);
   private final DoubleParameter haltTransitionDurationParameter = parameterFactory.createDouble("haltTransitionDuration", 1.0);

   private final FramePoint3D tempPoint = new FramePoint3D();

   private final YoDouble haltTime = new YoDouble("haltTime", registry);
   private final YoBoolean haltFlag = new YoBoolean("haltFlag", registry);

   public QuadrupedStepMessageHandler(QuadrupedStepStream stepStream, YoDouble robotTimestamp, YoVariableRegistry parentRegistry)
   {
      this.stepStream = stepStream;
      this.robotTimestamp = robotTimestamp;

      stepSequence = new RecyclingArrayList<>(STEP_QUEUE_SIZE, new GenericTypeBuilder<YoQuadrupedTimedStep>()
      {
         private int stepNumber = 0;
         @Override
         public YoQuadrupedTimedStep newInstance()
         {
            YoQuadrupedTimedStep step = new YoQuadrupedTimedStep("stepSequence" + stepNumber, registry);
            stepNumber++;
            return step;
         }
      });

      parentRegistry.addChild(registry);
   }

   public void initialize()
   {
      haltFlag.set(false);
   }

   public void consumeIncomingSteps()
   {
      // update step plan
      stepSequence.clear();
      for (int i = 0; i < stepStream.getSteps().size(); i++)
      {
         if (!haltFlag.getBooleanValue() || stepStream.getSteps().get(i).getTimeInterval().getEndTime() < haltTime.getDoubleValue())
         {
            YoQuadrupedTimedStep step = stepSequence.add();
            step.set(stepStream.getSteps().get(i));
         }
      }

      updateActiveSteps();
   }

   public void adjustStepQueue(FrameVector3DReadOnly stepAdjustment)
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
      return stepSequence.size() == 0 || stepSequence.getLast().getTimeInterval().getEndTime() < robotTimestamp.getDoubleValue();
   }

   public void halt()
   {
      if (haltFlag.getBooleanValue() == false)
      {
         haltFlag.set(true);
         haltTime.set(robotTimestamp.getDoubleValue() + haltTransitionDurationParameter.get());
      }
   }

   public RecyclingArrayList<YoQuadrupedTimedStep> getStepSequence()
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

}
