package us.ihmc.avatar.scs2;

import us.ihmc.avatar.wholeBodyHardwareControl.AvatarLowLevelOutputProcessor;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointReadOnly;
import us.ihmc.scs2.definition.controller.ControllerInput;
import us.ihmc.scs2.definition.controller.ControllerOutput;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListBasics;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputWriter;
import us.ihmc.tools.TimestampProvider;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;

public class InterpolatedSCS2OutputWriter implements JointDesiredOutputWriter
{
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final String robotName;
   private final double updateDt;

   private final boolean writeBeforeEstimatorTick;

   private JointDesiredOutputListBasics jointDesiredOutputList;
   private AvatarLowLevelOutputProcessor outputProcessor;
   private final SCS2OutputWriter scs2OutputWriter;

   public InterpolatedSCS2OutputWriter(String robotName,
                                       ControllerInput controllerInput,
                                       ControllerOutput controllerOutput,
                                       boolean writeBeforeEstimatorTick,
                                       double updateDt,
                                       JointDesiredOutputWriter customWriter)
   {
      this.robotName = robotName;
      this.updateDt = updateDt;
      this.writeBeforeEstimatorTick = writeBeforeEstimatorTick;

      scs2OutputWriter = new SCS2OutputWriter(controllerInput,
                                              controllerOutput,
                                              writeBeforeEstimatorTick,
                                              customWriter);

      registry.addChild(scs2OutputWriter.getYoVariableRegistry());
   }

   public void enableInterpolation(boolean enable)
   {
      outputProcessor.enableInterpolation(enable);
   }

   public void setInterpolationDuration(double duration)
   {
      outputProcessor.setInterpolationDuration(duration);
   }

   public void setYoTime(DoubleProvider yoTime)
   {
      outputProcessor.setYoTime(yoTime);
   }

   public void startDesiredsInterpolation()
   {
      outputProcessor.startDesiredsInterpolation();
   }

   @Override
   public void setJointDesiredOutputList(JointDesiredOutputListBasics jointDesiredOutputList)
   {
      this.jointDesiredOutputList = jointDesiredOutputList;
      OneDoFJointReadOnly[] joints = new OneDoFJointReadOnly[jointDesiredOutputList.getNumberOfJointsWithDesiredOutput()];
      for (int i = 0; i < jointDesiredOutputList.getNumberOfJointsWithDesiredOutput(); i++)
      {
         joints[i] = jointDesiredOutputList.getOneDoFJoint(i);
      }
      outputProcessor = new AvatarLowLevelOutputProcessor(robotName, joints, updateDt, registry);
      scs2OutputWriter.setJointDesiredOutputList(outputProcessor.getProcessedDesiredOutput());
   }

   @Override
   public void writeBefore(long timestamp)
   {
      if (writeBeforeEstimatorTick)
         outputProcessor.update(jointDesiredOutputList);
      scs2OutputWriter.writeBefore(timestamp);
   }

   @Override
   public void writeAfter()
   {
      if (!writeBeforeEstimatorTick)
         outputProcessor.update(jointDesiredOutputList);
      scs2OutputWriter.writeAfter();
   }

   @Override
   public YoRegistry getYoVariableRegistry()
   {
      return registry;
   }
}
