package us.ihmc.avatar.scs2;

import us.ihmc.scs2.definition.controller.ControllerInput;
import us.ihmc.scs2.definition.controller.ControllerOutput;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputWriter;

import java.util.function.Function;

public class SCS2JointDesiredOutputWriterFactory
{
   public record OutputWriterInfo(ControllerInput controllerInput, ControllerOutput controllerOutput, boolean writeBeforeEstimatorTick) { }
   private Function<OutputWriterInfo, JointDesiredOutputWriter> customOutputWriterSupplier = outputWriterInfo -> null;
   private boolean writeBeforeEstimatorTick = true;

   public SCS2JointDesiredOutputWriterFactory()
   {
   }

   public SCS2JointDesiredOutputWriterFactory(Function<OutputWriterInfo, JointDesiredOutputWriter> customOutputWriterSupplier)
   {
      this.customOutputWriterSupplier = customOutputWriterSupplier;
   }

   public void setWriteBeforeEstimatorTick(boolean writeBeforeEstimatorTick)
   {
      this.writeBeforeEstimatorTick = writeBeforeEstimatorTick;
   }

   public boolean getWriteBeforeEstimatorTick()
   {
      return writeBeforeEstimatorTick;
   }

   public JointDesiredOutputWriter build(ControllerInput controllerInput, ControllerOutput controllerOutput)
   {
      return new SCS2OutputWriter(controllerInput,
                                  controllerOutput,
                                  writeBeforeEstimatorTick,
                                  customOutputWriterSupplier.apply(new OutputWriterInfo(controllerInput, controllerOutput, writeBeforeEstimatorTick)));
   }
}
