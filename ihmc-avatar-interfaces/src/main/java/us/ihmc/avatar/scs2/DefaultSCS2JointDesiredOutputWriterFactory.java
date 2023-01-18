package us.ihmc.avatar.scs2;

import us.ihmc.scs2.definition.controller.ControllerInput;
import us.ihmc.scs2.definition.controller.ControllerOutput;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputWriter;

public class DefaultSCS2JointDesiredOutputWriterFactory implements SCS2JointDesiredOutputWriterFactory
{
   private JointDesiredOutputWriter customOutputWriter = null;
   private boolean writeBeforeEstimatorTick = true;

   public void setCustomJointDesiredOutputWriter(JointDesiredOutputWriter customOutputWriter)
   {
      this.customOutputWriter = customOutputWriter;
   }

   public void setWriteBeforeEstimatorTick(boolean writeBeforeEstimatorTick)
   {
      this.writeBeforeEstimatorTick = writeBeforeEstimatorTick;
   }

   public JointDesiredOutputWriter build(ControllerInput input, ControllerOutput output)
   {
      return new SCS2OutputWriter(input, output, writeBeforeEstimatorTick, customOutputWriter);
   }
}
