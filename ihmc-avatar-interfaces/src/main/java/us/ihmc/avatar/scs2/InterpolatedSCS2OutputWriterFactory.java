package us.ihmc.avatar.scs2;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.scs2.definition.controller.ControllerInput;
import us.ihmc.scs2.definition.controller.ControllerOutput;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputWriter;

public class InterpolatedSCS2OutputWriterFactory implements SCS2JointDesiredOutputWriterFactory
{
   private InterpolatedSCS2OutputWriter outputWriter;
   private DRCRobotModel robotModel;

   public InterpolatedSCS2OutputWriterFactory(DRCRobotModel robotModel)
   {
      this.robotModel = robotModel;
   }

   public InterpolatedSCS2OutputWriter getOutputWriter()
   {
      return outputWriter;
   }

   @Override
   public JointDesiredOutputWriter build(ControllerInput input, ControllerOutput output)
   {
      if (outputWriter == null)
      {
         outputWriter = new InterpolatedSCS2OutputWriter(robotModel.getSimpleRobotName(),
                                                         input,
                                                         output,
                                                         true,
                                                         robotModel.getEstimatorDT(),
                                                         null);
      }

      return outputWriter;
   }
}
