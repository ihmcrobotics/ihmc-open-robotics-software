package us.ihmc.steppr.logProcessor;

import java.io.IOException;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.logProcessor.DRCLogProcessor;
import us.ihmc.steppr.parameters.BonoRobotModel;

public class StepprLogProcessor extends DRCLogProcessor
{

   public StepprLogProcessor() throws IOException
   {
      super();
//      LogDataProcessorFunction stepprSensorPostProcessor = new StepprSensorPostProcessor(drcRobotModel, logDataProcessorHelper);
//      setLogDataProcessor(stepprSensorPostProcessor);
      StepprKneeHysterisisCompensationLogProcessor stepprKneeHysterisisCompensationLogProcessor = new StepprKneeHysterisisCompensationLogProcessor(scs);
      setLogDataProcessor(stepprKneeHysterisisCompensationLogProcessor);
      startLogger();
   }

   public static void main(String[] args) throws IOException
   {
      new StepprLogProcessor();
   }

   @Override
   public DRCRobotModel createDRCRobotModel()
   {
      return new BonoRobotModel(true, false);
   }

}
