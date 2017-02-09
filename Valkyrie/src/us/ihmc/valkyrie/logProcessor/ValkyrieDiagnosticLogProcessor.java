package us.ihmc.valkyrie.logProcessor;

import java.io.IOException;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.logProcessor.DRCLogProcessor;
import us.ihmc.avatar.logProcessor.LogDataProcessorFunction;
import us.ihmc.avatar.logProcessor.LogDataProcessorHelper;
import us.ihmc.valkyrie.ValkyrieRobotModel;

public class ValkyrieDiagnosticLogProcessor extends DRCLogProcessor
{
   public ValkyrieDiagnosticLogProcessor() throws IOException
   {
      super();
      LogDataProcessorHelper logDataProcessorHelper = createLogDataProcessorHelper();
      LogDataProcessorFunction logDataProcessor = new DiagnosticLogProcessorFunction(logDataProcessorHelper);
      setLogDataProcessor(logDataProcessor);
      startLogger();
   }

   public static void main(String[] args) throws IOException
   {
      new ValkyrieDiagnosticLogProcessor();
   }

   @Override
   public DRCRobotModel createDRCRobotModel()
   {
      return new ValkyrieRobotModel(DRCRobotModel.RobotTarget.SCS, false);
   }
}
