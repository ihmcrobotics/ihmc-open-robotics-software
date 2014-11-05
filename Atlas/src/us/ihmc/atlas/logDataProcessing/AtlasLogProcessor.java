package us.ihmc.atlas.logDataProcessing;

import java.io.IOException;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.darpaRoboticsChallenge.logProcessor.DRCLogProcessor;

public class AtlasLogProcessor extends DRCLogProcessor
{
   public AtlasLogProcessor() throws IOException
   {
      super();
      
      FootRotationProcessor footRotationProcessor = new FootRotationProcessor(logDataProcessorHelper);
      setLogDataProcessor(footRotationProcessor);
      
      startLogger();
   }

   public static void main(String[] args) throws IOException
   {
      new AtlasLogProcessor();
   }

   @Override
   public DRCRobotModel createDRCRobotModel()
   {
      return new AtlasRobotModel(AtlasRobotVersion.DRC_NO_HANDS, AtlasRobotModel.AtlasTarget.SIM, false);
   }
}
