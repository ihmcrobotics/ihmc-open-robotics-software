package us.ihmc.atlas.logDataProcessing;

import java.io.IOException;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.logProcessor.DRCLogProcessor;
import us.ihmc.avatar.logProcessor.LogDataProcessorHelper;

public class AtlasLogProcessor extends DRCLogProcessor
{
   public AtlasLogProcessor() throws IOException
   {
      super();
      
      LogDataProcessorHelper logDataProcessorHelper = createLogDataProcessorHelper();
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
      return new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, DRCRobotModel.RobotTarget.SCS, false);
   }
}
