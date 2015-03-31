package us.ihmc.valkyrie;

import java.net.URISyntaxException;

import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.DRCNetworkModuleParameters;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.DRCNetworkProcessor;

import com.martiansoftware.jsap.JSAPException;

public class ValkyrieNetworkProcessor
{
   private static final DRCRobotModel model = new ValkyrieRobotModel(true, true);

   
   public static void main(String[] args) throws URISyntaxException, JSAPException
   {
      DRCNetworkModuleParameters networkModuleParams = new DRCNetworkModuleParameters();
      
      networkModuleParams.setUseLocalControllerCommunicator(false);
      networkModuleParams.setUseUiModule(true);
      networkModuleParams.setUseBehaviorModule(true);
      networkModuleParams.setUseBehaviorVisualizer(true);
      
      new DRCNetworkProcessor(model, networkModuleParams);
   }
}
