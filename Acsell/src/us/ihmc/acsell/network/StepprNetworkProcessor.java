package us.ihmc.acsell.network;

import java.net.URISyntaxException;

import us.ihmc.acsell.parameters.BonoRobotModel;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.DRCNetworkModuleParameters;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.DRCNetworkProcessor;

import com.martiansoftware.jsap.JSAPException;

public class StepprNetworkProcessor
{
   private static final DRCRobotModel model = new BonoRobotModel(true, true);

   public static void main(String[] args) throws URISyntaxException, JSAPException
   {
            

      DRCNetworkModuleParameters networkModuleParams = new DRCNetworkModuleParameters();

      networkModuleParams.setUseLocalControllerCommunicator(false);
      networkModuleParams.setUseUiModule(true);
      networkModuleParams.setUseBehaviorModule(true);

      new DRCNetworkProcessor(model, networkModuleParams);
   }
}

