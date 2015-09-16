package us.ihmc.steppr.network;

import java.net.URISyntaxException;

import com.martiansoftware.jsap.JSAPException;

import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.DRCNetworkModuleParameters;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.DRCNetworkProcessor;
import us.ihmc.steppr.parameters.BonoRobotModel;

public class StepprNetworkProcessor
{
   private static final DRCRobotModel model = new BonoRobotModel(true, true);

   public static void main(String[] args) throws URISyntaxException, JSAPException
   {
            

      DRCNetworkModuleParameters networkModuleParams = new DRCNetworkModuleParameters();

      networkModuleParams.enableLocalControllerCommunicator(false);
      networkModuleParams.enableUiModule(true);
      networkModuleParams.enableBehaviorModule(true);

      new DRCNetworkProcessor(model, networkModuleParams);
   }
}

