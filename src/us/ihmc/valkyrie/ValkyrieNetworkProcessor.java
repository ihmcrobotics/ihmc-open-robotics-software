package us.ihmc.valkyrie;

import java.net.URI;
import java.net.URISyntaxException;

import us.ihmc.communication.configuration.NetworkParameters;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.DRCNetworkModuleParameters;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.DRCNetworkProcessor;

import com.martiansoftware.jsap.JSAPException;

public class ValkyrieNetworkProcessor
{
   private static final DRCRobotModel model = new ValkyrieRobotModel(DRCRobotModel.RobotTarget.REAL_ROBOT, true);

   
   public static void main(String[] args) throws URISyntaxException, JSAPException
   {
      DRCNetworkModuleParameters networkModuleParams = new DRCNetworkModuleParameters();
      
      networkModuleParams.enableLocalControllerCommunicator(false);
      networkModuleParams.enableUiModule(true);
      networkModuleParams.enableBehaviorModule(true);
      networkModuleParams.enableBehaviorVisualizer(true);

//      uncomment these for the sensors
      URI rosuri = NetworkParameters.getROSURI();
      if(rosuri != null)
      {
         networkModuleParams.enableRosModule(true);
         networkModuleParams.setRosUri(rosuri);
         networkModuleParams.enableSensorModule(true);
         System.out.println("ROS_MASTER_URI="+rosuri);
      }
      
      new DRCNetworkProcessor(model, networkModuleParams);
   }
}
