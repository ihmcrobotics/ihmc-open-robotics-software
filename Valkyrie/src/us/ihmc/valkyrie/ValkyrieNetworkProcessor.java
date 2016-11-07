package us.ihmc.valkyrie;

import java.net.URI;
import java.net.URISyntaxException;

import com.martiansoftware.jsap.JSAPException;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.networkProcessor.DRCNetworkModuleParameters;
import us.ihmc.avatar.networkProcessor.DRCNetworkProcessor;
import us.ihmc.communication.configuration.NetworkParameters;

public class ValkyrieNetworkProcessor
{
   private static final DRCRobotModel model = new ValkyrieRobotModel(DRCRobotModel.RobotTarget.REAL_ROBOT, true);

   
   public static void main(String[] args) throws URISyntaxException, JSAPException
   {
      DRCNetworkModuleParameters networkModuleParams = new DRCNetworkModuleParameters();
      
      networkModuleParams.enableControllerCommunicator(true);
      networkModuleParams.enableLocalControllerCommunicator(false);
      networkModuleParams.enableUiModule(true);
      networkModuleParams.enableBehaviorModule(false);
      networkModuleParams.enableBehaviorVisualizer(false);

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
