package us.ihmc.exampleSimulations.genericQuadruped.simulation;

import us.ihmc.communication.net.NetClassList;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.exampleSimulations.genericQuadruped.GenericQuadrupedNetClassList;
import us.ihmc.exampleSimulations.genericQuadruped.model.GenericQuadrupedModelFactory;
import us.ihmc.exampleSimulations.genericQuadruped.model.GenericQuadrupedPhysicalProperties;
import us.ihmc.quadrupedRobotics.input.QuadrupedBodyTeleopNode;
import us.ihmc.robotModels.FullQuadrupedRobotModel;
import us.ihmc.tools.inputDevices.joystick.Joystick;
import us.ihmc.tools.inputDevices.joystick.JoystickModel;
import us.ihmc.exampleSimulations.genericQuadruped.GenericQuadrupedSimulationFactory;
import us.ihmc.wholeBodyController.parameters.ParameterLoaderHelper;
import us.ihmc.yoVariables.parameters.YoParameter;

import java.io.IOException;
import java.io.InputStream;

/**
 * To run with simulation, make sure {@link GenericQuadrupedSimulationFactory#USE_NETWORKING} is true
 */
public class GenericQuadrupedTeleopNode
{
   private static final String parameterResourcePath = "/parameters/teleop.xml";

   private GenericQuadrupedTeleopNode() throws IOException, InterruptedException
   {
      String host = "localhost";
      NetworkPorts port = NetworkPorts.CONTROLLER_PORT;
      NetClassList netClassList = new GenericQuadrupedNetClassList();
      GenericQuadrupedModelFactory modelFactory = new GenericQuadrupedModelFactory();
      FullQuadrupedRobotModel fullRobotModel = modelFactory.createFullRobotModel();
      GenericQuadrupedPhysicalProperties physicalProperties = new GenericQuadrupedPhysicalProperties();

      Joystick joystick = new Joystick(JoystickModel.XBOX_ONE, 0);
      QuadrupedBodyTeleopNode eventListener = new QuadrupedBodyTeleopNode(host, port, netClassList, joystick, fullRobotModel, physicalProperties);

      InputStream parameterFile = getClass().getResourceAsStream(parameterResourcePath);
      ParameterLoaderHelper.loadParameters(this, parameterFile, eventListener.getRegistry());

      eventListener.start();
      joystick.addJoystickEventListener(eventListener);
   }

   public static void main(String[] args) throws IOException, InterruptedException
   {
      boolean joystickIsConnected = Joystick.isAJoystickConnectedToSystem();
      if(!joystickIsConnected)
      {
         throw new RuntimeException("No joystick detected!");
      }

      new GenericQuadrupedTeleopNode();
   }
}
