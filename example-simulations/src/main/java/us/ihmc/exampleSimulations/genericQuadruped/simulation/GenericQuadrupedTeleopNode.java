package us.ihmc.exampleSimulations.genericQuadruped.simulation;

import java.io.IOException;
import java.io.InputStream;

import javax.swing.JFrame;

import org.apache.commons.lang3.SystemUtils;

import us.ihmc.exampleSimulations.genericQuadruped.GenericQuadrupedSimulationFactory;
import us.ihmc.exampleSimulations.genericQuadruped.model.GenericQuadrupedModelFactory;
import us.ihmc.exampleSimulations.genericQuadruped.model.GenericQuadrupedPhysicalProperties;
import us.ihmc.exampleSimulations.genericQuadruped.parameters.GenericQuadrupedXGaitSettings;
import us.ihmc.quadrupedRobotics.input.QuadrupedXBoxAdapter;
import us.ihmc.robotModels.FullQuadrupedRobotModel;
import us.ihmc.tools.inputDevices.joystick.Joystick;
import us.ihmc.tools.inputDevices.joystick.JoystickModel;
import us.ihmc.wholeBodyController.parameters.ParameterLoaderHelper;

/**
 * To run with simulation, make sure {@link GenericQuadrupedSimulationFactory#USE_NETWORKING} is true
 */
public class GenericQuadrupedTeleopNode
{
   private GenericQuadrupedTeleopNode() throws IOException, InterruptedException
   {
      GenericQuadrupedModelFactory modelFactory = new GenericQuadrupedModelFactory();
      FullQuadrupedRobotModel fullRobotModel = modelFactory.createFullRobotModel();
      GenericQuadrupedPhysicalProperties physicalProperties = new GenericQuadrupedPhysicalProperties();
      GenericQuadrupedXGaitSettings xGaitSettings = new GenericQuadrupedXGaitSettings();

      String robotName = modelFactory.getRobotDescription().getName();
      Joystick joystick = new Joystick(JoystickModel.XBOX_ONE, 0);
      QuadrupedXBoxAdapter eventListener = new QuadrupedXBoxAdapter(robotName, joystick, fullRobotModel, xGaitSettings, physicalProperties);

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

      if (SystemUtils.IS_OS_WINDOWS)
         new JFrame("CLICK HERE TO DRIVE").setVisible(true);

      new GenericQuadrupedTeleopNode();
   }
}
