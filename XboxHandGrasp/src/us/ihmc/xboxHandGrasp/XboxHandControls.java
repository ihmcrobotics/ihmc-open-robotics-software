package us.ihmc.xboxHandGrasp;

import java.io.IOException;

import javax.swing.JFrame;
import javax.swing.JPanel;

import net.java.games.input.Event;
import us.ihmc.communication.configuration.NetworkParameterKeys;
import us.ihmc.communication.configuration.NetworkParameters;
import us.ihmc.humanoidOperatorInterface.networking.DRCUserInterfaceNetworkingManager;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HandConfiguration;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandDesiredConfigurationMessage;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.tools.inputDevices.joystick.Joystick;
import us.ihmc.tools.inputDevices.joystick.JoystickEventListener;

@SuppressWarnings("serial")
public class XboxHandControls extends JPanel implements JoystickEventListener
{

   private JFrame window;
   private Joystick joy;
   private HandDesiredConfigurationMessage handCommand;
   private DRCUserInterfaceNetworkingManager network;

   public XboxHandControls()
   {
      window = new JFrame("Hand Controller");
      window.setSize(400, 400);
      window.setVisible(true);
      window.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
      window.add(this);
      window.repaint();
      window.revalidate();

      try
      {
         joy = new Joystick();
      }
      catch (Exception e)
      {
         e.printStackTrace();
      }
      joy.addJoystickEventListener(this);
      
      String host = NetworkParameters.getHost(NetworkParameterKeys.networkManager);
      network = new DRCUserInterfaceNetworkingManager(host, null);
      try
      {
         network.connect();
      }
      catch (IOException e)
      {
         // TODO Auto-generated catch block
         e.printStackTrace();
      }
   }


   public static void main(String[] args)
   {
      new XboxHandControls();
   }


   @Override
   public void processEvent(Event event)
   {
      System.out.println(event.getComponent().toString());
      if (event.getValue() == 1)
      {
         switch (event.getComponent().toString())
         {
         case ("A"):
            handCommand = new HandDesiredConfigurationMessage(RobotSide.RIGHT, HandConfiguration.OPEN);
            break;
         case ("B"): 
            handCommand = new HandDesiredConfigurationMessage(RobotSide.RIGHT, HandConfiguration.CLOSE);
            break;
         case ("X"):
            handCommand = new HandDesiredConfigurationMessage(RobotSide.LEFT, HandConfiguration.OPEN);
            break;
         case ("Y"):
            handCommand = new HandDesiredConfigurationMessage(RobotSide.LEFT, HandConfiguration.CLOSE);
            break;
         }
      }
      if (handCommand != null)
      {
         network.sendPacket(handCommand);
      }
   }

}
