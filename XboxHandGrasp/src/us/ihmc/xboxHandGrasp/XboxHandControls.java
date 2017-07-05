package us.ihmc.xboxHandGrasp;

import java.awt.event.KeyEvent;
import java.awt.event.KeyListener;
import java.io.IOException;

import javax.swing.JFrame;
import javax.swing.JPanel;

import net.java.games.input.Event;
import us.ihmc.communication.configuration.NetworkParameterKeys;
import us.ihmc.communication.configuration.NetworkParameters;
import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.streamingData.QueueBasedStreamingDataProducer;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.humanoidBehaviors.communication.CommunicationBridge;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HandConfiguration;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandDesiredConfigurationMessage;
import us.ihmc.humanoidRobotics.kryo.IHMCCommunicationKryoNetClassList;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.tools.inputDevices.joystick.Joystick;
import us.ihmc.tools.inputDevices.joystick.JoystickEventListener;
import us.ihmc.tools.inputDevices.joystick.JoystickStatusListener;

@SuppressWarnings("serial")
public class XboxHandControls extends JPanel implements KeyListener//JoystickStatusListener, JoystickEventListener
{

   private JFrame window;
   private Joystick joy;
   private HandDesiredConfigurationMessage handCommand;
   private PacketCommunicator packetCom;

   public XboxHandControls()
   {
      window = new JFrame("Hand Controller");
      window.setSize(400, 400);
      window.setVisible(true);
      window.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
      window.add(this);
      window.addKeyListener(this);
      window.repaint();
      window.revalidate();

//      try
//      {
//         joy = new Joystick();
//      }
//      catch (Exception e)
//      {
//         e.printStackTrace();
//      }
//      joy.addJoystickEventListener(this);
//      joy.addJoystickStatusListener(this);

      String host = NetworkParameters.getHost(NetworkParameterKeys.networkManager);
      packetCom = PacketCommunicator.createTCPPacketCommunicatorClient(host, NetworkPorts.LEFT_HAND_PORT, new IHMCCommunicationKryoNetClassList());
      try
      {
         packetCom.connect();
      }
      catch (IOException e)
      {
         // TODO Auto-generated catch block
         e.printStackTrace();
      }
   }

   @Override
   public void keyPressed(KeyEvent e)
   {
      if (e.getKeyCode() == 65)
      {
         handCommand = new HandDesiredConfigurationMessage(RobotSide.LEFT, HandConfiguration.OPEN);
         System.out.println("Open");
      }
      else if (e.getKeyCode() == 66)
      {
         handCommand = new HandDesiredConfigurationMessage(RobotSide.LEFT, HandConfiguration.CLOSE);
         System.out.println("Close");
      }
      if (handCommand != null)
      {
         packetCom.send(handCommand);
      }
      
   }



   public static void main(String[] args)
   {
      new XboxHandControls();
   }

   
   
   @Override
   public void keyTyped(KeyEvent e)
   {
      // TODO Auto-generated method stub
      
   }
   
   @Override
   public void keyReleased(KeyEvent e)
   {
      // TODO Auto-generated method stub
      
   }


}
