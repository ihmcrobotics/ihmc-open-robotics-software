package us.ihmc.xboxHandGrasp;

import javax.swing.JFrame;
import javax.swing.JPanel;

import net.java.games.input.Event;
import us.ihmc.communication.streamingData.QueueBasedStreamingDataProducer;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HandConfiguration;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandDesiredConfigurationMessage;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.tools.inputDevices.joystick.Joystick;
import us.ihmc.tools.inputDevices.joystick.JoystickEventListener;
import us.ihmc.tools.inputDevices.joystick.JoystickStatusListener;

@SuppressWarnings("serial")
public class XboxHandControls extends JFrame implements JoystickStatusListener, JoystickEventListener
{

   private JPanel panel;
   private Joystick joy;
   private QueueBasedStreamingDataProducer<HandDesiredConfigurationMessage> handServer;
   private HandDesiredConfigurationMessage handCommand;

   public XboxHandControls()
   {
      super("Hand Controler");
      this.setSize(400, 400);
      this.setVisible(true);
      this.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
      panel = new JPanel();
      this.add(panel);
      this.repaint();
      this.revalidate();

      try
      {
         joy = new Joystick();
      }
      catch (Exception e)
      {
         e.printStackTrace();
      }
      joy.addJoystickEventListener(this);
      joy.addJoystickStatusListener(this);

      handServer = new QueueBasedStreamingDataProducer<HandDesiredConfigurationMessage>("Xbox_Hand_Controller");
      handServer.startProducingData();
   }

   @Override
   public void processEvent(Event e)
   {
      if (e.getComponent().toString().equals("Button 0"))
      {
         handCommand = new HandDesiredConfigurationMessage(RobotSide.LEFT, HandConfiguration.OPEN);
      }
      else if (e.getComponent().toString().equals("Button 1"))
      {
         handCommand = new HandDesiredConfigurationMessage(RobotSide.LEFT, HandConfiguration.CLOSE);
      }
      if (handCommand != null)
      {
         handServer.queueDataToSend(handCommand);
      }
   }

   @Override
   public void updateConnectivity(boolean connected)
   {
      // TODO Auto-generated method stub

   }

   public static void main(String[] args)
   {
      new XboxHandControls();
   }

}
