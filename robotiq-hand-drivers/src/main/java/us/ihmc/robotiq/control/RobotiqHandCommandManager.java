package us.ihmc.robotiq.control;

import java.awt.FlowLayout;
import java.awt.GridBagConstraints;
import java.awt.GridBagLayout;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.io.IOException;

import javax.swing.JButton;
import javax.swing.JComboBox;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.JSlider;

import us.ihmc.avatar.handControl.HandCommandManager;
import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HandConfiguration;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandDesiredConfigurationMessage;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandJointAnglePacket;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.ManualHandControlPacket;
import us.ihmc.humanoidRobotics.kryo.IHMCCommunicationKryoNetClassList;
import us.ihmc.robotics.robotSide.RobotSide;

public class RobotiqHandCommandManager extends HandCommandManager
{
   public RobotiqHandCommandManager(RobotSide robotSide)
   {
      super(RobotiqControlThread.class, robotSide);

      setupOutboundPacketListeners();
      setupInboundPacketListeners();
   }

   protected void setupInboundPacketListeners()
   {
      handManagerPacketCommunicator.attachListener(HandDesiredConfigurationMessage.class, new PacketConsumer<HandDesiredConfigurationMessage>()
      {
         public void receivedPacket(HandDesiredConfigurationMessage object)
         {
            sendHandCommand(object);
         }
      });

      handManagerPacketCommunicator.attachListener(ManualHandControlPacket.class, new PacketConsumer<ManualHandControlPacket>()
      {
         public void receivedPacket(ManualHandControlPacket object)
         {
            sendHandCommand(object);
         }
      });
   }

   protected void setupOutboundPacketListeners()
   {
      packetCommunicator.attachListener(HandJointAnglePacket.class, new PacketConsumer<HandJointAnglePacket>()
      {
         public void receivedPacket(HandJointAnglePacket object)
         {
            handManagerPacketCommunicator.send(object);
         }
      });
   }

   public static void main(String[] args)
   {
      JFrame frame = new JFrame();
      FlowLayout layout = new FlowLayout();
      frame.setLayout(layout);
      GridBagConstraints gc = new GridBagConstraints();
      frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);

      for (final RobotSide robotSide : RobotSide.values)
      {
         gc.gridx = 0;
         gc.gridy = 0;

         NetworkPorts port = robotSide == RobotSide.LEFT ? NetworkPorts.LEFT_HAND_MANAGER_PORT : NetworkPorts.RIGHT_HAND_MANAGER_PORT;
         final PacketCommunicator handModuleCommunicator = PacketCommunicator.createIntraprocessPacketCommunicator(port,
               new IHMCCommunicationKryoNetClassList());
         PacketDestination destination = robotSide == RobotSide.LEFT ? PacketDestination.LEFT_HAND : PacketDestination.RIGHT_HAND;
         try
         {
            handModuleCommunicator.connect();
         }
         catch (IOException e1)
         {
            e1.printStackTrace();
         }

         final RobotiqHandCommandManager commandManager = new RobotiqHandCommandManager(robotSide);

         final JComboBox<HandConfiguration> stateToSend = new JComboBox<HandConfiguration>(HandConfiguration.values());
         stateToSend.setSelectedItem(HandConfiguration.CALIBRATE);

         final JButton button = new JButton("Send");
         button.addActionListener(new ActionListener()
         {
            @Override
            public void actionPerformed(ActionEvent e)
            {
               handModuleCommunicator.send(new HandDesiredConfigurationMessage(robotSide, (HandConfiguration) (stateToSend.getSelectedItem())));
            }
         });
         
         JPanel panel = new JPanel(new GridBagLayout());
         panel.add(new JLabel(robotSide.name()), gc);
         
         gc.gridx++;
         panel.add(stateToSend, gc);

         gc.gridx++;
         panel.add(button, gc);

         //sliders
         final JSlider indexSlider = new JSlider(JSlider.HORIZONTAL, 1, 250, 1);
         final JSlider middleSlider = new JSlider(JSlider.HORIZONTAL, 1, 250, 1);
         final JSlider thumbSlider = new JSlider(JSlider.HORIZONTAL, 1, 250, 1);
         final JSlider spreadSlider = new JSlider(JSlider.HORIZONTAL, 1, 250, 0x8C);
         
         gc.gridx = 0;
         gc.gridy++;
         panel.add(new JLabel("Index"), gc);
         gc.gridx++;
         panel.add(indexSlider, gc);
         
         gc.gridx = 0;
         gc.gridy++;
         panel.add(new JLabel("Middle"), gc);
         gc.gridx++;
         panel.add(middleSlider, gc);
         
         gc.gridx = 0;
         gc.gridy++;
         panel.add(new JLabel("Thumb"), gc);
         gc.gridx++;
         panel.add(thumbSlider, gc);
         
         gc.gridx = 0;
         gc.gridy++;
         panel.add(new JLabel("Spread"), gc);
         gc.gridx++;
         panel.add(spreadSlider, gc);
         
         gc.gridx++;
         JButton sendSliderPositions = new JButton("Send");
         sendSliderPositions.addActionListener(new ActionListener()
         {
            @Override
            public void actionPerformed(ActionEvent e)
            {
               handModuleCommunicator.send(new ManualHandControlPacket(robotSide, indexSlider.getValue(), middleSlider.getValue(), thumbSlider.getValue(), spreadSlider.getValue(), 1));
            }
         });
         panel.add(sendSliderPositions, gc);
         
         frame.add(panel);
      }
      
      
      frame.pack();
      frame.setVisible(true);
   }
}
