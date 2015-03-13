package us.ihmc.robotiq.control;

import java.awt.FlowLayout;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;

import javax.swing.JButton;
import javax.swing.JComboBox;
import javax.swing.JFrame;

import us.ihmc.communication.kryo.IHMCCommunicationKryoNetClassList;
import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.communication.packetCommunicator.KryoLocalPacketCommunicator;
import us.ihmc.communication.packetCommunicator.interfaces.PacketCommunicator;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.communication.packets.dataobjects.FingerState;
import us.ihmc.communication.packets.manipulation.FingerStatePacket;
import us.ihmc.communication.packets.manipulation.HandJointAnglePacket;
import us.ihmc.communication.packets.manipulation.ManualHandControlPacket;
import us.ihmc.darpaRoboticsChallenge.handControl.HandCommandManager;
import us.ihmc.utilities.robotSide.RobotSide;

public class RobotiqHandCommandManager extends HandCommandManager
{
   private final KryoLocalPacketCommunicator handManagerPacketCommunicator;
   
	public RobotiqHandCommandManager(RobotSide robotSide)
	{
	   super(RobotiqControlThread.class, robotSide);
	   
	   handManagerPacketCommunicator = new KryoLocalPacketCommunicator(new IHMCCommunicationKryoNetClassList(),
	         robotSide.equals(RobotSide.LEFT) ? PacketDestination.LEFT_HAND_MANAGER.ordinal() : PacketDestination.RIGHT_HAND_MANAGER.ordinal(), "RobotiqHandCommunicator");
	   
	   setupOutboundPacketListeners();
	   setupInboundPacketListeners();
	}

   protected void setupInboundPacketListeners()
   {
      handManagerPacketCommunicator.attachListener(FingerStatePacket.class, new PacketConsumer<FingerStatePacket>()
      {
         public void receivedPacket(FingerStatePacket object)
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

   @Override
   public PacketCommunicator getCommunicator()
   {
      return handManagerPacketCommunicator;
   }
   
   public static void main(String[] args)
   {
      final RobotiqHandCommandManager commandManager = new RobotiqHandCommandManager(RobotSide.LEFT);
      JFrame frame = new JFrame();
      frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
      
      final JComboBox<FingerState> stateToSend = new JComboBox<FingerState>(FingerState.values());
      stateToSend.setSelectedItem(FingerState.CALIBRATE);
      
      final JButton button = new JButton("Send");
      button.addActionListener(new ActionListener()
      {
         @Override
         public void actionPerformed(ActionEvent e)
         {
            commandManager.getCommunicator().send(new FingerStatePacket(RobotSide.LEFT,
                  (FingerState)(stateToSend.getSelectedItem())));
         }
      });
      frame.setLayout(new FlowLayout());
      
      frame.getContentPane().add(stateToSend);
      frame.getContentPane().add(button);
      frame.pack();
      frame.setVisible(true);
   }
}
