package us.ihmc.avatar.sensors.microphone;

import java.io.IOException;

import controller_msgs.msg.dds.DrillDetectionPacket;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.net.ConnectionStateListener;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.humanoidRobotics.kryo.IHMCCommunicationKryoNetClassList;

public class DrillDetectionProcess extends DrillDetectionThread implements ConnectionStateListener
{
   public static final long SEND_PERIOD_MS = 1000;

   private long lastSentPacketTimestamp = 0;
   private PacketCommunicator packetCommunicator = null;

   public DrillDetectionProcess()
   {
      super(new DrillDetectionCalibrationHelper());
   }

   @Override
   public void connected()
   {
      System.out.println("Connected to the network manager");
   }

   @Override
   public void disconnected()
   {
      System.out.println("Disconnected from the network manager");
   }

   @Override
   public void onDrillDetectionResult(DrillDetectionResult result)
   {
      if (!packetCommunicator.isConnected()) { return; }

      long now = System.currentTimeMillis();
      if (now - lastSentPacketTimestamp < SEND_PERIOD_MS) { return; }

      DrillDetectionPacket packet = new DrillDetectionPacket();
      packet.setDestination(PacketDestination.UI.ordinal());
      packet.setIsDrillOn(result.isOn);
//      packet.averageValues = result.averageValues;

      packetCommunicator.send(packet);
      lastSentPacketTimestamp = now;
   }

   public void execute() throws IOException
   {
      System.out.println("Connection to the network manager on the CPU0...");
      packetCommunicator = PacketCommunicator.createTCPPacketCommunicatorClient("10.7.3.100", NetworkPorts.DRILL_DETECTOR, new IHMCCommunicationKryoNetClassList());
      packetCommunicator.attachStateListener(this);
      packetCommunicator.connect();

      System.out.println("Starting the webcam lifecycle management...");
      start();
   }

   public void terminate() throws InterruptedException
   {
      System.out.println("Disconnection from the network manager...");
      packetCommunicator.disconnect();

      System.out.println("Shutting down the webcam thread...");
      shutdown();
      join();
   }

   public static void main(String[] args) throws IOException, InterruptedException
   {
      DrillDetectionProcess process = new DrillDetectionProcess();
      process.execute();

      while (process.isRunning()) { ThreadTools.sleep(500); }

      process.terminate();
   }
}