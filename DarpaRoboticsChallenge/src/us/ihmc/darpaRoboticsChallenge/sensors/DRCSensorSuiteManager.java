package us.ihmc.darpaRoboticsChallenge.sensors;

import java.net.URI;

import us.ihmc.communication.packetCommunicator.interfaces.PacketCommunicator;

public interface DRCSensorSuiteManager
{
   public void initializeSimulatedSensors(PacketCommunicator scsSensorPacketCommunicator);

   public void initializePhysicalSensors(URI sensorURI);
   
   public PacketCommunicator getProcessedSensorsCommunicator();

}
