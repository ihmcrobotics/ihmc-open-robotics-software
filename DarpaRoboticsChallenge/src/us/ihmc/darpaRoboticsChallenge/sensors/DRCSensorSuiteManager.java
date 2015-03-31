package us.ihmc.darpaRoboticsChallenge.sensors;

import java.io.IOException;
import java.net.URI;

import us.ihmc.communication.packetCommunicator.interfaces.PacketCommunicator;

public interface DRCSensorSuiteManager
{
   public void initializeSimulatedSensors(PacketCommunicator scsSensorPacketCommunicator);

   public void initializePhysicalSensors(URI sensorURI);
   
   public void connect() throws IOException;

}
