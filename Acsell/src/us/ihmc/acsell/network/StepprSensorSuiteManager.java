package us.ihmc.acsell.network;

import java.io.IOException;
import java.net.URI;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.communication.kryo.IHMCCommunicationKryoNetClassList;
import us.ihmc.communication.packetCommunicator.PacketCommunicatorMock;
import us.ihmc.communication.packetCommunicator.interfaces.PacketCommunicator;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.darpaRoboticsChallenge.sensors.DRCSensorSuiteManager;

public class StepprSensorSuiteManager implements DRCSensorSuiteManager
{
   private final PacketCommunicatorMock sensorSuitePacketCommunicator = PacketCommunicatorMock.createIntraprocessPacketCommunicator(NetworkPorts.SENSOR_MANAGER,
         new IHMCCommunicationKryoNetClassList());
   
   public StepprSensorSuiteManager(SDFFullRobotModel sdfFullRobotModel, boolean useSimulatedSensors)
   {
   }
   
   @Override
   public void initializeSimulatedSensors(PacketCommunicator packetCommunicator)
   {
   }

   @Override
   public void initializePhysicalSensors(URI sensorURI)
   {
   }

   @Override
   public void connect() throws IOException
   {
      sensorSuitePacketCommunicator.connect();
   }


}
