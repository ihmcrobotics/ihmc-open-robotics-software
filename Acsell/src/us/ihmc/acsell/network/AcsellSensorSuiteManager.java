package us.ihmc.acsell.network;

import java.io.IOException;
import java.net.URI;

import us.ihmc.SdfLoader.SDFBaseFullRobotModel;
import us.ihmc.communication.kryo.IHMCCommunicationKryoNetClassList;
import us.ihmc.communication.net.ObjectCommunicator;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.darpaRoboticsChallenge.sensors.DRCSensorSuiteManager;

public class AcsellSensorSuiteManager implements DRCSensorSuiteManager
{
   private final PacketCommunicator sensorSuitePacketCommunicator = PacketCommunicator.createIntraprocessPacketCommunicator(NetworkPorts.SENSOR_MANAGER,
         new IHMCCommunicationKryoNetClassList());
   
   public AcsellSensorSuiteManager(SDFBaseFullRobotModel sdfFullRobotModel, boolean useSimulatedSensors)
   {
   }
   
   @Override
   public void initializeSimulatedSensors(ObjectCommunicator packetCommunicator)
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
