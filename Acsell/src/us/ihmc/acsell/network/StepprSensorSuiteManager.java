package us.ihmc.acsell.network;

import java.net.URI;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.communication.kryo.IHMCCommunicationKryoNetClassList;
import us.ihmc.communication.packetCommunicator.KryoLocalPacketCommunicator;
import us.ihmc.communication.packetCommunicator.interfaces.PacketCommunicator;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.darpaRoboticsChallenge.sensors.DRCSensorSuiteManager;

public class StepprSensorSuiteManager implements DRCSensorSuiteManager
{
   private final KryoLocalPacketCommunicator sensorSuitePacketCommunicator = new KryoLocalPacketCommunicator(new IHMCCommunicationKryoNetClassList(), PacketDestination.SENSOR_MANAGER.ordinal(), "STEPPR_SENSOR_MANAGER");
   
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
   public PacketCommunicator getProcessedSensorsCommunicator()
   {
      return sensorSuitePacketCommunicator;
   }
}
