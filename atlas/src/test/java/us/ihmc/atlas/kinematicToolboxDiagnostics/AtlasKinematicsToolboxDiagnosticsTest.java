package us.ihmc.atlas.kinematicToolboxDiagnostics;

import java.io.IOException;

import org.apache.commons.lang3.mutable.MutableInt;
import org.junit.Test;

import controller_msgs.msg.dds.RobotConfigurationData;
import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.KinematicToolboxDiagnosticEnvironment;
import us.ihmc.commons.PrintTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.humanoidRobotics.kryo.IHMCCommunicationKryoNetClassList;

public class AtlasKinematicsToolboxDiagnosticsTest
{
   @ContinuousIntegrationTest(estimatedDuration = 10.0)
   @Test(timeout = 30000)
   public void testReceiveRobotConfigurationData() throws IOException
   {
      AtlasRobotVersion atlasVersion = AtlasRobotVersion.ATLAS_UNPLUGGED_V5_DUAL_ROBOTIQ;
      RobotTarget target = RobotTarget.SCS;
      boolean headless = true;
      AtlasRobotModel atlasModel = new AtlasRobotModel(atlasVersion, target, headless);

      new KinematicToolboxDiagnosticEnvironment(atlasModel);

      PacketCommunicator controllerPacketCommunicator = PacketCommunicator.createIntraprocessPacketCommunicator(NetworkPorts.CONTROLLER_PORT,
                                                                                                                new IHMCCommunicationKryoNetClassList());
      
      controllerPacketCommunicator.connect();
      
      ThreadTools.sleep(10); // wait a sec to connect
      
      MutableInt receivedMessages = new MutableInt();
      controllerPacketCommunicator.attachListener(RobotConfigurationData.class, packet -> {
         PrintTools.info("Received: " + receivedMessages.intValue() + " joints: " + packet.getJointAngles().size());
         receivedMessages.increment();
      });

      
      while (receivedMessages.intValue() < 10);
   }
}
