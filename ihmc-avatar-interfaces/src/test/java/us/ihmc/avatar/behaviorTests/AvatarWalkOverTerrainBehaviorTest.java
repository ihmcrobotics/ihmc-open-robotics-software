package us.ihmc.avatar.behaviorTests;

import org.junit.Before;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.networkProcessor.DRCNetworkModuleParameters;
import us.ihmc.avatar.simulationStarter.DRCSimulationStarter;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.communication.packets.PlanarRegionsListMessage;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.footstepPlanning.polygonSnapping.PlanarRegionsListExamples;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.HumanoidBehaviorType;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.HumanoidBehaviorTypePacket;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.WalkOverTerrainGoalPacket;
import us.ihmc.humanoidRobotics.kryo.IHMCCommunicationKryoNetClassList;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.simulationConstructionSetTools.util.environments.CommonAvatarEnvironmentInterface;
import us.ihmc.simulationConstructionSetTools.util.environments.PlanarRegionsListDefinedEnvironment;
import us.ihmc.simulationconstructionset.FloatingJoint;
import us.ihmc.simulationconstructionset.FloatingRootJointRobot;

import java.io.IOException;

import static us.ihmc.avatar.roughTerrainWalking.AvatarBipedalFootstepPlannerEndToEndTest.*;

public abstract class AvatarWalkOverTerrainBehaviorTest implements MultiRobotTestInterface
{
   private DRCSimulationStarter simulationStarter;
   private PlanarRegionsList cinderBlockField;

   @Before
   public void setUp()
   {
      cinderBlockField = PlanarRegionsListExamples.generateCinderBlockField(CINDER_BLOCK_START_X, CINDER_BLOCK_START_Y, CINDER_BLOCK_SIZE, CINDER_BLOCK_HEIGHT,
                                                                            CINDER_BLOCK_COURSE_WIDTH_X_IN_NUMBER_OF_BLOCKS,
                                                                            CINDER_BLOCK_COURSE_LENGTH_Y_IN_NUMBER_OF_BLOCKS, CINDER_BLOCK_HEIGHT_VARIATION, - 0.03);


      simulationStarter = new DRCSimulationStarter(getRobotModel(), createCommonAvatarInterface(cinderBlockField));
   }

   private static CommonAvatarEnvironmentInterface createCommonAvatarInterface(PlanarRegionsList planarRegionsList)
   {
      double allowablePenetrationThickness = 0.05;
      boolean generateGroundPlane = false;
      return new PlanarRegionsListDefinedEnvironment("testEnvironment", planarRegionsList,
                                                     allowablePenetrationThickness, generateGroundPlane);
   }

   public void testWalkOverCinderBlocks() throws IOException
   {
      DRCNetworkModuleParameters networkModuleParameters = new DRCNetworkModuleParameters();
      networkModuleParameters.enableLocalControllerCommunicator(true);
      networkModuleParameters.enableBehaviorModule(true);
      networkModuleParameters.enableFootstepPlanningToolbox(true);
      simulationStarter.startSimulation(networkModuleParameters, true);

      PacketCommunicator behaviorCommunicator = PacketCommunicator.createIntraprocessPacketCommunicator(NetworkPorts.BEHAVIOUR_MODULE_PORT, new IHMCCommunicationKryoNetClassList());
      behaviorCommunicator.connect();

      PacketCommunicator controllerCommunicator = PacketCommunicator.createIntraprocessPacketCommunicator(NetworkPorts.CONTROLLER_PORT, new IHMCCommunicationKryoNetClassList());
      controllerCommunicator.connect();

      double courseLength = CINDER_BLOCK_COURSE_WIDTH_X_IN_NUMBER_OF_BLOCKS * CINDER_BLOCK_SIZE + CINDER_BLOCK_FIELD_PLATFORM_LENGTH;
      Point3D goalPosition = new Point3D(courseLength, 0.0, 0.0);

      behaviorCommunicator.send(PlanarRegionMessageConverter.convertToPlanarRegionsListMessage(cinderBlockField));
      behaviorCommunicator.send(new HumanoidBehaviorTypePacket(HumanoidBehaviorType.WAlK_OVER_TERRAIN));
      behaviorCommunicator.send(new WalkOverTerrainGoalPacket(goalPosition, new Quaternion()));

      FloatingJoint pelvisJoint = simulationStarter.getSDFRobot().getRootJoint();
      Point3D pelvisPosition = new Point3D();
      while(pelvisPosition.distanceXY(goalPosition) > 0.1)
      {
         ThreadTools.sleep(2000);
         pelvisJoint.getPosition(pelvisPosition);
      }
   }
}
