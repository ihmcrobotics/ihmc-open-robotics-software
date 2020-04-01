package us.ihmc.avatar.behaviorTests;

import static us.ihmc.avatar.roughTerrainWalking.AvatarBipedalFootstepPlannerEndToEndTest.*;

import java.io.IOException;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.networkProcessor.HumanoidNetworkProcessorParameters;
import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
import us.ihmc.commons.ContinuousIntegrationTools;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.HumanoidBehaviorType;
import us.ihmc.humanoidRobotics.kryo.IHMCCommunicationKryoNetClassList;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.geometry.PlanarRegionsListGenerator;
import us.ihmc.simulationConstructionSetTools.util.environments.CommonAvatarEnvironmentInterface;
import us.ihmc.simulationConstructionSetTools.util.environments.PlanarRegionsListDefinedEnvironment;
import us.ihmc.simulationConstructionSetTools.util.planarRegions.PlanarRegionsListExamples;
import us.ihmc.simulationconstructionset.FloatingJoint;
import us.ihmc.simulationconstructionset.util.ControllerFailureException;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;

public abstract class AvatarWalkOverTerrainBehaviorTest implements MultiRobotTestInterface
{
   private DRCSimulationTestHelper simulationTestHelper;
   private PlanarRegionsList cinderBlockField;

   @BeforeEach
   public void setUp()
   {
      PlanarRegionsListGenerator generator = new PlanarRegionsListGenerator();
      generator.translate(CINDER_BLOCK_START_X, CINDER_BLOCK_START_Y, 0.001);
      PlanarRegionsListExamples.generateCinderBlockField(generator,
                                                         CINDER_BLOCK_SIZE,
                                                         CINDER_BLOCK_HEIGHT,
                                                         CINDER_BLOCK_COURSE_WIDTH_X_IN_NUMBER_OF_BLOCKS,
                                                         CINDER_BLOCK_COURSE_LENGTH_Y_IN_NUMBER_OF_BLOCKS,
                                                         CINDER_BLOCK_HEIGHT_VARIATION,
                                                         -0.03,
                                                         0.6);
      cinderBlockField = generator.getPlanarRegionsList();

      SimulationTestingParameters parameters = SimulationTestingParameters.createFromSystemProperties();
      simulationTestHelper = new DRCSimulationTestHelper(parameters, getRobotModel(), createCommonAvatarInterface(cinderBlockField));
   }

   private static CommonAvatarEnvironmentInterface createCommonAvatarInterface(PlanarRegionsList planarRegionsList)
   {
      double allowablePenetrationThickness = 0.05;
      boolean generateGroundPlane = false;
      return new PlanarRegionsListDefinedEnvironment("testEnvironment", planarRegionsList, allowablePenetrationThickness, generateGroundPlane);
   }

   @Test
   public void testWalkOverCinderBlocks() throws IOException, BlockingSimulationRunner.SimulationExceededMaximumTimeException, ControllerFailureException
   {
      HumanoidNetworkProcessorParameters networkModuleParameters = new HumanoidNetworkProcessorParameters();
      networkModuleParameters.setUseBehaviorModule(true);
      networkModuleParameters.setUseFootstepPlanningToolboxModule(true);
      simulationTestHelper.setNetworkProcessorParameters(networkModuleParameters);

      simulationTestHelper.createSimulation(getClass().getSimpleName(), !ContinuousIntegrationTools.isRunningOnContinuousIntegrationServer(), true);

      PacketCommunicator behaviorCommunicator = PacketCommunicator.createIntraprocessPacketCommunicator(NetworkPorts.BEHAVIOUR_MODULE_PORT,
                                                                                                        new IHMCCommunicationKryoNetClassList());
      behaviorCommunicator.connect();

      PacketCommunicator controllerCommunicator = PacketCommunicator.createIntraprocessPacketCommunicator(NetworkPorts.CONTROLLER_PORT,
                                                                                                          new IHMCCommunicationKryoNetClassList());
      controllerCommunicator.connect();

      double courseLength = CINDER_BLOCK_COURSE_WIDTH_X_IN_NUMBER_OF_BLOCKS * CINDER_BLOCK_SIZE + CINDER_BLOCK_FIELD_PLATFORM_LENGTH;
      Point3D goalPosition = new Point3D(courseLength, 0.0, 0.0);

      behaviorCommunicator.send(PlanarRegionMessageConverter.convertToPlanarRegionsListMessage(cinderBlockField));
      behaviorCommunicator.send(HumanoidMessageTools.createWalkOverTerrainGoalPacket(goalPosition, new Quaternion()));
      behaviorCommunicator.send(HumanoidMessageTools.createHumanoidBehaviorTypePacket(HumanoidBehaviorType.WALK_OVER_TERRAIN));

      FloatingJoint pelvisJoint = simulationTestHelper.getRobot().getRootJoint();
      Point3D pelvisPosition = new Point3D();
      while (pelvisPosition.distanceXY(goalPosition) > 0.1)
      {
         simulationTestHelper.simulateAndBlock(2.0);
         pelvisJoint.getPosition(pelvisPosition);
      }
   }
}
