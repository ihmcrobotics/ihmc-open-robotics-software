package us.ihmc.avatar.roughTerrainWalking;

import org.junit.After;
import org.junit.Before;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.communication.packets.PlanarRegionsListMessage;
import us.ihmc.communication.packets.RequestPlanarRegionsListMessage;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataListMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataMessage;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationConstructionSetTools.util.environments.planarRegionEnvironments.PlanarRegionEnvironmentInterface;
import us.ihmc.simulationToolkit.controllers.PushRobotController;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;

public abstract class AvatarPushRecoveryOverGapTest implements MultiRobotTestInterface
{
   private SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();
   private DRCSimulationTestHelper drcSimulationTestHelper;

   private PushRobotController pushRobotController;

   public void testNoPush() throws SimulationExceededMaximumTimeException
   {
      String className = getClass().getSimpleName();

      double platformLength = 1.0;
      double gapWidth = 0.10;

      GapPlanarRegionEnvironment environment = new GapPlanarRegionEnvironment(platformLength, gapWidth);

      DRCRobotModel robotModel = getRobotModel();
      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, robotModel);
      drcSimulationTestHelper.setTestEnvironment(environment);
      drcSimulationTestHelper.createSimulation(className);

      PlanarRegionsList planarRegionsList = environment.getPlanarRegionsList();
      PlanarRegionsListMessage planarRegionsListMessage = PlanarRegionMessageConverter.convertToPlanarRegionsListMessage(planarRegionsList);
      drcSimulationTestHelper.getControllerCommunicator().attachListener(RequestPlanarRegionsListMessage.class, new PacketConsumer<RequestPlanarRegionsListMessage>()
      {
         @Override
         public void receivedPacket(RequestPlanarRegionsListMessage packet)
         {
            drcSimulationTestHelper.send(planarRegionsListMessage);
         }
      });

      ThreadTools.sleep(1000);
      drcSimulationTestHelper.getSimulationConstructionSet().setCameraPosition(8.0, -8.0, 5.0);
      drcSimulationTestHelper.getSimulationConstructionSet().setCameraFix(1.5, 0.0, 0.8);

      WalkingControllerParameters walkingControllerParameters = robotModel.getWalkingControllerParameters();

      double swingTime = walkingControllerParameters.getDefaultSwingTime();
      double transferTime = walkingControllerParameters.getDefaultTransferTime();

      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5);

      FootstepDataListMessage footsteps = new FootstepDataListMessage(swingTime, transferTime);
      Point3D location = new Point3D(0.35, 0.15, 0.0);
      Quaternion orientation = new Quaternion(0.0, 0.0, 0.0, 1.0);
      footsteps.add(new FootstepDataMessage(RobotSide.LEFT, location, orientation));

      location = new Point3D(0.75, -0.15, 0.0);
      footsteps.add(new FootstepDataMessage(RobotSide.RIGHT, location, orientation));

      location = new Point3D(1.05, 0.15, 0.0);
      footsteps.add(new FootstepDataMessage(RobotSide.LEFT, location, orientation));
      location = new Point3D(1.05, -0.15, 0.0);
      footsteps.add(new FootstepDataMessage(RobotSide.RIGHT, location, orientation));

      drcSimulationTestHelper.send(footsteps);

      double simulationTime = (swingTime + transferTime) * 4 + 1.0;
      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(simulationTime);
   }

   @Before
   public void showMemoryUsageBeforeTest()
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());
   }

   @After
   public void destroySimulationAndRecycleMemory()
   {
      if (simulationTestingParameters.getKeepSCSUp())
      {
         ThreadTools.sleepForever();
      }

      // Do this here in case a test fails. That way the memory will be recycled.
      if (drcSimulationTestHelper != null)
      {
         drcSimulationTestHelper.destroySimulation();
         drcSimulationTestHelper = null;
      }

      if (pushRobotController != null)
      {
         pushRobotController = null;
      }

      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
      simulationTestingParameters = null;
   }

   private class GapPlanarRegionEnvironment extends PlanarRegionEnvironmentInterface
   {
      public GapPlanarRegionEnvironment(double platformLength, double gapSize)
      {
         generator.translate(0.0, 0.0, -0.01);
         generator.addCubeReferencedAtBottomMiddle(platformLength, 1.0, 0.01); // ground

         generator.translate(platformLength + gapSize, 0.0, 0.0);
         generator.addCubeReferencedAtBottomMiddle(platformLength, 1.0, 0.01); // ground
      }

   }
}
