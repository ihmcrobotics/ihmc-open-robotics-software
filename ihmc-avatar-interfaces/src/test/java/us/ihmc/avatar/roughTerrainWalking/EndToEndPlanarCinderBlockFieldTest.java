package us.ihmc.avatar.roughTerrainWalking;

import static us.ihmc.robotics.Assert.assertTrue;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

import controller_msgs.msg.dds.ContinuousStepGeneratorInputMessage;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.testTools.scs2.SCS2AvatarTestingSimulation;
import us.ihmc.avatar.testTools.scs2.SCS2AvatarTestingSimulationFactory;
import us.ihmc.commonWalkingControlModules.controllers.Updatable;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationConstructionSetTools.util.environments.CommonAvatarEnvironmentInterface;
import us.ihmc.simulationConstructionSetTools.util.environments.planarRegionEnvironments.CinderBlockFieldPlanarRegionEnvironment;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;

public abstract class EndToEndPlanarCinderBlockFieldTest implements MultiRobotTestInterface
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();

   private boolean useImpulseBasedPhysicsEngine = false;
   private SCS2AvatarTestingSimulation simulationTestHelper;

   public double getSwingHeight()
   {
      return getRobotModel().getWalkingControllerParameters().getSwingTrajectoryParameters().getDefaultSwingHeight();
   }

   @BeforeEach
   public void showMemoryUsageBeforeTest()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
      useImpulseBasedPhysicsEngine = false;
   }

   @AfterEach
   public void destroySimulationAndRecycleMemory()
   {
      // Do this here in case a test fails. That way the memory will be recycled.
      if (simulationTestHelper != null)
      {
         simulationTestHelper.finishTest();
         simulationTestHelper = null;
      }

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }


   private void setupSimulation(CommonAvatarEnvironmentInterface environment)
   {
      SCS2AvatarTestingSimulationFactory simulationFactory = new SCS2AvatarTestingSimulationFactory(getRobotModel(), environment);
      simulationFactory.setDefaultHighLevelHumanoidControllerFactory();
      simulationFactory.setShowGUI(simulationTestingParameters.getCreateGUI());
      simulationFactory.setRunMultiThreaded(simulationTestingParameters.getRunMultiThreaded());
      simulationFactory.setUseImpulseBasedPhysicsEngine(useImpulseBasedPhysicsEngine);
      simulationTestHelper = simulationFactory.createAvatarTestingSimulation();
   }

   @Test
   @Tag("fast")
   public void testWalkingOverCinderBlockField() throws Exception
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      CinderBlockFieldPlanarRegionEnvironment cinderBlockFieldEnvironment = new CinderBlockFieldPlanarRegionEnvironment();

      setupSimulation(cinderBlockFieldEnvironment);
      simulationTestHelper.setKeepSCSUp(true);
      simulationTestHelper.getHighLevelHumanoidControllerFactory().addUpdatable(new Updatable()
      {
         int counter = 0;
         @Override
         public void update(double time)
         {
            if (counter > 10)
            {
               simulationTestHelper.publishToController(PlanarRegionMessageConverter.convertToPlanarRegionsListMessage(cinderBlockFieldEnvironment.getPlanarRegionsList()));
               counter = 0;
            }
            counter++;
         }
      });
      simulationTestHelper.start();

      ThreadTools.sleep(1000);
      boolean success = simulationTestHelper.simulateNow(0.5);
      assertTrue(success);

      ContinuousStepGeneratorInputMessage stepGeneratorMessage = new ContinuousStepGeneratorInputMessage();
      stepGeneratorMessage.setForwardVelocity(0.25);
      stepGeneratorMessage.setWalk(true);

      simulationTestHelper.publishToController(stepGeneratorMessage);

      success = simulationTestHelper.simulateNow(20.0);
      assertTrue(success);


//      simulationTestHelper.assertRobotsRootJointIsInBoundingBox(new BoundingBox3D(min, max));

   }


}
