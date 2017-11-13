package us.ihmc.avatar.behaviorTests;

import static org.junit.Assert.assertTrue;

import org.junit.After;
import org.junit.Before;

import us.ihmc.avatar.DRCObstacleCourseStartingLocation;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.testTools.DRCBehaviorTestHelper;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.humanoidBehaviors.behaviors.roughTerrain.PushAndWalkBehavior;
import us.ihmc.humanoidBehaviors.communication.CommunicationBridge;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.simulationConstructionSetTools.util.environments.FlatGroundEnvironment;
import us.ihmc.simulationToolkit.controllers.PushRobotController;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;
import us.ihmc.commons.thread.ThreadTools;

public abstract class AvatarPushAndWalkBehaviorTest implements MultiRobotTestInterface
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();
   private DRCBehaviorTestHelper drcBehaviorTestHelper;
   private PushRobotController pushRobotController;

   public void testBehavior() throws SimulationExceededMaximumTimeException
   {
      double z = 0.3;
      DRCRobotModel robotModel = getRobotModel();

      FlatGroundEnvironment flatGround = new FlatGroundEnvironment();
      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.DEFAULT_BUT_ALMOST_PI;
      drcBehaviorTestHelper = new DRCBehaviorTestHelper(flatGround, "DRCSimpleFlatGroundScriptTest", selectedLocation, simulationTestingParameters, robotModel);
      FullHumanoidRobotModel fullRobotModel = drcBehaviorTestHelper.getControllerFullRobotModel();
      pushRobotController = new PushRobotController(drcBehaviorTestHelper.getRobot(), fullRobotModel.getChest().getParentJoint().getName(), new Vector3D(0, 0, z));
      SimulationConstructionSet scs = drcBehaviorTestHelper.getSimulationConstructionSet();
      scs.addYoGraphic(pushRobotController.getForceVisualizer());

      HumanoidReferenceFrames referenceFrames = new HumanoidReferenceFrames(fullRobotModel);
      WalkingControllerParameters walkingControllerParameters = robotModel.getWalkingControllerParameters();
      CommunicationBridge communicationBridge = drcBehaviorTestHelper.getBehaviorCommunicationBridge();
      PushAndWalkBehavior pushAndWalkBehavior = new PushAndWalkBehavior(communicationBridge, referenceFrames, fullRobotModel, walkingControllerParameters, null);
      scs.addYoVariableRegistry(pushAndWalkBehavior.getYoVariableRegistry());

      drcBehaviorTestHelper.setupCameraForUnitTest(new Point3D(0.0, 0.0, 1.0), new Point3D(10.0, 10.0, 3.0));
      ThreadTools.sleep(1000);
      assertTrue(drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(0.5));
      drcBehaviorTestHelper.dispatchBehavior(pushAndWalkBehavior);

      double totalMass = fullRobotModel.getTotalMass();
      double force = totalMass * 0.4;
      double duration = 0.5;
      Vector3D direction = new Vector3D();

      for (int i = 0; i < 5; i++)
      {
         direction.set(1.0, 0.0, 0.0);
         pushRobotController.applyForce(direction, force, duration);
         assertTrue(drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(duration + 2.0));
      }

      for (int i = 0; i < 5; i++)
      {
         direction.set(0.0, 1.0, 0.0);
         pushRobotController.applyForce(direction, force, duration);
         assertTrue(drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(duration + 2.0));
      }

      for (int i = 0; i < 5; i++)
      {
         direction.set(1.0, 1.0, 0.0);
         pushRobotController.applyForce(direction, force, duration);
         assertTrue(drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(duration + 2.0));
      }
   }

   @Before
   public void showMemoryUsageBeforeTest()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
   }

   @After
   public void destroySimulationAndRecycleMemory()
   {
      if (simulationTestingParameters.getKeepSCSUp())
      {
         ThreadTools.sleepForever();
      }

      // Do this here in case a test fails. That way the memory will be recycled.
      if (drcBehaviorTestHelper != null)
      {
         drcBehaviorTestHelper.closeAndDispose();
         drcBehaviorTestHelper = null;
      }

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }

}
