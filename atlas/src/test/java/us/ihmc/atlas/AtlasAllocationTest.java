package us.ihmc.atlas;

import java.util.ArrayList;
import java.util.List;

import org.ejml.data.DenseMatrix64F;
import org.junit.After;
import org.junit.Assert;
import org.junit.Before;
import org.junit.Test;

import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepDataMessage;
import gnu.trove.list.array.TIntArrayList;
import us.ihmc.avatar.DRCEstimatorThread;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
import us.ihmc.commonWalkingControlModules.captureRegion.OneStepCaptureRegionCalculator;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.graphicsDescription.MeshDataGenerator;
import us.ihmc.robotics.allocations.AllocationTest;
import us.ihmc.robotics.geometry.ConvexPolygonTools;
import us.ihmc.robotics.lists.RecyclingArrayList;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationConstructionSetTools.util.environments.FlatGroundEnvironment;
import us.ihmc.simulationconstructionset.dataBuffer.MirroredYoVariableRegistry;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.wholeBodyController.DRCControllerThread;

public class AtlasAllocationTest implements AllocationTest
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();
   static
   {
      simulationTestingParameters.setCreateGUI(false);
   }

   private DRCSimulationTestHelper testHelper;

   @ContinuousIntegrationTest(estimatedDuration = 53.3, categoriesOverride = {IntegrationCategory.SLOW})
   @Test(timeout = 270000)
   public void testForAllocationsStanding() throws SimulationExceededMaximumTimeException
   {
      testInternal(() -> {
         try
         {
            testHelper.simulateAndBlockAndCatchExceptions(0.25);
         }
         catch (SimulationExceededMaximumTimeException e)
         {
            Assert.fail(e.getMessage());
         }
      });
   }

   @ContinuousIntegrationTest(estimatedDuration = 82.3, categoriesOverride = {IntegrationCategory.SLOW})
   @Test(timeout = 410000)
   public void testForAllocationsWalking() throws SimulationExceededMaximumTimeException
   {
      double defaultSwingDuration = 0.5;
      double defaultTransferDuration = 0.1;

      int warmupSteps = 2;
      testHelper.send(createFootsteps(warmupSteps, defaultSwingDuration, defaultTransferDuration, 0.0, 0.0));
      testHelper.simulateAndBlockAndCatchExceptions(defaultTransferDuration + warmupSteps * (defaultSwingDuration * defaultTransferDuration) + 0.25);

      int steps = 4;
      FootstepDataListMessage footsteps = createFootsteps(steps, defaultSwingDuration, defaultTransferDuration, 0.0, 0.3);

      testInternal(() -> {
         try
         {
            testHelper.send(footsteps);
            testHelper.simulateAndBlockAndCatchExceptions(defaultTransferDuration + steps * (defaultSwingDuration * defaultTransferDuration) + 0.25);
         }
         catch (SimulationExceededMaximumTimeException e)
         {
            Assert.fail(e.getMessage());
         }
      });
   }

   private FootstepDataListMessage createFootsteps(int steps, double defaultSwingDuration, double defaultTransferDuration, double xLocation, double stepLength)
   {
      RobotSide robotSide = RobotSide.LEFT;
      FootstepDataListMessage footstepListMessage = new FootstepDataListMessage();
      footstepListMessage.setDefaultSwingDuration(defaultSwingDuration);
      footstepListMessage.setDefaultTransferDuration(defaultTransferDuration);
      footstepListMessage.setFinalTransferDuration(defaultTransferDuration);
      for (int i = 0; i < steps; i++)
      {
         xLocation += stepLength;
         FootstepDataMessage footstepMessage = footstepListMessage.getFootstepDataList().add();
         footstepMessage.getLocation().set(new Point3D(xLocation, robotSide.negateIfRightSide(0.15), 0.0));
         footstepMessage.getOrientation().set(new Quaternion());
         footstepMessage.setRobotSide(robotSide.toByte());
         robotSide = robotSide.getOppositeSide();
      }
      return footstepListMessage;
   }

   private void setup() throws SimulationExceededMaximumTimeException
   {
      DRCRobotModel robotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, RobotTarget.SCS, false);
      testHelper = new DRCSimulationTestHelper(simulationTestingParameters, robotModel, new FlatGroundEnvironment());
      testHelper.createSimulation(getClass().getSimpleName());
      testHelper.simulateAndBlockAndCatchExceptions(0.25);
   }

   private void testInternal(Runnable whatToTestFor) throws SimulationExceededMaximumTimeException
   {
      List<Throwable> allocations = runAndCollectAllocations(whatToTestFor);

      if (!allocations.isEmpty())
      {
         allocations.forEach(allocation -> allocation.printStackTrace());
         Assert.fail("Found allocations in the controller.");
      }
   }

   @Override
   public List<Class<?>> getClassesOfInterest()
   {
      List<Class<?>> classesOfInterest = new ArrayList<>();
      classesOfInterest.add(DRCControllerThread.class);
      classesOfInterest.add(DRCEstimatorThread.class);
      return classesOfInterest;
   }

   @Override
   public List<Class<?>> getClassesToIgnore()
   {
      List<Class<?>> classesToIgnore = new ArrayList<>();
      classesToIgnore.add(MirroredYoVariableRegistry.class);
      classesToIgnore.add(MeshDataGenerator.class);

      // TODO: fix these.
      classesToIgnore.add(StatusMessageOutputManager.class);
      classesToIgnore.add(ConvexPolygonTools.class);
      classesToIgnore.add(OneStepCaptureRegionCalculator.class);

      return classesToIgnore;
   }

   @Override
   public List<String> getMethodsToIgnore()
   {
      List<String> methodsToIgnore = new ArrayList<>();
      methodsToIgnore.add(DenseMatrix64F.class.getName() + ".reshape");
      methodsToIgnore.add(TIntArrayList.class.getName() + ".ensureCapacity");
      methodsToIgnore.add(ConvexPolygon2D.class.getName() + ".setOrCreate");
      methodsToIgnore.add(RecyclingArrayList.class.getName() + ".ensureCapacity");
      return methodsToIgnore;
   }

   @Before
   public void before() throws SimulationExceededMaximumTimeException
   {
      AllocationTest.checkInstrumentation();

      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      setup();
   }

   @After
   public void destroySimulationAndRecycleMemory()
   {
      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());

      if (simulationTestingParameters.getKeepSCSUp())
      {
         ThreadTools.sleepForever();
      }

      if (testHelper != null)
      {
         testHelper.destroySimulation();
         testHelper = null;
      }
   }
}
