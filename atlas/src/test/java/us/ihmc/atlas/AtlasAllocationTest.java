package us.ihmc.atlas;

import java.util.ArrayList;
import java.util.List;

import org.ejml.alg.dense.decomposition.chol.CholeskyDecompositionCommon_D64;
import org.ejml.alg.dense.decomposition.lu.LUDecompositionBase_D64;
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
import us.ihmc.commons.PrintTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FrameConvexPolygon2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.graphicsDescription.MeshDataGenerator;
import us.ihmc.robotics.allocations.AllocationTest;
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
      simulationTestingParameters.setKeepSCSUp(false);
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

   @ContinuousIntegrationTest(estimatedDuration = 300.0, categoriesOverride = {IntegrationCategory.SLOW})
   @Test(timeout = 600000)
   public void testForAllocationsWalking() throws SimulationExceededMaximumTimeException
   {
      double defaultSwingDuration = 0.5;
      double defaultTransferDuration = 0.1;

      int warmupSteps = 4;
      testHelper.send(createFootsteps(warmupSteps, defaultSwingDuration, defaultTransferDuration, 0.0, 0.0));
      testHelper.simulateAndBlockAndCatchExceptions(3.0);

      int steps = 4;
      FootstepDataListMessage footsteps = createFootsteps(steps, defaultSwingDuration, defaultTransferDuration, 0.0, 0.3);

      testInternal(() -> {
         try
         {
            testHelper.send(footsteps);
            testHelper.simulateAndBlockAndCatchExceptions(4.0);
         }
         catch (SimulationExceededMaximumTimeException e)
         {
            Assert.fail(e.getMessage());
         }
      });

      testHelper.assertRobotsRootJointIsInBoundingBox(new BoundingBox3D(0.9, -0.1, 0.0, 1.1, 0.1, 5.0));
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

      // These are places specific to the simulation and will not show up on the real robot.
      classesToIgnore.add(MirroredYoVariableRegistry.class);
      classesToIgnore.add(MeshDataGenerator.class);

      // TODO: fix these!
      classesToIgnore.add(StatusMessageOutputManager.class);

      return classesToIgnore;
   }

   @Override
   public List<String> getMethodsToIgnore()
   {
      List<String> methodsToIgnore = new ArrayList<>();

      // These methods are "safe" as they will only allocate to increase their capacity.
      methodsToIgnore.add(DenseMatrix64F.class.getName() + ".reshape");
      methodsToIgnore.add(TIntArrayList.class.getName() + ".ensureCapacity");
      methodsToIgnore.add(ConvexPolygon2D.class.getName() + ".setOrCreate");
      methodsToIgnore.add(FrameConvexPolygon2D.class.getName() + ".setOrCreate");
      methodsToIgnore.add(RecyclingArrayList.class.getName() + ".ensureCapacity");
      methodsToIgnore.add(LUDecompositionBase_D64.class.getName() + ".decomposeCommonInit");
      methodsToIgnore.add(CholeskyDecompositionCommon_D64.class.getName() + ".decompose");

      // Ignore the following methods as they are related to printouts.
      methodsToIgnore.add(Throwable.class.getName() + ".printStackTrace");
      methodsToIgnore.add(PrintTools.class.getName() + ".print");

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
