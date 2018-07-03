package us.ihmc.atlas;

import java.util.List;
import java.util.Random;

import org.ejml.alg.dense.decomposition.bidiagonal.BidiagonalDecompositionRow_D64;
import org.ejml.alg.dense.decomposition.chol.CholeskyDecompositionCommon_D64;
import org.ejml.alg.dense.decomposition.lu.LUDecompositionBase_D64;
import org.ejml.data.DenseMatrix64F;
import org.junit.After;
import org.junit.Assert;
import org.junit.Before;
import org.junit.Test;

import controller_msgs.msg.dds.ArmTrajectoryMessage;
import controller_msgs.msg.dds.ChestTrajectoryMessage;
import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepDataMessage;
import controller_msgs.msg.dds.PelvisTrajectoryMessage;
import controller_msgs.msg.dds.SE3TrajectoryMessage;
import controller_msgs.msg.dds.SE3TrajectoryPointMessage;
import gnu.trove.list.array.TIntArrayList;
import us.ihmc.avatar.DRCEstimatorThread;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
import us.ihmc.commons.PrintTools;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.commons.allocations.AllocationProfiler;
import us.ihmc.commons.allocations.AllocationRecord;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FrameConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.graphicsDescription.MeshDataGenerator;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.jMonkeyEngineToolkit.jme.JMEGraphicsObject;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.MovingReferenceFrame;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.ScrewTestTools;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationConstructionSetTools.util.environments.FlatGroundEnvironment;
import us.ihmc.simulationconstructionset.dataBuffer.MirroredYoVariableRegistry;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;
import us.ihmc.wholeBodyController.DRCControllerThread;

public class AtlasAllocationTest
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();
   static
   {
      simulationTestingParameters.setCreateGUI(false);
      simulationTestingParameters.setKeepSCSUp(false);
   }

   private DRCSimulationTestHelper testHelper;
   private AllocationProfiler allocationProfiler = new AllocationProfiler();
   
   @Before
   public void before() throws SimulationExceededMaximumTimeException
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
      
      AllocationProfiler.checkInstrumentation();
      
      allocationProfiler.includeAllocationsInsideClass(DRCControllerThread.class.getName()); // only testing these classes!
      allocationProfiler.includeAllocationsInsideClass(DRCEstimatorThread.class.getName()); // only testing these classes!
      allocationProfiler.excludeAllocationsInsideClass(MirroredYoVariableRegistry.class.getName());
      allocationProfiler.excludeAllocationsInsideClass(MeshDataGenerator.class.getName());
      allocationProfiler.excludeAllocationsInsideClass(JMEGraphicsObject.class.getName());
      allocationProfiler.excludeAllocationsInsideClass(StatusMessageOutputManager.class.getName()); // fix this

      // These methods are "safe" as they will only allocate to increase their capacity.
      allocationProfiler.excludeAllocationsInsideMethod(DenseMatrix64F.class.getName() + ".reshape");
      allocationProfiler.excludeAllocationsInsideMethod(TIntArrayList.class.getName() + ".ensureCapacity");
      allocationProfiler.excludeAllocationsInsideMethod(ConvexPolygon2D.class.getName() + ".setOrCreate");
      allocationProfiler.excludeAllocationsInsideMethod(FrameConvexPolygon2D.class.getName() + ".setOrCreate");
      allocationProfiler.excludeAllocationsInsideMethod(RecyclingArrayList.class.getName() + ".ensureCapacity");
      allocationProfiler.excludeAllocationsInsideMethod(LUDecompositionBase_D64.class.getName() + ".decomposeCommonInit");
      allocationProfiler.excludeAllocationsInsideMethod(CholeskyDecompositionCommon_D64.class.getName() + ".decompose");
      allocationProfiler.excludeAllocationsInsideMethod(BidiagonalDecompositionRow_D64.class.getName() + ".init");

      // Ignore the following methods as they are related to printouts.
      allocationProfiler.excludeAllocationsInsideMethod(Throwable.class.getName() + ".printStackTrace");
      allocationProfiler.excludeAllocationsInsideMethod(PrintTools.class.getName() + ".print");
      
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      setup();
   }

   @ContinuousIntegrationTest(estimatedDuration = 300.0, categoriesOverride = {IntegrationCategory.SLOW})
   @Test(timeout = 600000)
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
      testHelper.publishToController(createFootsteps(warmupSteps, defaultSwingDuration, defaultTransferDuration, 0.0, 0.0));
      testHelper.simulateAndBlockAndCatchExceptions(3.0);

      int steps = 4;
      FootstepDataListMessage footsteps = createFootsteps(steps, defaultSwingDuration, defaultTransferDuration, 0.0, 0.3);

      testInternal(() -> {
         try
         {
            testHelper.publishToController(footsteps);
            testHelper.simulateAndBlockAndCatchExceptions(4.0);
         }
         catch (SimulationExceededMaximumTimeException e)
         {
            Assert.fail(e.getMessage());
         }
      });

      testHelper.assertRobotsRootJointIsInBoundingBox(new BoundingBox3D(0.9, -0.1, 0.0, 1.1, 0.1, 5.0));
   }

   @ContinuousIntegrationTest(estimatedDuration = 300.0, categoriesOverride = {IntegrationCategory.SLOW})
   @Test(timeout = 600000)
   public void testForAllocationsDuringPelvisMotion() throws SimulationExceededMaximumTimeException
   {
      Random random = new Random(42884L);
      double minMax = 0.05;
      double timeStep = 0.1;
      double duration = 1.0;
      PelvisTrajectoryMessage message = createPelvisTrajectory(random, minMax, timeStep, duration);

      testInternal(() -> {
         try
         {
            testHelper.publishToController(message);
            testHelper.simulateAndBlockAndCatchExceptions(duration + 0.25);
         }
         catch (SimulationExceededMaximumTimeException e)
         {
            Assert.fail(e.getMessage());
         }
      });
   }

   @ContinuousIntegrationTest(estimatedDuration = 300.0, categoriesOverride = {IntegrationCategory.SLOW})
   @Test(timeout = 600000)
   public void testForAllocationsWithPelvisUserControl() throws SimulationExceededMaximumTimeException
   {
      Random random = new Random(4281284L);
      double minMax = 0.05;
      double timeStep = 0.1;
      double duration = 1.0;
      PelvisTrajectoryMessage message = createPelvisTrajectory(random, minMax, timeStep, duration);

      message.setEnableUserPelvisControl(true);
      message.setEnableUserPelvisControlDuringWalking(true);

      testInternal(() -> {
         try
         {
            testHelper.publishToController(message);
            testHelper.simulateAndBlockAndCatchExceptions(duration + 0.25);
         }
         catch (SimulationExceededMaximumTimeException e)
         {
            Assert.fail(e.getMessage());
         }
      });
   }

   @ContinuousIntegrationTest(estimatedDuration = 300.0, categoriesOverride = {IntegrationCategory.SLOW})
   @Test(timeout = 600000)
   public void testForAllocationsDuringArmMotion() throws SimulationExceededMaximumTimeException
   {
      Random random = new Random(4281284L);
      double duration = 0.3;
      ArmTrajectoryMessage message = createArmTrajectory(random, duration);

      testInternal(() -> {
         try
         {
            testHelper.publishToController(message);
            testHelper.simulateAndBlockAndCatchExceptions(duration + 0.25);
         }
         catch (SimulationExceededMaximumTimeException e)
         {
            Assert.fail(e.getMessage());
         }
      });
   }

   @ContinuousIntegrationTest(estimatedDuration = 300.0, categoriesOverride = {IntegrationCategory.SLOW})
   @Test(timeout = 600000)
   public void testForAllocationsDuringChestMotion() throws SimulationExceededMaximumTimeException
   {
      Random random = new Random(4281284L);
      double duration = 0.3;
      ChestTrajectoryMessage message = createChestTrajectory(random, duration);

      testInternal(() -> {
         try
         {
            testHelper.publishToController(message);
            testHelper.simulateAndBlockAndCatchExceptions(duration + 0.25);
         }
         catch (SimulationExceededMaximumTimeException e)
         {
            Assert.fail(e.getMessage());
         }
      });
   }

   private ChestTrajectoryMessage createChestTrajectory(Random random, double duration)
   {
      FullHumanoidRobotModel fullRobotModel = testHelper.getControllerFullRobotModel();
      HumanoidReferenceFrames humanoidReferenceFrames = new HumanoidReferenceFrames(fullRobotModel);
      ReferenceFrame pelvisZUpFrame = humanoidReferenceFrames.getPelvisZUpFrame();
      humanoidReferenceFrames.updateFrames();

      RigidBody pelvis = fullRobotModel.getPelvis();
      RigidBody chest = fullRobotModel.getChest();
      OneDoFJoint[] spineClone = ScrewTools.cloneOneDoFJointPath(pelvis, chest);
      ScrewTestTools.setRandomPositionsWithinJointLimits(spineClone, random);
      RigidBody chestClone = spineClone[spineClone.length - 1].getSuccessor();
      FrameQuaternion desiredRandomChestOrientation = new FrameQuaternion(chestClone.getBodyFixedFrame());
      desiredRandomChestOrientation.changeFrame(ReferenceFrame.getWorldFrame());

      Quaternion desiredOrientation = new Quaternion(desiredRandomChestOrientation);
      ChestTrajectoryMessage message = HumanoidMessageTools.createChestTrajectoryMessage(duration, desiredOrientation, ReferenceFrame.getWorldFrame(),
                                                                                         pelvisZUpFrame);
      return message;
   }

   private ArmTrajectoryMessage createArmTrajectory(Random random, double duration)
   {
      FullHumanoidRobotModel fullRobotModel = testHelper.getControllerFullRobotModel();
      RigidBody chest = fullRobotModel.getChest();
      RigidBody hand = fullRobotModel.getHand(RobotSide.LEFT);
      OneDoFJoint[] armJoints = ScrewTools.createOneDoFJointPath(chest, hand);
      double[] desiredJointPositions = new double[armJoints.length];
      for (int i = 0; i < armJoints.length; i++)
      {
         OneDoFJoint joint = armJoints[i];
         desiredJointPositions[i] = RandomNumbers.nextDouble(random, joint.getJointLimitLower(), joint.getJointLimitUpper());
      }
      ArmTrajectoryMessage message = HumanoidMessageTools.createArmTrajectoryMessage(RobotSide.LEFT, duration, desiredJointPositions);
      return message;
   }

   private PelvisTrajectoryMessage createPelvisTrajectory(Random random, double minMax, double timeStep, double duration)
   {
      RigidBody pelvis = testHelper.getControllerFullRobotModel().getPelvis();
      MovingReferenceFrame pelvisFrame = pelvis.getParentJoint().getFrameAfterJoint();
      SE3TrajectoryMessage trajectory = new SE3TrajectoryMessage();
      for (double time = 0.0; time <= duration; time += timeStep)
      {
         FramePoint3D position = new FramePoint3D(pelvisFrame, EuclidCoreRandomTools.nextPoint3D(random, minMax));
         FrameQuaternion orientation = new FrameQuaternion(pelvisFrame, EuclidCoreRandomTools.nextQuaternion(random, minMax));
         FrameVector3D linearVelocity = new FrameVector3D(pelvisFrame, EuclidCoreRandomTools.nextPoint3D(random, minMax));
         FrameVector3D angularVelocity = new FrameVector3D(pelvisFrame, EuclidCoreRandomTools.nextPoint3D(random, minMax));
         position.changeFrame(ReferenceFrame.getWorldFrame());
         orientation.changeFrame(ReferenceFrame.getWorldFrame());
         linearVelocity.changeFrame(ReferenceFrame.getWorldFrame());
         angularVelocity.changeFrame(ReferenceFrame.getWorldFrame());
         SE3TrajectoryPointMessage trajectoryPoint = HumanoidMessageTools.createSE3TrajectoryPointMessage(time, position, orientation, linearVelocity,
                                                                                                          angularVelocity);
         trajectory.getTaskspaceTrajectoryPoints().add().set(trajectoryPoint);
      }

      PelvisTrajectoryMessage message = new PelvisTrajectoryMessage();
      message.getSe3Trajectory().set(trajectory);
      return message;
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

   private void testInternal(Runnable whatToTestFor)
   {
      List<AllocationRecord> allocations = allocationProfiler.recordAllocations(whatToTestFor);

      if (!allocations.isEmpty())
      {
         allocations.forEach(allocation -> System.out.println(allocation));
         Assert.fail("Found allocations in the controller.");
      }
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
