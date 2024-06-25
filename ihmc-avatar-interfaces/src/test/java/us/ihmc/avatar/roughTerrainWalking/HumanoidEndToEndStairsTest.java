package us.ihmc.avatar.roughTerrainWalking;

import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.Random;
import java.util.function.Consumer;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.TestInfo;

import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepDataMessage;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.initialSetup.OffsetAndYawRobotInitialSetup;
import us.ihmc.avatar.testTools.EndToEndTestTools;
import us.ihmc.avatar.testTools.scs2.SCS2AvatarTestingSimulation;
import us.ihmc.avatar.testTools.scs2.SCS2AvatarTestingSimulationFactory;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.simulationConstructionSetTools.util.environments.CommonAvatarEnvironmentInterface;
import us.ihmc.simulationConstructionSetTools.util.ground.CombinedTerrainObject3D;
import us.ihmc.simulationconstructionset.util.ground.TerrainObject3D;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;

public abstract class HumanoidEndToEndStairsTest implements MultiRobotTestInterface
{
   private static final boolean EXPORT_TORQUE_SPEED_DATA = false;
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();

   private SCS2AvatarTestingSimulation simulationTestHelper;
   private int numberOfSteps = 6;
   private double stepHeight = 9.25 * 0.0254;
   private double stepLength = 0.32;
   private boolean useExperimentalPhysicsEngine = false;

   @BeforeEach
   public void showMemoryUsageBeforeTest()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");

      numberOfSteps = 6;
      stepHeight = 9.25 * 0.0254;
      stepLength = 0.32;
      useExperimentalPhysicsEngine = false;
   }

   @AfterEach
   public void destroySimulationAndRecycleMemory()
   {
      if (simulationTestHelper != null)
      {
         simulationTestHelper.finishTest();
         simulationTestHelper = null;
      }

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }

   public void setNumberOfSteps(int numberOfSteps)
   {
      this.numberOfSteps = numberOfSteps;
   }

   public void setStepHeight(double stepHeight)
   {
      this.stepHeight = stepHeight;
   }

   public void setStepLength(double stepLength)
   {
      this.stepLength = stepLength;
   }

   public void setUseExperimentalPhysicsEngine(boolean useExperimentalPhysicsEngine)
   {
      this.useExperimentalPhysicsEngine = useExperimentalPhysicsEngine;
   }

   public void testStairs(TestInfo testInfo, boolean squareUpSteps, boolean goingUp, double swingDuration, double transferDuration, double heightOffset) throws Exception
   {
      testStairs(testInfo, squareUpSteps, goingUp, swingDuration, transferDuration, heightOffset, null);
   }

   public void testStairs(TestInfo testInfo,
                          boolean squareUpSteps,
                          boolean goingUp,
                          double swingDuration,
                          double transferDuration,
                          double heightOffset,
                          Consumer<FootstepDataListMessage> corruptor)
         throws Exception
   {
      DRCRobotModel robotModel = getRobotModel();
      double actualFootLength = robotModel.getWalkingControllerParameters().getSteppingParameters().getActualFootLength();
      double startX = goingUp ? 0.0 : 1.2 + numberOfSteps * stepLength + 0.3;
      double startZ = goingUp ? 0.0 : numberOfSteps * stepHeight;

      StairsEnvironment environment = new StairsEnvironment(numberOfSteps, stepHeight, stepLength, true);
      SCS2AvatarTestingSimulationFactory simulationTestHelperFactory = SCS2AvatarTestingSimulationFactory.createDefaultTestSimulationFactory(robotModel, environment, simulationTestingParameters);
      simulationTestHelperFactory.setStartingLocationOffset(new OffsetAndYawRobotInitialSetup(startX, 0, startZ));
      simulationTestHelperFactory.setUseImpulseBasedPhysicsEngine(useExperimentalPhysicsEngine);
      simulationTestHelper = simulationTestHelperFactory.createAvatarTestingSimulation();
      simulationTestHelper.start();

      simulationTestHelper.setCameraFocusPosition(startX, 0.0, 0.8 + startZ);
      simulationTestHelper.setCameraPosition(startX, -5.0, 0.8 + startZ);

      assertTrue(simulationTestHelper.simulateNow(0.5));

      FootstepDataListMessage footsteps = EndToEndTestTools.generateStairsFootsteps(squareUpSteps, goingUp, stepHeight, stepLength, 0.25, numberOfSteps);
      if (goingUp)
         translate(footsteps, new Vector3D(0.6 - 0.045 - actualFootLength / 2.0, 0.0, 0.0));
      else
         translate(footsteps, new Vector3D(1.8 - 0.045 - actualFootLength / 2.0 + (numberOfSteps + 1) * stepLength, 0.0, startZ));
      EndToEndTestTools.setStepDurations(footsteps, swingDuration, transferDuration);
      if (corruptor != null)
         corruptor.accept(footsteps);
      publishHeightOffset(heightOffset);

      simulationTestHelper.setInPoint();

      publishFootstepsAndSimulate(robotModel, footsteps);

      if (EXPORT_TORQUE_SPEED_DATA)
      {
         EndToEndTestTools.exportTorqueSpeedCurves(simulationTestHelper,
                                                   EndToEndTestTools.getDataOutputFolder(robotModel.getSimpleRobotName(), null),
                                                   testInfo.getTestMethod().get().getName());
      }
   }

   private void publishHeightOffset(double heightOffset) throws Exception
   {
      if (!Double.isFinite(heightOffset) || EuclidCoreTools.epsilonEquals(0.0, heightOffset, 1.0e-3))
         return;
      MovingReferenceFrame rootJointFrame = simulationTestHelper.getControllerFullRobotModel().getRootJoint().getFrameAfterJoint();
      double z = rootJointFrame.getTransformToRoot().getTranslationZ();
      simulationTestHelper.publishToController(HumanoidMessageTools.createPelvisHeightTrajectoryMessage(0.5, z + heightOffset));
      assertTrue(simulationTestHelper.simulateNow(0.5));
   }

   private void publishFootstepsAndSimulate(DRCRobotModel robotModel, FootstepDataListMessage footsteps) throws Exception
   {
      double walkingDuration = EndToEndTestTools.computeWalkingDuration(footsteps, robotModel.getWalkingControllerParameters());
      simulationTestHelper.publishToController(footsteps);
      assertTrue(simulationTestHelper.simulateNow(1.1 * walkingDuration));
   }

   private static FootstepDataListMessage translate(FootstepDataListMessage message, Tuple3DReadOnly translation)
   {
      for (int i = 0; i < message.getFootstepDataList().size(); i++)
      {
         message.getFootstepDataList().get(i).getLocation().add(translation);
      }

      return message;
   }

   public static Consumer<FootstepDataListMessage> createFootstepCorruptor(Random random,
                                                                           double rangeX,
                                                                           double rangeY,
                                                                           double rangeZ,
                                                                           double rangeYaw,
                                                                           double rangePitch,
                                                                           double rangeRoll)
   {
      return footstepDataList ->
      {
         for (int i = 0; i < footstepDataList.getFootstepDataList().size(); i++)
         {
            FootstepDataMessage footstep = footstepDataList.getFootstepDataList().get(i);
            footstep.getLocation().addX(EuclidCoreRandomTools.nextDouble(random, rangeX));
            footstep.getLocation().addY(EuclidCoreRandomTools.nextDouble(random, rangeY));
            footstep.getLocation().addZ(EuclidCoreRandomTools.nextDouble(random, rangeZ));
            YawPitchRoll yawPitchRoll = new YawPitchRoll(footstep.getOrientation());
            yawPitchRoll.addYaw(EuclidCoreRandomTools.nextDouble(random, rangeYaw));
            yawPitchRoll.addPitch(EuclidCoreRandomTools.nextDouble(random, rangePitch));
            yawPitchRoll.addRoll(EuclidCoreRandomTools.nextDouble(random, rangeRoll));
            footstep.getOrientation().set(yawPitchRoll);
         }
      };
   }

   private static class StairsEnvironment implements CommonAvatarEnvironmentInterface
   {
      private final CombinedTerrainObject3D terrainObject = new CombinedTerrainObject3D("Stairs");

      public StairsEnvironment(int numberOfSteps, double stepHeight, double stepLength, boolean includeDown)
      {
         this(numberOfSteps, stepHeight, stepLength, 1.75, includeDown);
      }

      public StairsEnvironment(int numberOfSteps, double stepHeight, double stepLength, double stairsWidth, boolean includeDown)
      {
         double startingBlockLength = 1.2;
         double xStart = -0.5 * startingBlockLength;
         double xEnd = 0.5 * startingBlockLength;
         double yStart = -0.5 * stairsWidth;
         double yEnd = 0.5 * stairsWidth;
         double zStart = -0.10;
         double zEnd = 0.0;
         terrainObject.addBox(xStart, yStart, xEnd, yEnd, zStart, zEnd);

         for (int i = 0; i < numberOfSteps - 1; i++)
         {
            xStart = xEnd;
            xEnd += stepLength;
            zEnd += stepHeight;
            terrainObject.addBox(xStart, yStart, xEnd + 3.0e-2, yEnd, zStart, zEnd);
         }

         xStart = xEnd;
         xEnd += startingBlockLength + 2 * stepLength;
         zEnd = numberOfSteps * stepHeight;
         terrainObject.addBox(xStart, yStart, xEnd, yEnd, zStart, zEnd);

         if (includeDown)
         {
            for (int i = 0; i < numberOfSteps - 1; i++)
            {
               xStart = xEnd;
               xEnd += stepLength;
               zEnd -= stepHeight;
               terrainObject.addBox(xStart - 3.0e-2, yStart, xEnd, yEnd, zStart, zEnd);
            }

            xStart = xEnd - 3.0e-2;
            xEnd += startingBlockLength + stepLength;
            zEnd = 0.0;
            terrainObject.addBox(xStart, yStart, xEnd, yEnd, zStart, zEnd);
         }
      }

      @Override
      public TerrainObject3D getTerrainObject3D()
      {
         return terrainObject;
      }
   }
}
