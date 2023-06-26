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

public abstract class OpenSourceSpiralStairs implements MultiRobotTestInterface
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
      stepLength = 0.1;
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

   public void testStairs(TestInfo testInfo, boolean slow, boolean up, double swingDuration, double transferDuration, double heightOffset) throws Exception
   {
      testStairs(testInfo, slow, up, swingDuration, transferDuration, heightOffset, null);
   }

   public void testStairs(TestInfo testInfo,
                          boolean slow,
                          boolean up,
                          double swingDuration,
                          double transferDuration,
                          double heightOffset,
                          Consumer<FootstepDataListMessage> corruptor)
         throws Exception
   {
      DRCRobotModel robotModel = getRobotModel();
      double actualFootLength = robotModel.getWalkingControllerParameters().getSteppingParameters().getActualFootLength();
      double startX = up ? 0.0 : 1.2 + numberOfSteps * stepLength + 0.3;
      double startZ = up ? 0.0 : numberOfSteps * stepHeight;
   
      
      //Step parameters 
      double stepSizeX =1;
      double stepSizeY= 0.5;
      double stepSizeZ= 0.08;
   
       
      //Environment parameters;
      double spiralRadius=3;
      double stepsPerRadian=0.55;
      int noOfSteps=10;
      
      SpiralStairsEnvironment spiralStairsEnvironment = new SpiralStairsEnvironment(stepSizeX,stepSizeY,stepSizeZ,spiralRadius,stepsPerRadian,noOfSteps);
      
      
      SCS2AvatarTestingSimulationFactory simulationTestHelperFactory = SCS2AvatarTestingSimulationFactory.createDefaultTestSimulationFactory(robotModel, spiralStairsEnvironment, simulationTestingParameters);
      simulationTestHelperFactory.setStartingLocationOffset(new OffsetAndYawRobotInitialSetup(startX, 0, startZ));
      simulationTestHelperFactory.setUseImpulseBasedPhysicsEngine(useExperimentalPhysicsEngine);
      simulationTestHelper = simulationTestHelperFactory.createAvatarTestingSimulation();
      simulationTestHelper.start();
   
      simulationTestHelper.setCameraFocusPosition(startX, 0.0, 0.8 + startZ);
      simulationTestHelper.setCameraPosition(startX, -5.0, 0.8 + startZ);
   
      assertTrue(simulationTestHelper.simulateNow(0.5));
      
      boolean oneStairAtATime=false;
      double stanceWidth= 0.25;
      
      FootstepDataListMessage footsteps = spiralStairsEnvironment.generateStairsFootsteps(oneStairAtATime,
                                                                                          swingDuration,
                                                                                          transferDuration,
                                                                                          actualFootLength,
                                                                                          stanceWidth);

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

}
