package us.ihmc.avatar.obstacleCourseTests;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.TestInfo;

import controller_msgs.FootstepDataListMessage;
import controller_msgs.FootstepDataMessage;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.testTools.scs2.SCS2AvatarTestingSimulation;
import us.ihmc.avatar.testTools.scs2.SCS2AvatarTestingSimulationFactory;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.simulationConstructionSetTools.util.environments.CommonAvatarEnvironmentInterface;
import us.ihmc.simulationConstructionSetTools.util.ground.CombinedTerrainObject3D;
import us.ihmc.simulationconstructionset.util.ground.TerrainObject3D;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;

import static org.junit.jupiter.api.Assertions.*;

public abstract class AvatarCustomSteppingStonesTest implements MultiRobotTestInterface
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();
   static
   {
      simulationTestingParameters.setRunMultiThreaded(false);
   }

   private SCS2AvatarTestingSimulation simulationTestHelper;
   private boolean useExperimentalPhysicsEngine = false;

   private double swingTime = 0.6;
   private double transferTime = 0.25;

   private int numberOfSteps = 1;
   private boolean squareUpOnLastStep = true;
   private RobotSide firstStepSide = RobotSide.LEFT;

   public abstract double getStepLength();

   @BeforeEach
   public void showMemoryUsageBeforeTest()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
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

   public void changeWalkingParameters(double transferTime, double swingTime)
   {
      this.transferTime = transferTime;
      this.swingTime = swingTime;
   }

   public void setNumberOfSteps(int numberOfSteps)
   {
      this.numberOfSteps = numberOfSteps;
   }

   public void setTakeSquareUpStep(boolean squareUp)
   {
      this.squareUpOnLastStep = squareUp;
   }

   public void testTakingStepOneFootAtATime(TestInfo testInfo, double stepHeight)
   {
      setTakeSquareUpStep(false);

      StepsEnvironment steps = new StepsEnvironment();
      double startYPosition = 0.0;
      double width = 0.6;
      double depth = 0.15;

      createStepsEnvironment(steps, getStepLength(), startYPosition, width, depth, stepHeight, 0.0);
      setupTest(testInfo, steps);

      RobotSide nextSuportSide = firstStepSide;
      for (int i = 0; i < numberOfSteps; i++)
      {
         if (i == numberOfSteps - 1)
            walkForward(i * getStepLength(), 1, i * stepHeight, nextSuportSide, 0.0);
         else
            walkForward((i + 1) * getStepLength(), 1, (i + 1) * stepHeight, nextSuportSide, 0.0);

         assertTrue(simulationTestHelper.simulateNow(0.1));
         nextSuportSide = nextSuportSide.getOppositeSide();
      }
   }

   public void testTakingStep(TestInfo testInfo, double stepHeight)
   {
      testTakingStep(testInfo, stepHeight, 0.0);
   }

   public void testTakingStep(TestInfo testInfo, double stepHeight, double initialStepYoffset)
   {
      testTakingStep(testInfo, stepHeight, initialStepYoffset, 0.0);
   }
   public void testTakingStep(TestInfo testInfo, double stepHeight, double initialStepYoffset, double heightOffset)
   {
      StepsEnvironment steps = new StepsEnvironment();
      double startYPosition = 0.0;
      double width = 0.6;
      double depth = 0.15;

      createStepsEnvironment(steps, getStepLength(), startYPosition, width, depth, stepHeight, initialStepYoffset);
      setupTest(testInfo, steps);

      if (!MathTools.epsilonEquals(heightOffset, 0.0, 1e-2))
         sendPelvisHeightOffset(heightOffset);

      walkForward(getStepLength(), numberOfSteps, stepHeight, initialStepYoffset);
      assertTrue(simulationTestHelper.simulateNow(1.5));
   }

   private void sendPelvisHeightOffset(double heightOffset)
   {
      FullHumanoidRobotModel fullRobotModel = simulationTestHelper.getControllerFullRobotModel();
      FramePoint3D pelvisPosition = new FramePoint3D(fullRobotModel.getPelvis().getBodyFixedFrame());
      pelvisPosition.changeFrame(ReferenceFrame.getWorldFrame());
      pelvisPosition.add(0.0, 0.0, heightOffset);
      double desiredHeight = pelvisPosition.getZ();
      simulationTestHelper.publishToController(HumanoidMessageTools.createPelvisHeightTrajectoryMessage(0.5, desiredHeight));
   }


   private void setupTest(TestInfo testInfo, CommonAvatarEnvironmentInterface environment)
   {
      DRCRobotModel robotModel = getRobotModel();
      SCS2AvatarTestingSimulationFactory simulationTestHelperFactory = SCS2AvatarTestingSimulationFactory.createDefaultTestSimulationFactory(robotModel,
                                                                                                                                             environment,
                                                                                                                                             simulationTestingParameters);
      simulationTestHelperFactory.setUseImpulseBasedPhysicsEngine(useExperimentalPhysicsEngine);
      simulationTestHelper = simulationTestHelperFactory.createAvatarTestingSimulation();
      simulationTestHelper.start();
      simulationTestHelper.setCamera(new Point3D(0.6, 0.0, 0.6), new Point3D(10.0, 3.0, 3.0));

      assertTrue(simulationTestHelper.simulateNow(0.5));
   }

   private void createStepsEnvironment(StepsEnvironment stepsEnvironment,
                                       double stepLength,
                                       double startYPosition,
                                       double width,
                                       double depth,
                                       double stepHeight,
                                       double firstStepYOffset)
   {
      if (firstStepYOffset != 0.0)
      {
         stepsEnvironment.addStep(0.0, firstStepYOffset - width, width, depth, stepHeight);
      }

      for (int i = 0; i < numberOfSteps; i++)
      {
         stepsEnvironment.addStep((i + 1) * stepLength, startYPosition, width, depth, (i + 1) * stepHeight);
      }
   }

   private void walkForward(double stepLength, int steps, double stepHeight, double firstStepYOffset)
   {
      walkForward(stepLength, steps, stepHeight, firstStepSide, firstStepYOffset);
   }

   private void walkForward(double stepLength, int steps, double stepHeight, RobotSide nextSupportStepSide, double firstStepYOffset)
   {
      double stepWidth = 0.14;

      FootstepDataListMessage footsteps = HumanoidMessageTools.createFootstepDataListMessage(swingTime, transferTime);
      RobotSide robotSide = nextSupportStepSide;
      double footstepX, footstepY;
      for (int i = 1; i <= steps; i++)
      {
         if (firstStepYOffset != 0.0)
         {
            robotSide = robotSide.getOppositeSide();
            footstepY = robotSide == RobotSide.LEFT ? (stepWidth + firstStepYOffset) : -stepWidth + firstStepYOffset;
            Point3D location = new Point3D(0.0, footstepY, stepHeight);
            Quaternion orientation = new Quaternion();
            FootstepDataMessage footstepData = HumanoidMessageTools.createFootstepDataMessage(robotSide, location, orientation);
            footsteps.getFootstepDataList().add().set(footstepData);
         }

         if (!squareUpOnLastStep && i == steps)
            break;

         robotSide = robotSide.getOppositeSide();
         footstepY = robotSide == RobotSide.LEFT ? stepWidth + firstStepYOffset : -stepWidth + firstStepYOffset;
         footstepX = stepLength * i;
         Point3D location = new Point3D(footstepX, footstepY, 0.0);
         location.setZ(i * stepHeight);
         Quaternion orientation = new Quaternion();
         FootstepDataMessage footstepData = HumanoidMessageTools.createFootstepDataMessage(robotSide, location, orientation);
         footsteps.getFootstepDataList().add().set(footstepData);
      }
      // closing step
      robotSide = robotSide.getOppositeSide();
      footstepY = robotSide == RobotSide.LEFT ? stepWidth + firstStepYOffset : -stepWidth + firstStepYOffset;
      footstepX = stepLength * steps;
      Point3D location = new Point3D(footstepX, footstepY, 0.0);
      location.setZ(steps * stepHeight);
      Quaternion orientation = new Quaternion();
      FootstepDataMessage footstepData = HumanoidMessageTools.createFootstepDataMessage(robotSide, location, orientation);
      footsteps.getFootstepDataList().add().set(footstepData);

      simulationTestHelper.publishToController(footsteps);
      assertTrue(simulationTestHelper.simulateNow(footsteps.getFootstepDataList().size() * 1.1 * (swingTime + transferTime)));
   }

   private static class StepsEnvironment implements CommonAvatarEnvironmentInterface
   {
      private final CombinedTerrainObject3D terrainObject = new CombinedTerrainObject3D("Steps");

      public StepsEnvironment()
      {
         double length = 0.3;
         terrainObject.addBox(-length / 2.0, -length, length / 2.0, length, -0.1);
      }

      public void addStep(double startXPosition, double startYPosition, double width, double depth, double height)
      {
         terrainObject.addBox(startXPosition - depth, startYPosition - width, startXPosition + depth, startYPosition + width, height, height - 0.1);
      }

      @Override
      public TerrainObject3D getTerrainObject3D()
      {
         return terrainObject;
      }
   }

}
