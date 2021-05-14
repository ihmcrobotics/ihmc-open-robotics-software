package us.ihmc.avatar.obstacleCourseTests;

import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepDataMessage;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.TestInfo;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.simulationConstructionSetTools.util.environments.CommonAvatarEnvironmentInterface;
import us.ihmc.simulationConstructionSetTools.util.ground.CombinedTerrainObject3D;
import us.ihmc.simulationconstructionset.util.ground.TerrainObject3D;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;

import static us.ihmc.robotics.Assert.assertTrue;

public abstract class AvatarCustomSteppingStonesTest implements MultiRobotTestInterface
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();
   static
   {
      simulationTestingParameters.setRunMultiThreaded(false);
   }

   private DRCSimulationTestHelper drcSimulationTestHelper;
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

   public void testTakingStepOneFootAtATime(TestInfo testInfo, double stepHeight) throws SimulationExceededMaximumTimeException
   {
      setTakeSquareUpStep(false);

      StepsEnvironment steps = new StepsEnvironment();
      double startYPosition = 0.0;
      double width = 0.6;
      double depth = 0.15;

      createStepsEnvironment(steps, getStepLength(), startYPosition, width, depth, stepHeight, 0.0);
      setupTest(testInfo, steps);

      RobotSide nextSuportSide = firstStepSide;
      for(int i = 0; i < numberOfSteps; i++)
      {
         if(i == numberOfSteps-1)
            walkForward(i*getStepLength(), 1, i*stepHeight, nextSuportSide, 0.0);
         else
            walkForward((i+1)*getStepLength(), 1, (i+1)*stepHeight, nextSuportSide, 0.0);

         assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.1));
         nextSuportSide = nextSuportSide.getOppositeSide();
      }
   }

   public void testTakingStep(TestInfo testInfo, double stepHeight) throws SimulationExceededMaximumTimeException
   {
      testTakingStep(testInfo, stepHeight, 0.0);
   }

   public void testTakingStep(TestInfo testInfo, double stepHeight, double initialStepYoffset) throws SimulationExceededMaximumTimeException
   {
      StepsEnvironment steps = new StepsEnvironment();
      double startYPosition = 0.0;
      double width = 0.6;
      double depth = 0.15;

      createStepsEnvironment(steps, getStepLength(), startYPosition, width, depth, stepHeight, initialStepYoffset);
      setupTest(testInfo, steps);

      walkForward(getStepLength(), numberOfSteps, stepHeight, initialStepYoffset);
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(3.0));
   }

   private void setupTest(TestInfo testInfo, CommonAvatarEnvironmentInterface environment) throws SimulationExceededMaximumTimeException
   {
      DRCRobotModel robotModel = getRobotModel();
      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, robotModel, environment);
      drcSimulationTestHelper.getSCSInitialSetup().setUseExperimentalPhysicsEngine(useExperimentalPhysicsEngine);
      drcSimulationTestHelper.createSimulation(getClass().getSimpleName()+ " " + testInfo.getTestMethod().get().getName());
      drcSimulationTestHelper.setupCameraForUnitTest(new Point3D(0.6, 0.0, 0.6), new Point3D(10.0, 3.0, 3.0));

      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0));
   }

   private void createStepsEnvironment(StepsEnvironment stepsEnvironment, double stepLength, double startYPosition, double width, double depth, double stepHeight, double firstStepYOffset)
   {
      if(firstStepYOffset != 0.0)
      {
         stepsEnvironment.addStep(0.0, firstStepYOffset - width, width, depth, stepHeight);
      }

      for(int i = 0; i < numberOfSteps; i++)
      {
         stepsEnvironment.addStep((i+1)*stepLength, startYPosition, width, depth, (i+1)*stepHeight);
      }
   }

   private void walkForward(double stepLength, int steps, double stepHeight, double firstStepYOffset) throws SimulationExceededMaximumTimeException
   {
      walkForward(stepLength, steps, stepHeight, firstStepSide, firstStepYOffset);
   }

   private void walkForward(double stepLength, int steps, double stepHeight, RobotSide nextSupportStepSide, double firstStepYOffset) throws SimulationExceededMaximumTimeException
   {
      double stepWidth = 0.14;

      ReferenceFrame pelvisFrame = drcSimulationTestHelper.getSDFFullRobotModel().getPelvis().getBodyFixedFrame();

      FootstepDataListMessage footsteps = HumanoidMessageTools.createFootstepDataListMessage(swingTime, transferTime);
      RobotSide robotSide = nextSupportStepSide;
      double footstepX, footstepY;
      for (int i = 1; i <= steps; i++)
      {
         if(firstStepYOffset != 0.0)
         {
            robotSide = robotSide.getOppositeSide();
            footstepY = robotSide == RobotSide.LEFT ? (stepWidth+firstStepYOffset) : -stepWidth+firstStepYOffset;
            FramePoint3D location = new FramePoint3D(pelvisFrame, 0.0, footstepY, stepHeight);
            location.changeFrame(ReferenceFrame.getWorldFrame());
            Quaternion orientation = new Quaternion();
            FootstepDataMessage footstepData = HumanoidMessageTools.createFootstepDataMessage(robotSide, location, orientation);
            footsteps.getFootstepDataList().add().set(footstepData);
         }

         if(!squareUpOnLastStep && i == steps)
            break;

         robotSide = robotSide.getOppositeSide();
         footstepY = robotSide == RobotSide.LEFT ? stepWidth+firstStepYOffset : -stepWidth+firstStepYOffset;
         footstepX = stepLength * i;
         FramePoint3D location = new FramePoint3D(pelvisFrame, footstepX, footstepY, 0.0);
         location.changeFrame(ReferenceFrame.getWorldFrame());
         location.setZ(i*stepHeight);
         Quaternion orientation = new Quaternion();
         FootstepDataMessage footstepData = HumanoidMessageTools.createFootstepDataMessage(robotSide, location, orientation);
         footsteps.getFootstepDataList().add().set(footstepData);
      }
      // closing step
      robotSide = robotSide.getOppositeSide();
      footstepY = robotSide == RobotSide.LEFT ? stepWidth+firstStepYOffset : -stepWidth+firstStepYOffset;
      footstepX = stepLength * steps;
      FramePoint3D location = new FramePoint3D(pelvisFrame, footstepX, footstepY, 0.0);
      location.changeFrame(ReferenceFrame.getWorldFrame());
      location.setZ(steps*stepHeight);
      Quaternion orientation = new Quaternion();
      FootstepDataMessage footstepData = HumanoidMessageTools.createFootstepDataMessage(robotSide, location, orientation);
      footsteps.getFootstepDataList().add().set(footstepData);

      drcSimulationTestHelper.publishToController(footsteps);
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(numberOfSteps*1.5));
   }

   private static class StepsEnvironment implements CommonAvatarEnvironmentInterface
   {
      private final CombinedTerrainObject3D terrainObject = new CombinedTerrainObject3D("Steps");

      public StepsEnvironment()
      {
         double length = 0.3;
         terrainObject.addBox(-length/2.0, -length, length/2.0, length, -0.1);
      }

      public void addStep(double startXPosition, double startYPosition, double width, double depth, double height)
      {
         terrainObject.addBox(startXPosition-depth, startYPosition-width, startXPosition + depth, startYPosition + width, height, height-0.1);
      }

      @Override
      public TerrainObject3D getTerrainObject3D() {
         return terrainObject;
      }
   }

}
