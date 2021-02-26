package us.ihmc.avatar.obstacleCourseTests;

import static us.ihmc.robotics.Assert.assertTrue;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepDataMessage;
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
import us.ihmc.simulationConstructionSetTools.util.environments.FlatGroundEnvironment;
import us.ihmc.simulationConstructionSetTools.util.ground.CombinedTerrainObject3D;
import us.ihmc.simulationconstructionset.util.ground.TerrainObject3D;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public abstract class AvatarToeOffTest implements MultiRobotTestInterface
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();
   static
   {
      simulationTestingParameters.setRunMultiThreaded(false);
   }

   private DRCSimulationTestHelper drcSimulationTestHelper;
   private boolean useExperimentalPhysicsEngine = false;

   private boolean isAnkleAtJointLimit = false;
   private double anklePitchLowerLimit;
   private double anklePitchUpperLimit;
   private double swingTime = 0.6;
   private double transferTime = 0.25;
   private double stepHeight = 0.0;

   public abstract double getStepLength();

   public abstract double getMaxStepLength();

   public abstract int getNumberOfSteps();

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

   public void setUseExperimentalPhysicsEngine(boolean useExperimentalPhysicsEngine)
   {
      this.useExperimentalPhysicsEngine = useExperimentalPhysicsEngine;
   }

   public void setStepHeight(double height)
   {
      stepHeight = height;
   }

   @Test
   public void testShortSteps(TestInfo testInfo) throws SimulationExceededMaximumTimeException
   {
      FlatGroundEnvironment flatGround = new FlatGroundEnvironment();
      setupTest(testInfo, flatGround);
      setYoVariablesToDoToeOffInSS(0.0, 0.3, 0.1);

      walkForward(getStepLength(), getNumberOfSteps());
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(4.0));
   }

   @Test
   public void testToeOffTakingStepAndCheckingAnkleLimits(TestInfo testInfo) throws SimulationExceededMaximumTimeException
   {
      StepsEnvironment steps = new StepsEnvironment();
      double startYPosition = 0.0;
      double width = 0.5;
      double depth = 0.5;

      steps.addStep(getStepLength(), startYPosition, width, depth, stepHeight);
      setupTest(testInfo, steps);
      anklePitchLowerLimit = drcSimulationTestHelper.getControllerFullRobotModel().getLegJoint(RobotSide.LEFT, LegJointName.ANKLE_PITCH).getJointLimitLower();
      anklePitchUpperLimit = drcSimulationTestHelper.getControllerFullRobotModel().getLegJoint(RobotSide.LEFT, LegJointName.ANKLE_PITCH).getJointLimitUpper();

      walkForward(getStepLength(), 1, 0.0, stepHeight);
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0));
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

   private void walkForward(double stepLength, int steps) throws SimulationExceededMaximumTimeException
   {
      walkForward(stepLength, steps, 0.0, 0.0);
   }

   private void walkForward(double stepLength, int steps, double initialXPosition, double stepHeight) throws SimulationExceededMaximumTimeException
   {
      double stepWidth = 0.14;

      ReferenceFrame pelvisFrame = drcSimulationTestHelper.getSDFFullRobotModel().getPelvis().getBodyFixedFrame();

      FootstepDataListMessage footsteps = HumanoidMessageTools.createFootstepDataListMessage(swingTime, transferTime);
      RobotSide robotSide = RobotSide.LEFT;
      double footstepX, footstepY;
      for (int i = 1; i <= steps; i++)
      {
         robotSide = i % 2 == 0 ? RobotSide.LEFT : RobotSide.RIGHT;
         footstepY = robotSide == RobotSide.LEFT ? stepWidth : -stepWidth;
         footstepX = stepLength * i + initialXPosition;
         FramePoint3D location = new FramePoint3D(pelvisFrame, footstepX, footstepY, 0.0);
         location.changeFrame(ReferenceFrame.getWorldFrame());
         location.setZ(stepHeight);
         Quaternion orientation = new Quaternion(0.0, 0.0, 0.0, 1.0);
         FootstepDataMessage footstepData = HumanoidMessageTools.createFootstepDataMessage(robotSide, location, orientation);
         footsteps.getFootstepDataList().add().set(footstepData);
      }
      // closing step
      robotSide = robotSide.getOppositeSide();
      footstepY = robotSide == RobotSide.LEFT ? stepWidth : -stepWidth;
      footstepX = stepLength * steps + initialXPosition;
      FramePoint3D location = new FramePoint3D(pelvisFrame, footstepX, footstepY, 0.0);
      location.changeFrame(ReferenceFrame.getWorldFrame());
      location.setZ(stepHeight);
      Quaternion orientation = new Quaternion(0.0, 0.0, 0.0, 1.0);
      FootstepDataMessage footstepData = HumanoidMessageTools.createFootstepDataMessage(robotSide, location, orientation);
      footsteps.getFootstepDataList().add().set(footstepData);

      drcSimulationTestHelper.publishToController(footsteps);
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.9 + transferTime + swingTime));
      updateAnkleLimitStatus();
      assertTrue((isAnkleLimitAtJointLimit == 0.0) && drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.1));
      updateAnkleLimitStatus();
      assertTrue((isAnkleLimitAtJointLimit == 0.0) && drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.1));
      updateAnkleLimitStatus();
      assertTrue((isAnkleLimitAtJointLimit == 0.0) && drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.1));
      updateAnkleLimitStatus();
      assertTrue((isAnkleLimitAtJointLimit == 0.0) && drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.1));
      updateAnkleLimitStatus();
      assertTrue(isAnkleLimitAtJointLimit == 0.0);
   }

   private void setYoVariablesToDoToeOffInSS(double icpProximity, double icpPercentLengthToLeadingFoot, double ecmpProximity)
   {
      YoDouble yoICPProximity, yoICPPercent, yoECMPProximity;

      YoBoolean toeOffAtJointLimit = (YoBoolean) drcSimulationTestHelper.getYoVariable("forceToeOffAtJointLimit");
      toeOffAtJointLimit.set(true);

      YoBoolean toeOffInSS = (YoBoolean) drcSimulationTestHelper.getYoVariable("doToeOffIfPossibleInSingleSupport");
      yoICPProximity = (YoDouble) drcSimulationTestHelper.getYoVariable("icpProximityForToeOff");
      yoICPPercent = (YoDouble) drcSimulationTestHelper.getYoVariable("icpPercentOfStanceForSSToeOff");
      yoECMPProximity = (YoDouble) drcSimulationTestHelper.getYoVariable("ecmpProximityForToeOff");

      toeOffInSS.set(true);
      yoICPProximity.set(icpProximity);
      yoICPPercent.set(icpPercentLengthToLeadingFoot);
      yoECMPProximity.set(ecmpProximity);
   }

   private void updateAnkleLimitStatus()
   {
      double leftAnklePitch =  drcSimulationTestHelper.getControllerFullRobotModel().getLegJoint(RobotSide.LEFT, LegJointName.ANKLE_PITCH).getQ();
      double rightAnklePitch =  drcSimulationTestHelper.getControllerFullRobotModel().getLegJoint(RobotSide.RIGHT, LegJointName.ANKLE_PITCH).getQ();
      isAnkleAtJointLimit |= MathTools.epsilonCompare(leftAnklePitch, anklePitchLowerLimit, Math.toRadians(1.0));
      isAnkleAtJointLimit |= MathTools.epsilonCompare(leftAnklePitch, anklePitchUpperLimit, Math.toRadians(1.0));
      isAnkleAtJointLimit |= MathTools.epsilonCompare(rightAnklePitch, anklePitchLowerLimit, Math.toRadians(1.0));
      isAnkleAtJointLimit |= MathTools.epsilonCompare(rightAnklePitch, anklePitchUpperLimit, Math.toRadians(1.0));
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
