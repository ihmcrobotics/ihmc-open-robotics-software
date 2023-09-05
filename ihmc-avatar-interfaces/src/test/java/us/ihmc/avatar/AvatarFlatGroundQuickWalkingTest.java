package us.ihmc.avatar;

import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import controller_msgs.msg.dds.FootstepDataListMessage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.testTools.EndToEndTestTools;
import us.ihmc.avatar.testTools.scs2.SCS2AvatarTestingSimulation;
import us.ihmc.avatar.testTools.scs2.SCS2AvatarTestingSimulationFactory;
import us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator.HeadingAndVelocityEvaluationScriptParameters;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationConstructionSetTools.util.environments.FlatGroundEnvironment;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.yoVariables.variable.YoDouble;

public abstract class AvatarFlatGroundQuickWalkingTest implements MultiRobotTestInterface
{
   protected static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();
   private SCS2AvatarTestingSimulation simulationTestHelper;

   @BeforeEach
   public void setup()
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());
   }

   @AfterEach
   public void tearDown()
   {
      if (simulationTestHelper != null)
      {
         simulationTestHelper.finishTest();
         simulationTestHelper = null;
      }

      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   public abstract double getFastSwingTime();

   public abstract double getFastTransferTime();

   public abstract double getMaxForwardStepLength();

   public double getFastStepWidth()
   {
      return 0.25;
   }

   public int getNumberOfSteps()
   {
      return 30;
   }

   @Test
   public void testForwardWalking() throws Exception
   {
      setupSim(getRobotModel(), false, false, null);
      assertTrue(simulationTestHelper.simulateNow(2.0));

      ((YoDouble) simulationTestHelper.findVariable("icpDistanceFromFootPolygonThreshold")).set(0.20);

      CommonHumanoidReferenceFrames referenceFrames = simulationTestHelper.getControllerReferenceFrames();
      MovingReferenceFrame midFootZUpGroundFrame = referenceFrames.getMidFootZUpGroundFrame();
      FramePose3D startPose = new FramePose3D(midFootZUpGroundFrame);
      startPose.changeFrame(ReferenceFrame.getWorldFrame());
      FootstepDataListMessage footsteps = EndToEndTestTools.generateForwardSteps(RobotSide.LEFT,
                                                                                 getNumberOfSteps(),
                                                                                 EndToEndTestTools.trapezoidFunction(0.2,
                                                                                                                     getMaxForwardStepLength(),
                                                                                                                     0.15,
                                                                                                                     0.85),
                                                                                 getFastStepWidth(),
                                                                                 getFastSwingTime(),
                                                                                 getFastTransferTime(),
                                                                                 startPose,
                                                                                 true);
//      footsteps.setOffsetFootstepsHeightWithExecutionError(true);
//      footsteps.setAreFootstepsAdjustable(true);
      simulationTestHelper.publishToController(footsteps);

      boolean success = simulationTestHelper.simulateNow(1.1
            * EndToEndTestTools.computeWalkingDuration(footsteps, getRobotModel().getWalkingControllerParameters()));
      assertTrue(success);
   }

   public void setUsePerfectSensors(boolean usePerfectSensors)
   {
      simulationTestingParameters.setUsePefectSensors(usePerfectSensors);
   }

   private void setupSim(DRCRobotModel robotModel,
                         boolean useVelocityAndHeadingScript,
                         boolean cheatWithGroundHeightAtForFootstep,
                         HeadingAndVelocityEvaluationScriptParameters walkingScriptParameters)
   {
      FlatGroundEnvironment flatGround = new FlatGroundEnvironment();
      SCS2AvatarTestingSimulationFactory simulationTestHelperFactory = SCS2AvatarTestingSimulationFactory.createDefaultTestSimulationFactory(robotModel,
                                                                                                                                             flatGround,
                                                                                                                                             simulationTestingParameters);

      if (cheatWithGroundHeightAtForFootstep)
         simulationTestHelperFactory.setComponentBasedFootstepDataMessageGeneratorParameters(useVelocityAndHeadingScript,
                                                                                             walkingScriptParameters);
      else
         simulationTestHelperFactory.setComponentBasedFootstepDataMessageGeneratorParameters(useVelocityAndHeadingScript,
                                                                                             flatGround.getTerrainObject3D().getHeightMapIfAvailable(),
                                                                                             walkingScriptParameters);

      simulationTestHelper = simulationTestHelperFactory.createAvatarTestingSimulation();

      simulationTestHelper.start();
   }
}
