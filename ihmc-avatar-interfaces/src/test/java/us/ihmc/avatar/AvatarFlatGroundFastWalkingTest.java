package us.ihmc.avatar;

import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleUnaryOperator;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepDataMessage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
import us.ihmc.avatar.testTools.EndToEndTestTools;
import us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator.HeadingAndVelocityEvaluationScriptParameters;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationConstructionSetTools.util.environments.FlatGroundEnvironment;
import us.ihmc.simulationconstructionset.FloatingJoint;
import us.ihmc.simulationconstructionset.OneDegreeOfFreedomJoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;

public abstract class AvatarFlatGroundFastWalkingTest implements MultiRobotTestInterface
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();

   private DRCSimulationTestHelper drcSimulationTestHelper;

   @BeforeEach
   public void setup()
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());
   }

   @AfterEach
   public void tearDown()
   {
      if (simulationTestingParameters.getKeepSCSUp())
         ThreadTools.sleepForever();

      if (drcSimulationTestHelper != null)
      {
         drcSimulationTestHelper.destroySimulation();
         drcSimulationTestHelper = null;
      }

      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   public abstract double getFastSwingTime();

   public abstract double getFastTransferTime();

   public abstract double getMaxForwardStepLength();

   @Test
   public void testForwardWalking() throws Exception
   {
      setupSim(getRobotModel(), false, false, null);
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(2.0));

      CommonHumanoidReferenceFrames referenceFrames = drcSimulationTestHelper.getReferenceFrames();
      MovingReferenceFrame midFootZUpGroundFrame = referenceFrames.getMidFootZUpGroundFrame();
      FramePose3D startPose = new FramePose3D(midFootZUpGroundFrame);
      startPose.changeFrame(ReferenceFrame.getWorldFrame());
      FootstepDataListMessage footsteps = forwardSteps(RobotSide.LEFT,
                                                       30,
                                                       trapezoidFunction(0.2, getMaxForwardStepLength(), 0.15, 0.85),
                                                       0.25,
                                                       getFastSwingTime(),
                                                       getFastTransferTime(),
                                                       startPose,
                                                       true);
      footsteps.setOffsetFootstepsHeightWithExecutionError(true);
      drcSimulationTestHelper.publishToController(footsteps);
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.1
            * EndToEndTestTools.computeWalkingDuration(footsteps, getRobotModel().getWalkingControllerParameters())));
   }

   private void setupSim(DRCRobotModel robotModel, boolean useVelocityAndHeadingScript, boolean cheatWithGroundHeightAtForFootstep,
                         HeadingAndVelocityEvaluationScriptParameters walkingScriptParameters)
   {
      FlatGroundEnvironment flatGround = new FlatGroundEnvironment();
      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, robotModel, flatGround);
      drcSimulationTestHelper.setAddFootstepMessageGenerator(true);
      drcSimulationTestHelper.setUseHeadingAndVelocityScript(useVelocityAndHeadingScript);
      drcSimulationTestHelper.setCheatWithGroundHeightAtFootstep(cheatWithGroundHeightAtForFootstep);
      drcSimulationTestHelper.setWalkingScriptParameters(walkingScriptParameters);
      drcSimulationTestHelper.createSimulation(robotModel.getSimpleRobotName() + "FlatGroundWalking");
      setupRobotStateVarGroup();
   }

   private void setupRobotStateVarGroup()
   {
      SimulationConstructionSet scs = drcSimulationTestHelper.getSimulationConstructionSet();
      List<String> stateVars = new ArrayList<>();
      List<OneDegreeOfFreedomJoint> joints = new ArrayList<>();
      Robot robot = scs.getRobots()[0];
      robot.getAllOneDegreeOfFreedomJoints(joints);
      stateVars.add(robot.getYoTime().getFullNameString());
      FloatingJoint rootJoint = (FloatingJoint) robot.getRootJoints().get(0);
      stateVars.add(rootJoint.getQx().getFullNameString());
      stateVars.add(rootJoint.getQy().getFullNameString());
      stateVars.add(rootJoint.getQz().getFullNameString());
      stateVars.add(rootJoint.getQdx().getFullNameString());
      stateVars.add(rootJoint.getQdy().getFullNameString());
      stateVars.add(rootJoint.getQdz().getFullNameString());
      stateVars.add(rootJoint.getQuaternionQx().getFullNameString());
      stateVars.add(rootJoint.getQuaternionQy().getFullNameString());
      stateVars.add(rootJoint.getQuaternionQz().getFullNameString());
      stateVars.add(rootJoint.getQuaternionQs().getFullNameString());

      for (OneDegreeOfFreedomJoint joint : joints)
      {
         stateVars.add(joint.getQYoVariable().getFullNameString());
         stateVars.add(joint.getQDYoVariable().getFullNameString());
         stateVars.add(joint.getTauYoVariable().getFullNameString());
      }
      scs.setupVarGroup("RobotState", stateVars.toArray(new String[0]));
   }

   public static DoubleUnaryOperator trapezoidFunction(double bottomValue, double plateauValue, double startPlateau, double endPlateau)
   {
      return percent ->
      {
         if (percent < startPlateau)
            return EuclidCoreTools.interpolate(bottomValue, plateauValue, percent / startPlateau);
         else if (percent > endPlateau)
            return EuclidCoreTools.interpolate(plateauValue, bottomValue, (percent - endPlateau) / (1.0 - endPlateau));
         else
            return plateauValue;
      };
   }

   public static FootstepDataListMessage forwardSteps(RobotSide initialStepSide, int numberOfSteps, DoubleUnaryOperator stepLengthFunction, double stepWidth,
                                                       double swingTime, double transferTime, Pose3DReadOnly startPose, boolean squareUp)
   {
      FootstepDataListMessage message = new FootstepDataListMessage();
      FootstepDataMessage footstep = message.getFootstepDataList().add();

      RobotSide stepSide = initialStepSide;
      Pose3D stepPose = new Pose3D(startPose);
      stepPose.appendTranslation(0.5 * stepLengthFunction.applyAsDouble(0.0), stepSide.negateIfRightSide(0.5 * stepWidth), 0.0);
      footstep.setRobotSide(stepSide.toByte());
      footstep.getLocation().set(stepPose.getPosition());
      footstep.getOrientation().set(stepPose.getOrientation());
      footstep.setSwingDuration(swingTime);

      for (int i = 1; i < numberOfSteps; i++)
      {
         stepSide = stepSide.getOppositeSide();
         stepPose.appendTranslation(stepLengthFunction.applyAsDouble(i / (numberOfSteps - 1.0)), stepSide.negateIfRightSide(stepWidth), 0.0);
         footstep = message.getFootstepDataList().add();
         footstep.setRobotSide(stepSide.toByte());
         footstep.getLocation().set(stepPose.getPosition());
         footstep.getOrientation().set(stepPose.getOrientation());
         footstep.setTransferDuration(transferTime);
         footstep.setSwingDuration(swingTime);
      }

      if (squareUp)
      {
         stepSide = stepSide.getOppositeSide();
         stepPose.appendTranslation(0.0, stepSide.negateIfRightSide(stepWidth), 0.0);
         footstep = message.getFootstepDataList().add();
         footstep.setRobotSide(stepSide.toByte());
         footstep.getLocation().set(stepPose.getPosition());
         footstep.getOrientation().set(stepPose.getOrientation());
         footstep.setTransferDuration(transferTime);
         footstep.setSwingDuration(swingTime);
      }

      return message;
   }
}
