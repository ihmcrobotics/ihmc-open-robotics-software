package us.ihmc.avatar;

import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.ArrayList;
import java.util.List;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import controller_msgs.msg.dds.FootstepDataListMessage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
import us.ihmc.avatar.testTools.EndToEndTestTools;
import us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator.HeadingAndVelocityEvaluationScriptParameters;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
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
   protected static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();
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
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(2.0));

      CommonHumanoidReferenceFrames referenceFrames = drcSimulationTestHelper.getReferenceFrames();
      MovingReferenceFrame midFootZUpGroundFrame = referenceFrames.getMidFootZUpGroundFrame();
      FramePose3D startPose = new FramePose3D(midFootZUpGroundFrame);
      startPose.changeFrame(ReferenceFrame.getWorldFrame());
      FootstepDataListMessage footsteps = EndToEndTestTools.forwardSteps(RobotSide.LEFT,
                                                       getNumberOfSteps(),
                                                       EndToEndTestTools.trapezoidFunction(0.2, getMaxForwardStepLength(), 0.15, 0.85),
                                                       getFastStepWidth(),
                                                       getFastSwingTime(),
                                                       getFastTransferTime(),
                                                       startPose,
                                                       true);
      footsteps.setOffsetFootstepsHeightWithExecutionError(true);
      drcSimulationTestHelper.publishToController(footsteps);

      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.1 * EndToEndTestTools.computeWalkingDuration(footsteps, getRobotModel().getWalkingControllerParameters()));
      assertTrue(success);
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
}
