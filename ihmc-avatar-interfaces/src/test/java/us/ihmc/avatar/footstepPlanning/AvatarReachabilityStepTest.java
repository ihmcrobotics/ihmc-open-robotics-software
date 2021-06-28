package us.ihmc.avatar.footstepPlanning;

import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepDataMessage;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.initialSetup.HumanoidRobotMutableInitialSetup;
import us.ihmc.avatar.multiContact.KinematicsToolboxSnapshotDescription;
import us.ihmc.avatar.multiContact.SixDoFMotionControlAnchorDescription;
import us.ihmc.avatar.reachabilityMap.footstep.StepReachabilityFileTools;
import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
import us.ihmc.commons.ContinuousIntegrationTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.idl.IDLSequence;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullRobotModelUtils;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.geometry.PlanarRegionsListGenerator;
import us.ihmc.robotics.partNames.HumanoidJointNameMap;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationConstructionSetTools.util.environments.FlatGroundEnvironment;
import us.ihmc.simulationConstructionSetTools.util.environments.PlanarRegionsListDefinedEnvironment;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;

import java.util.List;
import java.util.Random;
import java.util.stream.Collectors;

import static org.junit.jupiter.api.Assertions.*;
import static us.ihmc.robotics.Assert.assertTrue;

public abstract class AvatarReachabilityStepTest implements MultiRobotTestInterface
{
   private static final boolean visualize = true;
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();
   private DRCSimulationTestHelper drcSimulationTestHelper;

   private static final int numberOfStancesToCheck = 10;
   private static final double solutionQualityThreshold = 2.2;
   private static final double initialStanceTime = 1.0;
   private static final double stepTime = 5.0;
   private static final double finalStanceTime = 2.0;
   private static final Random random = new Random(3920);

   @BeforeEach
   public void setup()
   {
      simulationTestingParameters.getKeepSCSUp();
//      simulationTestingParameters.setKeepSCSUp(visualize && !ContinuousIntegrationTools.isRunningOnContinuousIntegrationServer());
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

   protected abstract HumanoidRobotMutableInitialSetup createInitialSetup(HumanoidJointNameMap jointNameMap);

   @Test
   public void testSingleStep() throws Exception
   {
      DRCRobotModel robotModel = getRobotModel();
      List<KinematicsToolboxSnapshotDescription> snapShots = StepReachabilityFileTools.loadKinematicsSnapshots(robotModel);

      LogTools.info("Filtering feasible solutions");
      List<KinematicsToolboxSnapshotDescription> feasibleSolutions = snapShots.stream()
                                                                              .filter(snapshot -> snapshot.getIkSolution().getSolutionQuality()
                                                                                                  < solutionQualityThreshold)
                                                                              .collect(Collectors.toList());
      LogTools.info(feasibleSolutions.size() + " feasible solutions found");

      if (feasibleSolutions.size() < numberOfStancesToCheck)
      {
         LogTools.error("Not enough feasible solutions found to check. Wanted to test " + numberOfStancesToCheck + " but only " + feasibleSolutions.size()
                        + " found with solution quality of " + solutionQualityThreshold + ". Increase solution quality.");
         fail();
      }

      for (int i = 0; i < numberOfStancesToCheck; i++)
      {
         testSteps(robotModel, feasibleSolutions);

         drcSimulationTestHelper.getBlockingSimulationRunner().destroySimulation();
         drcSimulationTestHelper.getAvatarSimulation().dispose();
         drcSimulationTestHelper.getSimulationStarter().close();
         drcSimulationTestHelper.getROS2Node().destroy();
      }
   }

   private void testSteps(DRCRobotModel robotModel, List<KinematicsToolboxSnapshotDescription> feasibleSolutions)
         throws BlockingSimulationRunner.SimulationExceededMaximumTimeException
   {
      int indexToTest = random.nextInt(feasibleSolutions.size());

      KinematicsToolboxSnapshotDescription snapshotToTest = feasibleSolutions.get(indexToTest);
      feasibleSolutions.remove(indexToTest);

      IDLSequence.Float jointAngles = snapshotToTest.getIkSolution().getDesiredJointAngles();
      Vector3D rootPosition = snapshotToTest.getIkSolution().getDesiredRootTranslation();
      Quaternion rootOrientation = snapshotToTest.getIkSolution().getDesiredRootOrientation();

      FullHumanoidRobotModel fullRobotModel = robotModel.createFullRobotModel();
      OneDoFJointBasics[] ikJoints = FullRobotModelUtils.getAllJointsExcludingHands(fullRobotModel);
      for (int i = 0; i < jointAngles.size(); i++)
      {
         ikJoints[i].setQ(jointAngles.get(i));
      }

      fullRobotModel.getRootJoint().setJointConfiguration(rootOrientation, rootPosition);
      fullRobotModel.updateFrames();

      LogTools.info("Starting to generate regions");
      PlanarRegionsListGenerator generator = new PlanarRegionsListGenerator();
      for (RobotSide robotSide : RobotSide.values)
      {
         generator.identity();
         MovingReferenceFrame soleFrame = fullRobotModel.getSoleFrame(robotSide);
         generator.setTransform(soleFrame.getTransformToWorldFrame());
         generator.addRectangle(0.4, 0.4);
      }

      ReferenceFrame leftFoot = fullRobotModel.getFoot(RobotSide.LEFT).getBodyFixedFrame();

      int indexToStep = random.nextInt(feasibleSolutions.size());

      // Get location and orientation of step
      KinematicsToolboxSnapshotDescription snapshotToStep = feasibleSolutions.get(indexToTest);
      feasibleSolutions.remove(indexToStep);
      SixDoFMotionControlAnchorDescription leftStep = snapshotToStep.getSixDoFAnchors().get(0);
      assert (leftStep.getRigidBodyName().equals(fullRobotModel.getFoot(RobotSide.LEFT).getName()));
      Point3D desiredPose = leftStep.getInputMessage().getDesiredPositionInWorld();
      Quaternion orientation = leftStep.getInputMessage().getDesiredOrientationInWorld();

      // Create a stepping stone at the end of the step
      generator.identity();
      FramePoint3D step = new FramePoint3D(leftFoot, desiredPose.getX(), desiredPose.getY(), desiredPose.getZ());
      step.changeFrame(ReferenceFrame.getWorldFrame());
      generator.translate(step.getX(), step.getY(), step.getZ());
      generator.addRectangle(0.4, 0.4);

      PlanarRegionsList planarRegionsList = generator.getPlanarRegionsList();
      PlanarRegionsListDefinedEnvironment environment = new PlanarRegionsListDefinedEnvironment(planarRegionsList, 0.015, false);

      HumanoidRobotMutableInitialSetup initialSetup = createInitialSetup(robotModel.getJointMap());
      initialSetup.getRootJointPosition().set(rootPosition);
      initialSetup.getRootJointOrientation().set(rootOrientation);

      for (int i = 0; i < ikJoints.length; i++)
      {
         initialSetup.getJointPositions().put(ikJoints[i].getName(), ikJoints[i].getQ());
      }

      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, robotModel, environment);
      drcSimulationTestHelper.setInitialSetup(initialSetup);
      drcSimulationTestHelper.createSimulation(robotModel.getSimpleRobotName() + "FlatGroundWalking");

      // this is the ROS message to command footsteps to the robot
      FootstepDataListMessage footstepDataListMessage = new FootstepDataListMessage();
      FootstepDataMessage footstepData = HumanoidMessageTools.createFootstepDataMessage(RobotSide.LEFT, step, orientation);
      footstepDataListMessage.getFootstepDataList().add().set(footstepData);

      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(initialStanceTime);
      if (!visualize)
         Assertions.assertTrue(success);

      drcSimulationTestHelper.publishToController(footstepDataListMessage);

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(stepTime);
      if (!visualize)
         Assertions.assertTrue(success);
   }
}
