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
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.geometry.PlanarRegionsListGenerator;
import us.ihmc.robotics.partNames.HumanoidJointNameMap;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationConstructionSetTools.util.environments.PlanarRegionsListDefinedEnvironment;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;

import java.util.List;
import java.util.Random;
import java.util.stream.Collectors;

import static org.junit.jupiter.api.Assertions.*;

public abstract class AvatarReachabilityMultiStepTest implements MultiRobotTestInterface
{
   private enum Mode
   {
      FLAT_FORWARD, FLAT_BACKWARDS, FLAT_LEFT, FLAT_RIGHT, FLAT_RANDOM, FORWARD_STAIRS, BACKWARDS_STAIRS, RANDOM
   }

   private static final Mode mode = Mode.FLAT_RANDOM;

   private static final boolean visualize = true;
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();
   private DRCSimulationTestHelper drcSimulationTestHelper;

   private static final int numberOfStancesToCheck = 10;
   private static final int numberOfStepsToTake = 5;
   private static final double solutionQualityThreshold = 2.2;
   private static final double initialStanceTime = 1.0;
   private static final double swingDuration = 2;
   private static final double stepTime = numberOfStepsToTake * swingDuration * 1.5;
   private static final double finalStanceTime = 2.0;
   private static final Random random = new Random(1000);

   @BeforeEach
   public void setup()
   {
      simulationTestingParameters.setKeepSCSUp(visualize && !ContinuousIntegrationTools.isRunningOnContinuousIntegrationServer());
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
   public void testMultipleSteps() throws Exception
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

//         drcSimulationTestHelper.getBlockingSimulationRunner().destroySimulation();
         drcSimulationTestHelper.getAvatarSimulation().dispose();
         drcSimulationTestHelper.getSimulationStarter().close();
         drcSimulationTestHelper.getROS2Node().destroy();
      }
   }

   private void testSteps(DRCRobotModel robotModel, List<KinematicsToolboxSnapshotDescription> feasibleSolutions)
         throws BlockingSimulationRunner.SimulationExceededMaximumTimeException
   {
      KinematicsToolboxSnapshotDescription frameToStart = feasibleSolutions.get(0);
      Vector3D rootPosition = frameToStart.getIkSolution().getDesiredRootTranslation();
      Quaternion rootOrientation = new Quaternion();
      //      Quaternion rootOrientation = frameToStart.getIkSolution().getDesiredRootOrientation();

      FullHumanoidRobotModel fullRobotModel = robotModel.createFullRobotModel();
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

      // Should be in reference to right foot frame (at origin)? Check this
      RigidBodyTransform rightSoleFrame = fullRobotModel.getSoleFrame(RobotSide.RIGHT).getTransformToWorldFrame();

      // this is the ROS message to command footsteps to the robot
      FootstepDataListMessage footstepDataListMessage = new FootstepDataListMessage();
      Point3D previousPose = new Point3D(rightSoleFrame.getTranslationX(), rightSoleFrame.getTranslationY(), rightSoleFrame.getTranslationZ());
      double previousYaw = 0.0;

      for (int i = 0; i < numberOfStepsToTake; i++)
      {
         RobotSide robotSide = i % 2 == 0 ? RobotSide.LEFT : RobotSide.RIGHT;
         int stepIndex = getNextStep(feasibleSolutions, robotSide);
         KinematicsToolboxSnapshotDescription snapshotToStep = feasibleSolutions.get(stepIndex);
         feasibleSolutions.remove(stepIndex);

         SixDoFMotionControlAnchorDescription footstep = snapshotToStep.getSixDoFAnchors().get(0);
         assert (footstep.getRigidBodyName().equals(fullRobotModel.getFoot(RobotSide.LEFT).getName()));
         Point3D desiredPose = footstep.getInputMessage().getDesiredPositionInWorld();
         Quaternion desiredOrientation = footstep.getInputMessage().getDesiredOrientationInWorld();

         // Adjust candidate foot position if right step
         if (robotSide == RobotSide.RIGHT)
         {
            double stepY = desiredPose.getY();
            desiredPose.setY(-stepY);

            double stepYaw = desiredOrientation.getYaw();
            double stepPitch = desiredOrientation.getPitch();
            double stepRoll = desiredOrientation.getRoll();
            desiredOrientation.setYawPitchRoll(-stepYaw, stepPitch, stepRoll);
         }

         // Desired pose and orientation should be relative to the previous step
         double newX = desiredPose.getX() + previousPose.getX();
         double newY = desiredPose.getY() + previousPose.getY();
         double newZ = desiredPose.getZ() + previousPose.getZ();
         double newYaw = desiredOrientation.getYaw() + previousYaw;
         double stepPitch = desiredOrientation.getPitch();
         double stepRoll = desiredOrientation.getRoll();
         desiredPose.setX(newX);
         desiredPose.setY(newY);
         desiredPose.setZ(newZ);
         desiredOrientation.setYawPitchRoll(newYaw, stepPitch, stepRoll);

         // Create stepping stones at the end of each step
         generator.identity();
         FramePoint3D step = new FramePoint3D(ReferenceFrame.getWorldFrame(), desiredPose.getX(), desiredPose.getY(), desiredPose.getZ());
         generator.translate(step.getX(), step.getY(), step.getZ());
         generator.addRectangle(0.4, 0.4);

         // Add to footstep command list
         FootstepDataMessage footstepData = HumanoidMessageTools.createFootstepDataMessage(robotSide, step, desiredOrientation);
         footstepData.setSwingDuration(swingDuration);
         footstepDataListMessage.getFootstepDataList().add().set(footstepData);

         previousPose.setX(desiredPose.getX());
         previousPose.setY(desiredPose.getY());
         previousPose.setZ(desiredPose.getZ());
         previousYaw = desiredOrientation.getYaw();
      }

      PlanarRegionsList planarRegionsList = generator.getPlanarRegionsList();
      PlanarRegionsListDefinedEnvironment environment = new PlanarRegionsListDefinedEnvironment(planarRegionsList, 0.015, false);

      HumanoidRobotMutableInitialSetup initialSetup = createInitialSetup(robotModel.getJointMap());
      initialSetup.getRootJointPosition().set(rootPosition);
      initialSetup.getRootJointOrientation().set(rootOrientation);

      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, robotModel, environment);
      drcSimulationTestHelper.setInitialSetup(initialSetup);
      drcSimulationTestHelper.createSimulation(robotModel.getSimpleRobotName() + "FlatGroundWalking");

      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(initialStanceTime);
      if (!visualize)
         Assertions.assertTrue(success);

      drcSimulationTestHelper.publishToController(footstepDataListMessage);

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(stepTime);
      if (!visualize)
         Assertions.assertTrue(success);
   }

   private int getNextStep(List<KinematicsToolboxSnapshotDescription> feasibleSolutions, RobotSide robotSide)
   {
      int stepIndex = -1;
      int maxNumOfIterations = 1000;
      int count = 0;
      while (count < maxNumOfIterations)
      {
         int randIndex = random.nextInt(feasibleSolutions.size());
         KinematicsToolboxSnapshotDescription snapshotToTest = feasibleSolutions.get(randIndex);
         SixDoFMotionControlAnchorDescription leftFoot = snapshotToTest.getSixDoFAnchors().get(0);
         Point3D leftFootDesiredPosition = leftFoot.getInputMessage().getDesiredPositionInWorld();
         switch (mode)
         {
            case FLAT_FORWARD:
               if (leftFootDesiredPosition.getX() >= 0 && leftFootDesiredPosition.getZ() == 0)
                  return randIndex;
               break;
            case FLAT_BACKWARDS:
               if (leftFootDesiredPosition.getX() <= 0 && leftFootDesiredPosition.getZ() == 0)
                  return randIndex;
               break;
            case FLAT_LEFT:
               if (robotSide == RobotSide.LEFT && leftFootDesiredPosition.getY() >= 0 && leftFootDesiredPosition.getZ() == 0)
                  return randIndex;
               if (robotSide == RobotSide.RIGHT && leftFootDesiredPosition.getY() <= 0 && leftFootDesiredPosition.getZ() == 0)
                  return randIndex;
               break;
            case FLAT_RIGHT:
               if (robotSide == RobotSide.LEFT && leftFootDesiredPosition.getY() <= 0 && leftFootDesiredPosition.getZ() == 0)
                  return randIndex;
               if (robotSide == RobotSide.RIGHT && leftFootDesiredPosition.getY() >= 0 && leftFootDesiredPosition.getZ() == 0)
                  return randIndex;
               break;
            case FLAT_RANDOM:
               if (leftFootDesiredPosition.getZ() == 0)
                  return randIndex;
               break;
            case FORWARD_STAIRS:
               if (leftFootDesiredPosition.getX() >= 0 && leftFootDesiredPosition.getZ() > 0)
                  return randIndex;
               break;
            case BACKWARDS_STAIRS:
               if (leftFootDesiredPosition.getX() <= 0 && leftFootDesiredPosition.getZ() > 0)
                  return randIndex;
               break;
            case RANDOM:
               return randIndex;
         }
         count++;
      }
      if (stepIndex == -1)
         LogTools.error("Could not find valid next step in multi-step sequence in " + mode + "mode. Increase solution quality or maxNumOfIterations.");
      return stepIndex;
   }
}
