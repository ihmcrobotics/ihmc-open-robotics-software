package us.ihmc.avatar.footstepPlanning;

import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepDataMessage;
import org.junit.jupiter.api.AfterEach;
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
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.geometry.PlanarRegionsListGenerator;
import us.ihmc.robotics.partNames.HumanoidJointNameMap;
import us.ihmc.robotics.referenceFrames.TransformReferenceFrame;
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
      FLAT_FORWARDS, FLAT_BACKWARDS, FLAT_LEFT, FLAT_RIGHT, FLAT_RANDOM, STAIRS_FORWARDS, STAIRS_BACKWARDS, RANDOM
   }

   private static final boolean visualize = true;
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();
   private DRCSimulationTestHelper drcSimulationTestHelper;

   private static final int numberOfStancesToCheck = 1;
   private static final int numberOfStepsToTake = 5;
   private static final double solutionQualityThreshold = 2.2;
   private static final double initialStanceTime = 0.5;
   private static final double swingDuration = 2;
   private static final double stepTime = numberOfStepsToTake * swingDuration * 1.5;
   private static final Random random = new Random(250);

   private DRCRobotModel robotModel;
   private List<KinematicsToolboxSnapshotDescription> feasibleSolutions;

   @BeforeEach
   public void setup()
   {
      simulationTestingParameters.setKeepSCSUp(visualize && !ContinuousIntegrationTools.isRunningOnContinuousIntegrationServer());
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());
      robotModel = getRobotModel();
      List<KinematicsToolboxSnapshotDescription> snapShots = StepReachabilityFileTools.loadKinematicsSnapshots(robotModel);
      LogTools.info("Filtering feasible solutions");
      feasibleSolutions = snapShots.stream()
                                   .filter(snapshot -> snapshot.getIkSolution().getSolutionQuality() < solutionQualityThreshold)
                                   .collect(Collectors.toList());
      LogTools.info(feasibleSolutions.size() + " feasible solutions found");
      if (feasibleSolutions.size() < numberOfStancesToCheck)
      {
         LogTools.error("Not enough feasible solutions found to check. Wanted to test " + numberOfStancesToCheck + " but only " + feasibleSolutions.size()
                        + " found with solution quality of " + solutionQualityThreshold + ". Increase solution quality.");
         fail();
      }
   }

   @AfterEach
   public void tearDown()
   {
      if (simulationTestingParameters.getKeepSCSUp())
         ThreadTools.sleepForever();

      // Do this here in case a test fails. That way the memory will be recycled.
      if (drcSimulationTestHelper != null)
      {
         drcSimulationTestHelper.destroySimulation();
         drcSimulationTestHelper = null;
      }

      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   protected abstract HumanoidRobotMutableInitialSetup createInitialSetup(HumanoidJointNameMap jointNameMap);

   @Test
   public void testFlatForwards() throws Exception
   {
      for (int i = 0; i < numberOfStancesToCheck; i++)
      {
         testSteps(robotModel, feasibleSolutions, Mode.FLAT_FORWARDS);
         endDRCSimulationTest();
      }
   }

   @Test
   public void testFlatBackwards() throws Exception
   {
      for (int i = 0; i < numberOfStancesToCheck; i++)
      {
         testSteps(robotModel, feasibleSolutions, Mode.FLAT_BACKWARDS);
         endDRCSimulationTest();
      }
   }

   @Test
   public void testFlatLeft() throws Exception
   {
      for (int i = 0; i < numberOfStancesToCheck; i++)
      {
         testSteps(robotModel, feasibleSolutions, Mode.FLAT_LEFT);
         endDRCSimulationTest();
      }
   }

   @Test
   public void testFlatRight() throws Exception
   {
      for (int i = 0; i < numberOfStancesToCheck; i++)
      {
         testSteps(robotModel, feasibleSolutions, Mode.FLAT_RIGHT);
         endDRCSimulationTest();
      }
   }

   @Test
   public void testFlatRandom() throws Exception
   {
      for (int i = 0; i < numberOfStancesToCheck; i++)
      {
         testSteps(robotModel, feasibleSolutions, Mode.FLAT_RANDOM);
         endDRCSimulationTest();
      }
   }

   @Test
   public void testStairsForwards() throws Exception
   {
      for (int i = 0; i < numberOfStancesToCheck; i++)
      {
         testSteps(robotModel, feasibleSolutions, Mode.STAIRS_FORWARDS);
         endDRCSimulationTest();
      }
   }

   @Test
   public void testStairsBackwards() throws Exception
   {
      for (int i = 0; i < numberOfStancesToCheck; i++)
      {
         testSteps(robotModel, feasibleSolutions, Mode.STAIRS_BACKWARDS);
         endDRCSimulationTest();
      }
   }

   // TODO Complete test and move on to next one
   private void testSteps(DRCRobotModel robotModel, List<KinematicsToolboxSnapshotDescription> feasibleSolutions, Mode mode)
         throws BlockingSimulationRunner.SimulationExceededMaximumTimeException
   {
      FullHumanoidRobotModel fullRobotModel = robotModel.createFullRobotModel();

      LogTools.info("Starting to generate regions");
      PlanarRegionsListGenerator generator = new PlanarRegionsListGenerator();
      for (RobotSide robotSide : RobotSide.values)
      {
         generator.identity();
         MovingReferenceFrame soleFrame = fullRobotModel.getSoleFrame(robotSide);
         generator.setTransform(soleFrame.getTransformToWorldFrame());
         generator.addRectangle(0.4, 0.4);
      }

      // this is the ROS message to command footsteps to the robot
      FootstepDataListMessage footstepDataListMessage = new FootstepDataListMessage();

      // Starting stance reference is right foot frame
      RigidBodyTransform rightSole = fullRobotModel.getSoleFrame(RobotSide.RIGHT).getTransformToWorldFrame();
      Point3D previousPose = new Point3D(rightSole.getTranslationX(), rightSole.getTranslationY(), rightSole.getTranslationZ());
      double previousYaw = rightSole.getRotation().getYaw();

      for (int i = 0; i < numberOfStepsToTake; i++)
      {
         RobotSide robotSide = i % 2 == 0 ? RobotSide.LEFT : RobotSide.RIGHT;

         // Load random step from script
         int stepIndex = getNextStep(feasibleSolutions, robotSide, mode);
         KinematicsToolboxSnapshotDescription snapshotToStep = feasibleSolutions.get(stepIndex);
         feasibleSolutions.remove(stepIndex);
         SixDoFMotionControlAnchorDescription footstep = snapshotToStep.getSixDoFAnchors().get(0);
         assert (footstep.getRigidBodyName().equals(fullRobotModel.getFoot(RobotSide.LEFT).getName()));

         Point3D loadedPose = footstep.getInputMessage().getDesiredPositionInWorld();
         Quaternion loadedOrientation = footstep.getInputMessage().getDesiredOrientationInWorld();

         TransformReferenceFrame stanceFootFrame = new TransformReferenceFrame("stanceFootFrame", ReferenceFrame.getWorldFrame());
         stanceFootFrame.setTransformAndUpdate(new RigidBodyTransform(new Quaternion(previousYaw, 0, 0), previousPose));

         TransformReferenceFrame loadedFootFrame = new TransformReferenceFrame("loadedFootFrame", stanceFootFrame);
         loadedFootFrame.setTransformAndUpdate(new RigidBodyTransform(loadedOrientation, loadedPose));

         // Adjust candidate foot position if right step
         if (robotSide == RobotSide.LEFT)
         {
            double stepY = loadedPose.getY();
            loadedPose.setY(-stepY);

            double stepYaw = loadedOrientation.getYaw();
            double stepPitch = loadedOrientation.getPitch();
            double stepRoll = loadedOrientation.getRoll();
            loadedOrientation.setYawPitchRoll(-stepYaw, stepPitch, stepRoll);
         }

         // Get desired step in stance frame
         FramePose3D footPose = new FramePose3D();
         footPose.setToZero(loadedFootFrame);
         footPose.changeFrame(stanceFootFrame);

         // In world frame, set stepping stones and add to command list
         footPose.changeFrame(ReferenceFrame.getWorldFrame());
         double newX = footPose.getX();
         double newY = footPose.getY();
         double newZ = footPose.getZ();
         Quaternion orientation = new Quaternion(footPose.getOrientation());

         // Create stepping stones at the end of each step
         generator.identity();
         FramePoint3D step = new FramePoint3D(ReferenceFrame.getWorldFrame(), newX, newY, newZ);
         generator.translate(step.getX(), step.getY(), step.getZ());
         generator.addRectangle(0.4, 0.4);

         // Add to footstep command list
         FootstepDataMessage footstepData = HumanoidMessageTools.createFootstepDataMessage(robotSide, step, orientation);
         footstepData.setSwingDuration(swingDuration);
         footstepDataListMessage.getFootstepDataList().add().set(footstepData);

         previousPose.setX(step.getX());
         previousPose.setY(step.getY());
         previousPose.setZ(step.getZ());
         previousYaw = orientation.getYaw();
      }

      PlanarRegionsList planarRegionsList = generator.getPlanarRegionsList();
      PlanarRegionsListDefinedEnvironment environment = new PlanarRegionsListDefinedEnvironment(planarRegionsList, 0.015, false);

      HumanoidRobotMutableInitialSetup initialSetup = createInitialSetup(robotModel.getJointMap());

      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, robotModel, environment);
      drcSimulationTestHelper.setInitialSetup(initialSetup);
      drcSimulationTestHelper.createSimulation(robotModel.getSimpleRobotName() + "FlatGroundWalking");

      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(initialStanceTime);
      if (!visualize)
         assertTrue(success);

      drcSimulationTestHelper.publishToController(footstepDataListMessage);

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(stepTime);
      if (!visualize)
         assertTrue(success);
   }

   private int getNextStep(List<KinematicsToolboxSnapshotDescription> feasibleSolutions, RobotSide robotSide, Mode mode)
   {
      int stepIndex = -1;
      int maxNumOfIterations = 1000;
      int count = 0;
      while (count < maxNumOfIterations)
      {
         int randIndex = random.nextInt(feasibleSolutions.size());
         KinematicsToolboxSnapshotDescription snapshotToTest = feasibleSolutions.get(randIndex);
         SixDoFMotionControlAnchorDescription leftFoot = snapshotToTest.getSixDoFAnchors().get(0);
         Point3D desiredPosition = leftFoot.getInputMessage().getDesiredPositionInWorld();
         switch (mode)
         {
            case FLAT_FORWARDS:
               if (desiredPosition.getX() > 0 && desiredPosition.getZ() == 0)
                  return randIndex;
               break;
            case FLAT_BACKWARDS:
               if (desiredPosition.getX() < 0 && desiredPosition.getZ() == 0)
                  return randIndex;
               break;
            case FLAT_LEFT:
               if (desiredPosition.getY() > 0 && desiredPosition.getZ() == 0)
                  return randIndex;
               break;
            case FLAT_RIGHT:
               if (desiredPosition.getY() < 0 && desiredPosition.getZ() == 0)
                  return randIndex;
               break;
            case FLAT_RANDOM:
               if (desiredPosition.getZ() == 0)
                  return randIndex;
               break;
            case STAIRS_FORWARDS:
               // Make sure that the next step isn't right above the previous one (i.e. when both x and y are too close)
               boolean xTooCloseAbove = Math.abs(desiredPosition.getX()) <= 0.1;
               boolean yTooCloseAbove = Math.abs(desiredPosition.getY()) <= 0.1;
               if (desiredPosition.getX() >= 0 && desiredPosition.getZ() > 0 && !(xTooCloseAbove && yTooCloseAbove))
                  return randIndex;
               break;
            case STAIRS_BACKWARDS:
               // Make sure that the next step isn't right below the previous one (i.e. when both x and y are too close)
               boolean xTooCloseBelow = Math.abs(desiredPosition.getX()) <= 0.1;
               boolean yTooCloseBelow = Math.abs(desiredPosition.getY()) <= 0.1;
               if (desiredPosition.getX() <= 0 && desiredPosition.getZ() > 0 && !(xTooCloseBelow && yTooCloseBelow))
                  return randIndex;
               break;
            case RANDOM:
               return randIndex;
         }
         count++;
      }
      if (stepIndex == -1)
         LogTools.error("Could not find valid next step in multi-step sequence in " + mode + " mode. Increase solution quality or maxNumOfIterations.");
      return stepIndex;
   }

   public void endDRCSimulationTest()
   {
      drcSimulationTestHelper.getBlockingSimulationRunner().destroySimulation();
      drcSimulationTestHelper.getAvatarSimulation().dispose();
      drcSimulationTestHelper.getSimulationStarter().close();
      drcSimulationTestHelper.getROS2Node().destroy();
   }
}
