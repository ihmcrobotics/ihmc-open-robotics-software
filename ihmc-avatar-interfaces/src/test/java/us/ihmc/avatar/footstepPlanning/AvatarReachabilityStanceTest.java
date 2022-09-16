package us.ihmc.avatar.footstepPlanning;

import static org.junit.jupiter.api.Assertions.fail;

import java.util.List;
import java.util.Random;
import java.util.stream.Collectors;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import controller_msgs.msg.dds.HandTrajectoryMessage;
import controller_msgs.msg.dds.HeadTrajectoryMessage;
import controller_msgs.msg.dds.PelvisHeightTrajectoryMessage;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.initialSetup.HumanoidRobotInitialSetup;
import us.ihmc.avatar.multiContact.KinematicsToolboxSnapshotDescription;
import us.ihmc.avatar.reachabilityMap.footstep.StepReachabilityIOHelper;
import us.ihmc.avatar.testTools.scs2.SCS2AvatarTestingSimulation;
import us.ihmc.avatar.testTools.scs2.SCS2AvatarTestingSimulationFactory;
import us.ihmc.commons.ContinuousIntegrationTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
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
import us.ihmc.simulationConstructionSetTools.util.environments.PlanarRegionsListDefinedEnvironment;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;

public abstract class AvatarReachabilityStanceTest implements MultiRobotTestInterface
{
   private static final boolean visualize = false;
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();

   private static final int numberOfStancesToCheck = 4;
   private static final double solutionQualityThreshold = 2.2;
   private static final double simulationTime = 2.0;
   private static final Random random = new Random(3920);

   @BeforeEach
   public void setup()
   {
      simulationTestingParameters.setKeepSCSUp(!ContinuousIntegrationTools.isRunningOnContinuousIntegrationServer() && visualize);
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());
   }

   @AfterEach
   public void tearDown()
   {
      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   protected abstract HumanoidRobotInitialSetup createInitialSetup(HumanoidJointNameMap jointNameMap);

   @Test
   public void testStaticStances() throws Exception
   {
      DRCRobotModel robotModel = getRobotModel();
      StepReachabilityIOHelper stepReachabilityIOHelper = new StepReachabilityIOHelper();
      stepReachabilityIOHelper.loadStepReachability(robotModel);
      List<KinematicsToolboxSnapshotDescription> snapShots = stepReachabilityIOHelper.getReachabilityIKData();

      LogTools.info("Filtering feasible solutions");
      List<KinematicsToolboxSnapshotDescription> feasibleSolutions = snapShots.stream()
                                                                              .filter(snapshot -> snapshot.getIkSolution()
                                                                                                          .getSolutionQuality() < solutionQualityThreshold)
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
         performStanceCheck(robotModel, feasibleSolutions);
      }
   }

   private void performStanceCheck(DRCRobotModel robotModel, List<KinematicsToolboxSnapshotDescription> feasibleSolutions)
   {
      SCS2AvatarTestingSimulation simulationTestHelper = null;
      try
      {
         LogTools.info("Number of feasible solutions: " + feasibleSolutions.size());

         int indexToTest = random.nextInt(feasibleSolutions.size());
         LogTools.info("Random index chosen: " + indexToTest);

         KinematicsToolboxSnapshotDescription snapshotToTest = feasibleSolutions.get(indexToTest);
         feasibleSolutions.remove(indexToTest);

         IDLSequence.Float jointAngles = snapshotToTest.getIkSolution().getDesiredJointAngles();
         Point3D rootPosition = snapshotToTest.getIkSolution().getDesiredRootPosition();
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

         PlanarRegionsList planarRegionsList = generator.getPlanarRegionsList();
         PlanarRegionsListDefinedEnvironment environment = new PlanarRegionsListDefinedEnvironment(planarRegionsList, 0.015, false);

         HumanoidRobotInitialSetup initialSetup = createInitialSetup(robotModel.getJointMap());
         initialSetup.getRootJointPosition().set(rootPosition);
         initialSetup.getRootJointOrientation().set(rootOrientation);

         for (int i = 0; i < ikJoints.length; i++)
         {
            initialSetup.getJointPositions().put(ikJoints[i].getName(), ikJoints[i].getQ());
         }

         SCS2AvatarTestingSimulationFactory simulationTestHelperFactory = SCS2AvatarTestingSimulationFactory.createDefaultTestSimulationFactory(robotModel,
                                                                                                                                                environment,
                                                                                                                                                simulationTestingParameters);
         simulationTestHelperFactory.setRobotInitialSetup(initialSetup);
         simulationTestHelper = simulationTestHelperFactory.createAvatarTestingSimulation();
         simulationTestHelper.start();

         simulationTestHelper.simulateNow(2 * robotModel.getControllerDT());
         holdCurrentPosition(simulationTestHelper, fullRobotModel);
         boolean success = simulationTestHelper.simulateNow(simulationTime);

         if (!visualize)
         {
            Assertions.assertTrue(success);
         }
      }
      finally
      {
         if (simulationTestHelper != null)
            simulationTestHelper.finishTest();
      }
   }

   private void holdCurrentPosition(SCS2AvatarTestingSimulation simulationTestHelper, FullHumanoidRobotModel fullRobotModel)
   {
      long sequenceId = 10;

      FramePoint3D pelvisPosition = new FramePoint3D(fullRobotModel.getRootJoint().getFrameAfterJoint());
      pelvisPosition.changeFrame(ReferenceFrame.getWorldFrame());
      PelvisHeightTrajectoryMessage pelvisHeightTrajectoryMessage = HumanoidMessageTools.createPelvisHeightTrajectoryMessage(0.001, pelvisPosition.getZ());
      pelvisHeightTrajectoryMessage.setEnableUserPelvisControl(true);
      pelvisHeightTrajectoryMessage.setSequenceId(sequenceId++);
      simulationTestHelper.publishToController(pelvisHeightTrajectoryMessage);

      for (RobotSide robotSide : RobotSide.values)
      {
         FramePose3D handPose = new FramePose3D(fullRobotModel.getHandControlFrame(robotSide));
         handPose.changeFrame(ReferenceFrame.getWorldFrame());
         HandTrajectoryMessage handTrajectoryMessage = HumanoidMessageTools.createHandTrajectoryMessage(robotSide,
                                                                                                        0.001,
                                                                                                        handPose,
                                                                                                        ReferenceFrame.getWorldFrame());
         handTrajectoryMessage.setForceExecution(true);
         handTrajectoryMessage.setSequenceId(sequenceId++);
         simulationTestHelper.publishToController(handTrajectoryMessage);
      }

      FramePose3D headPose = new FramePose3D(fullRobotModel.getHead().getBodyFixedFrame());
      headPose.changeFrame(ReferenceFrame.getWorldFrame());
      HeadTrajectoryMessage headTrajectoryMessage = HumanoidMessageTools.createHeadTrajectoryMessage(0.001,
                                                                                                     headPose.getOrientation(),
                                                                                                     ReferenceFrame.getWorldFrame(),
                                                                                                     ReferenceFrame.getWorldFrame());
      headTrajectoryMessage.setSequenceId(sequenceId++);
      simulationTestHelper.publishToController(headTrajectoryMessage);
   }
}
