package us.ihmc.atlas.controllerAPI;

import static org.junit.Assert.assertTrue;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.DRCObstacleCourseStartingLocation;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.DRCRobotModel.RobotTarget;
import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.desiredFootStep.ComponentBasedDesiredFootstepCalculator;
import us.ihmc.commonWalkingControlModules.desiredHeadingAndVelocity.ManualDesiredVelocityControlModule;
import us.ihmc.commonWalkingControlModules.desiredHeadingAndVelocity.RateBasedDesiredHeadingControlModule;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ContactableBodiesFactory;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactableFoot;
import us.ihmc.humanoidRobotics.communication.packets.ExecutionMode;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.ArmTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataListMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataMessage;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.math.trajectories.waypoints.SimpleTrajectoryPoint1D;
import us.ihmc.robotics.math.trajectories.waypoints.SimpleTrajectoryPoint1DList;
import us.ihmc.robotics.math.trajectories.waypoints.TrajectoryPoint1DCalculator;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.simulationconstructionset.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.bambooTools.SimulationTestingParameters;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.tools.MemoryTools;
import us.ihmc.tools.thread.ThreadTools;
import us.ihmc.wholeBodyController.RobotContactPointParameters;

public class AtlasUpperBodyTrajectoriesWhileWalkingTest
{
   protected static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromEnvironmentVariables();
   static
   {
      //TODO: remove this once state estimator parameters are tuned
      simulationTestingParameters.setUsePefectSensors(true);
      simulationTestingParameters.setKeepSCSUp(true);
   }

   private static final double EPSILON_FOR_DESIREDS = 1.0e-10;

   protected DRCSimulationTestHelper drcSimulationTestHelper;

   @ContinuousIntegrationTest(categoriesOverride = IntegrationCategory.FAST, estimatedDuration = 31.3)
   @Test(timeout = 160000)
   public void testWalkingWithRandomArmTrajectoryMovements() throws Exception
   {
      Random random = new Random(564654L);
      DRCRobotModel robotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_DUAL_ROBOTIQ, RobotTarget.SCS, false);
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.DEFAULT;

      drcSimulationTestHelper = new DRCSimulationTestHelper(getClass().getSimpleName(), selectedLocation, simulationTestingParameters, robotModel);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5);
      assertTrue(success);

      FullHumanoidRobotModel fullRobotModel = drcSimulationTestHelper.getControllerFullRobotModel();
      fullRobotModel.updateFrames();
      HumanoidReferenceFrames referenceFrames = new HumanoidReferenceFrames(fullRobotModel);

      YoVariableRegistry registry = drcSimulationTestHelper.getYovariableRegistry();
      double timeToCompleteWalking = sendWalkingPacket(robotModel, fullRobotModel, referenceFrames, registry);
      sendArmTrajectoryMessageWithRandomPoints(random, robotModel, fullRobotModel);

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(timeToCompleteWalking);
      assertTrue(success);
   }
   
   @ContinuousIntegrationTest(categoriesOverride = IntegrationCategory.FAST, estimatedDuration = 31.3)
   @Test(timeout = 160000)
   public void testWalkingWithArmsHoldingInFeetFrame() throws Exception
   {
      Random random = new Random(564654L);
      DRCRobotModel robotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_DUAL_ROBOTIQ, RobotTarget.SCS, false);
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());
      
      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.DEFAULT;
      
      drcSimulationTestHelper = new DRCSimulationTestHelper(getClass().getSimpleName(), selectedLocation, simulationTestingParameters, robotModel);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(2.5);
      assertTrue(success);
      
      FullHumanoidRobotModel fullRobotModel = drcSimulationTestHelper.getControllerFullRobotModel();
      fullRobotModel.updateFrames();
      HumanoidReferenceFrames referenceFrames = new HumanoidReferenceFrames(fullRobotModel);
      
      YoVariableRegistry registry = drcSimulationTestHelper.getYovariableRegistry();
      double timeToCompleteWalking = sendWalkingPacket(robotModel, fullRobotModel, referenceFrames, registry);
      
      
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
      for (RobotSide robotSide : RobotSide.values) 
      {
         ReferenceFrame handFrame = referenceFrames.getHandFrame(robotSide);
         FramePose handPosition = new FramePose();
         handPosition.setToZero(handFrame);
         handPosition.changeFrame(worldFrame);
         Point3D position = new Point3D();
         handPosition.getPosition(position);
         Quaternion orientation = new Quaternion();
         handPosition.getOrientation(orientation);
         
         HandTrajectoryMessage handHoldMessage = new HandTrajectoryMessage(robotSide, 1);
         handHoldMessage.setTrajectoryReferenceFrameId(referenceFrames.getAnkleZUpFrame(robotSide.getOppositeSide()));
         handHoldMessage.setExpressedInReferenceFrameId(worldFrame);
         Vector3D zeroVelocity = new Vector3D();
         handHoldMessage.setTrajectoryPoint(0, 11.0, position, orientation, zeroVelocity, zeroVelocity, worldFrame);
         drcSimulationTestHelper.send(handHoldMessage);
      }
      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(timeToCompleteWalking);
      assertTrue(success);
   }

   private void sendArmTrajectoryMessageWithRandomPoints(Random random, DRCRobotModel robotModel, FullHumanoidRobotModel fullRobotModel)
         throws SimulationExceededMaximumTimeException
   {
      boolean success;
      long id = 1264L;
      double timePerWaypoint = 0.5;
      int numberOfMessages = 5;
      int numberOfTrajectoryPoints = 5;
      double trajectoryTime = (numberOfTrajectoryPoints + 1) * timePerWaypoint;
      
      SideDependentList<OneDoFJoint[]> armsJoints = new SideDependentList<>();
      SideDependentList<List<ArmTrajectoryMessage>> armTrajectoryMessages = new SideDependentList<>();
      
      for (RobotSide robotSide : RobotSide.values)
      {
         RigidBody chest = fullRobotModel.getChest();
         RigidBody hand = fullRobotModel.getHand(robotSide);
         OneDoFJoint[] armJoints = ScrewTools.createOneDoFJointPath(chest, hand);
         armsJoints.put(robotSide, armJoints);
         int numberOfJoints = ScrewTools.computeDegreesOfFreedom(armJoints);

         List<ArmTrajectoryMessage> messageList = new ArrayList<>();

         for (int messageIndex = 0; messageIndex < numberOfMessages; messageIndex++)
         {
            ArmTrajectoryMessage armTrajectoryMessage = new ArmTrajectoryMessage(robotSide, numberOfJoints, numberOfTrajectoryPoints);
            armTrajectoryMessage.setUniqueId(id);
            if (messageIndex > 0)
               armTrajectoryMessage.setExecutionMode(ExecutionMode.QUEUE, id - 1);
            id++;
            
            TrajectoryPoint1DCalculator trajectoryPoint1DCalculator = new TrajectoryPoint1DCalculator();

            for (int jointIndex = 0; jointIndex < numberOfJoints; jointIndex++)
            {
               OneDoFJoint joint = armJoints[jointIndex];

               trajectoryPoint1DCalculator.clear();

               for (int trajectoryPointIndex = 0; trajectoryPointIndex < numberOfTrajectoryPoints; trajectoryPointIndex++)
               {
                  double desiredJointPosition = RandomNumbers.nextDouble(random, joint.getJointLimitLower(), joint.getJointLimitUpper());
                  trajectoryPoint1DCalculator.appendTrajectoryPoint(desiredJointPosition);
               }

               trajectoryPoint1DCalculator.computeTrajectoryPointTimes(timePerWaypoint, trajectoryTime);
               trajectoryPoint1DCalculator.computeTrajectoryPointVelocities(true);
               SimpleTrajectoryPoint1DList trajectoryData = trajectoryPoint1DCalculator.getTrajectoryData();

               for (int trajectoryPointIndex = 0; trajectoryPointIndex < numberOfTrajectoryPoints; trajectoryPointIndex++)
               {
                  SimpleTrajectoryPoint1D trajectoryPoint = trajectoryData.getTrajectoryPoint(trajectoryPointIndex);
                  armTrajectoryMessage.setTrajectoryPoint(jointIndex, trajectoryPointIndex, trajectoryPoint.getTime(), trajectoryPoint.getPosition(),
                        trajectoryPoint.getVelocity());
               }
            }
            messageList.add(armTrajectoryMessage);
            drcSimulationTestHelper.send(armTrajectoryMessage);

            success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(robotModel.getControllerDT());
            assertTrue(success);
         }
         armTrajectoryMessages.put(robotSide, messageList);
      }
   }

   private double sendWalkingPacket(DRCRobotModel robotModel, FullHumanoidRobotModel fullRobotModel, HumanoidReferenceFrames referenceFrames,
         YoVariableRegistry registry)
   {
      WalkingControllerParameters walkingControllerParameters = robotModel.getWalkingControllerParameters();
      double swingTime = walkingControllerParameters.getDefaultSwingTime();
      double transferTime = walkingControllerParameters.getDefaultTransferTime();
      double stepTime = swingTime + transferTime;
      
      RateBasedDesiredHeadingControlModule desiredHeadingControlModule = new RateBasedDesiredHeadingControlModule(0.0, robotModel.getControllerDT(), registry);
      ManualDesiredVelocityControlModule desiredVelocityControlModule = new ManualDesiredVelocityControlModule(
            desiredHeadingControlModule.getDesiredHeadingFrame(), registry);

      RobotContactPointParameters contactPointParameters = robotModel.getContactPointParameters();
      ContactableBodiesFactory contactableBodiesFactory = contactPointParameters.getContactableBodiesFactory();
      SideDependentList<ContactableFoot> bipedFeet = contactableBodiesFactory.createFootContactableBodies(fullRobotModel, referenceFrames);

      ComponentBasedDesiredFootstepCalculator desiredFootstepCalculator = new ComponentBasedDesiredFootstepCalculator(referenceFrames.getPelvisZUpFrame(),
            bipedFeet, desiredHeadingControlModule, desiredVelocityControlModule, registry);

      desiredVelocityControlModule.setDesiredVelocity(new FrameVector2d(ReferenceFrame.getWorldFrame(), 0.15, 0.0, "desiredVelocityControlModule"));
      desiredFootstepCalculator.setInPlaceWidth(walkingControllerParameters.getInPlaceWidth());
      desiredFootstepCalculator.setMaxStepLength(walkingControllerParameters.getMaxStepLength());
      desiredFootstepCalculator.setMinStepWidth(walkingControllerParameters.getMinStepWidth());
      desiredFootstepCalculator.setMaxStepWidth(walkingControllerParameters.getMaxStepWidth());
      desiredFootstepCalculator.setStepPitch(walkingControllerParameters.getStepPitch());
      
      desiredFootstepCalculator.initialize();
      FootstepDataListMessage footsteps = computeNextFootsteps(RobotSide.LEFT, desiredFootstepCalculator, stepTime);
      footsteps.setDefaultSwingDuration(swingTime);
      footsteps.setDefaultTransferDuration(transferTime);
      
      int numberOfSteps = footsteps.getDataList().size();
      drcSimulationTestHelper.send(footsteps);
      
      int timeWalking = numberOfSteps;
      double timeToCompleteWalking = stepTime * timeWalking;
      return timeToCompleteWalking;
   }
   
   private FootstepDataListMessage computeNextFootsteps(RobotSide supportLeg, ComponentBasedDesiredFootstepCalculator componentBasedDesiredFootstepCalculator, double stepTime)
   {
      componentBasedDesiredFootstepCalculator.initializeDesiredFootstep(supportLeg, stepTime);
      FootstepDataMessage footStep = componentBasedDesiredFootstepCalculator.updateAndGetDesiredFootstep(supportLeg);
      FootstepDataListMessage footsteps = new FootstepDataListMessage(Double.NaN, Double.NaN);
      
      RobotSide robotSide = supportLeg;
      FootstepDataMessage previousFootStep = footStep;
      for (int i = 0; i < 30; i++)
      {
         footStep = componentBasedDesiredFootstepCalculator.predictFootstepAfterDesiredFootstep(robotSide, previousFootStep, stepTime * i, stepTime);
         footsteps.add(footStep);
         robotSide = robotSide.getOppositeSide();
         previousFootStep = footStep;
      }
      footsteps.setExecutionMode(ExecutionMode.OVERRIDE);

      return footsteps;
   }

   @Before
   public void showMemoryUsageBeforeTest()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
   }

   @After
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
}
