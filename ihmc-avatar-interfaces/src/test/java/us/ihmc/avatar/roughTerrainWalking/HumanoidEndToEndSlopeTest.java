package us.ihmc.avatar.roughTerrainWalking;

import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.TestInfo;

import controller_msgs.msg.dds.ChestTrajectoryMessage;
import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepDataMessage;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.initialSetup.OffsetAndYawRobotInitialSetup;
import us.ihmc.avatar.testTools.EndToEndTestTools;
import us.ihmc.avatar.testTools.scs2.SCS2AvatarTestingSimulation;
import us.ihmc.avatar.testTools.scs2.SCS2AvatarTestingSimulationFactory;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.trajectories.TrajectoryType;
import us.ihmc.scs2.simulation.parameters.ContactParameters;
import us.ihmc.simulationConstructionSetTools.util.environments.CommonAvatarEnvironmentInterface;
import us.ihmc.simulationConstructionSetTools.util.ground.CombinedTerrainObject3D;
import us.ihmc.simulationconstructionset.util.ground.TerrainObject3D;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;

public abstract class HumanoidEndToEndSlopeTest implements MultiRobotTestInterface
{
   private static final boolean EXPORT_TORQUE_SPEED_DATA = false;
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();

   private SCS2AvatarTestingSimulation simulationTestHelper;

   @BeforeEach
   public void showMemoryUsageBeforeTest()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
   }

   @AfterEach
   public void destroySimulationAndRecycleMemory()
   {
      if (simulationTestHelper != null)
      {
         simulationTestHelper.finishTest();
         simulationTestHelper = null;
      }

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }

   public void testSlope(TestInfo testInfo,
                        boolean up,
                        boolean useSideSteps,
                        double swingDuration,
                        double transferDuration,
                        double maxStepLength,
                        double heightOffset,
                        double torsoPitch,
                        boolean useExperimentalPhysicsEngine,
                        boolean disableToeOff)
        throws Exception
   {
      double slopeAngle = Math.toRadians(30.0);
      testSlope(testInfo, up, useSideSteps, swingDuration, transferDuration, maxStepLength, heightOffset, torsoPitch, useExperimentalPhysicsEngine, disableToeOff, slopeAngle);
   }


   public void testSlope(TestInfo testInfo,
                         boolean up,
                         boolean useSideSteps,
                         double swingDuration,
                         double transferDuration,
                         double maxStepLength,
                         double heightOffset,
                         double torsoPitch,
                         boolean useExperimentalPhysicsEngine,
                         boolean disableToeOff,
                         double slopeAngle)
         throws Exception
   {
      DRCRobotModel robotModel = getRobotModel();
      double footLength = robotModel.getWalkingControllerParameters().getSteppingParameters().getActualFootLength();
      double footWidth = robotModel.getWalkingControllerParameters().getSteppingParameters().getActualFootWidth();
      double slopeLength = 3.0;
      double topZ = Math.tan(slopeAngle) * slopeLength;
      double startX = (up ? 0.0 : 1.2 + slopeLength) + 0.3;
      double startZ = up ? 0.0 : topZ;
      double startYaw = useSideSteps ? -Math.PI / 2.0 : 0.0;

      SlopeEnvironment environment = new SlopeEnvironment(slopeAngle, 1.5, slopeLength);
      SCS2AvatarTestingSimulationFactory simulationTestHelperFactory = SCS2AvatarTestingSimulationFactory.createDefaultTestSimulationFactory(robotModel,
                                                                                                                                             environment,
                                                                                                                                             simulationTestingParameters);
      simulationTestHelperFactory.setStartingLocationOffset(new OffsetAndYawRobotInitialSetup(startX, 0, startZ, startYaw));
      simulationTestHelperFactory.setUseImpulseBasedPhysicsEngine(useExperimentalPhysicsEngine);
      if (useExperimentalPhysicsEngine)
      {
         ContactParameters contactParameters = ContactParameters.defaultIneslasticContactParameters(true);
         contactParameters.setCoefficientOfFriction(0.9);
         contactParameters.setCoulombMomentFrictionRatio(0.8);
         simulationTestHelperFactory.setImpulseBasedPhysicsEngineContactParameters(contactParameters);
      }
      simulationTestHelper = simulationTestHelperFactory.createAvatarTestingSimulation();
      simulationTestHelper.start();

      simulationTestHelper.setCameraPosition(startX, -slopeLength, 0.8 + startZ);

      if (disableToeOff)
      {
         simulationTestHelper.findVariable("GeometricToeOffManager", "doToeOffIfPossibleInDoubleSupport").setValueFromDouble(0);
         simulationTestHelper.findVariable("LegJointLimitsInspector", "doToeOffWhenHittingAnkleLimit").setValueFromDouble(0);
      }

      assertTrue(simulationTestHelper.simulateNow(0.5));
      publishHeightOffset(heightOffset);
      pitchTorsoForward(torsoPitch);

      double slopeStartX = up ? 0.6 : 1.8 + slopeLength;
      double slopeEndX = slopeStartX + slopeLength;
      double slopeStartZ = up ? 0.0 : topZ;
      double stanceWidth = 0.30;
      FootstepDataListMessage footsteps;

      if (useSideSteps)
      {
         double minStanceWidth = 0.20;
         footsteps = createSlopeSideFootsteps(slopeStartX,
                                              slopeEndX,
                                              slopeStartZ,
                                              up ? -slopeAngle : slopeAngle,
                                              footWidth,
                                              stanceWidth,
                                              minStanceWidth,
                                              maxStepLength);
      }
      else
      {
         footsteps = createSlopeFootsteps(slopeStartX, slopeEndX, slopeStartZ, up ? -slopeAngle : slopeAngle, footLength, stanceWidth, maxStepLength);
         computeSwingWaypoints(robotModel, footsteps);
      }
      setStepDurations(footsteps, swingDuration, transferDuration);

      simulationTestHelper.setInPoint();

      publishFootstepsAndSimulate(robotModel, footsteps);

      if (EXPORT_TORQUE_SPEED_DATA)
      {
         EndToEndTestTools.exportTorqueSpeedCurves(simulationTestHelper,
                                                   EndToEndTestTools.getDataOutputFolder(robotModel.getSimpleRobotName(), null),
                                                   testInfo.getTestMethod().get().getName());
      }
   }

   private void publishHeightOffset(double heightOffset) throws Exception
   {
      if (!Double.isFinite(heightOffset) || EuclidCoreTools.epsilonEquals(0.0, heightOffset, 1.0e-3))
         return;

      MovingReferenceFrame rootJointFrame = simulationTestHelper.getControllerFullRobotModel().getRootJoint().getFrameAfterJoint();
      double z = rootJointFrame.getTransformToRoot().getTranslationZ();
      simulationTestHelper.publishToController(HumanoidMessageTools.createPelvisHeightTrajectoryMessage(0.5, z + heightOffset));
      assertTrue(simulationTestHelper.simulateNow(0.5));
   }

   private void pitchTorsoForward(double angle) throws Exception
   {
      if (!Double.isFinite(angle) || EuclidCoreTools.epsilonEquals(0.0, angle, 1.0e-3))
         return;

      ChestTrajectoryMessage message = HumanoidMessageTools.createChestTrajectoryMessage(0.5,
                                                                                         new YawPitchRoll(0.0, angle, 0.0),
                                                                                         ReferenceFrame.getWorldFrame());
      simulationTestHelper.publishToController(message);
      assertTrue(simulationTestHelper.simulateNow(0.5));
   }

   private void computeSwingWaypoints(DRCRobotModel robotModel, FootstepDataListMessage footsteps)
   {
      double swingHeight = 1.5 * robotModel.getWalkingControllerParameters().getSwingTrajectoryParameters().getDefaultSwingHeight();

      for (int i = 2; i < footsteps.getFootstepDataList().size(); i++)
      {
         FootstepDataMessage previousFootstep = footsteps.getFootstepDataList().get(i - 2);
         FootstepDataMessage footstep = footsteps.getFootstepDataList().get(i);
         Point3D start = new Point3D(previousFootstep.getLocation());
         Point3D end = new Point3D(footstep.getLocation());
         Point3D firstWaypoint = new Point3D();
         Point3D secondWaypoint = new Point3D();
         firstWaypoint.interpolate(start, end, 0.25);
         secondWaypoint.interpolate(start, end, 0.90);
         Vector3D startNormal = new Vector3D(Axis3D.Z);
         previousFootstep.getOrientation().transform(startNormal);
         Vector3D endNormal = new Vector3D(Axis3D.Z);
         footstep.getOrientation().transform(endNormal);

         firstWaypoint.scaleAdd(swingHeight, startNormal, firstWaypoint);
         secondWaypoint.scaleAdd(swingHeight, endNormal, secondWaypoint);

         footstep.getCustomPositionWaypoints().add().set(firstWaypoint);
         footstep.getCustomPositionWaypoints().add().set(secondWaypoint);
         footstep.setTrajectoryType(TrajectoryType.CUSTOM.toByte());
      }
   }

   private static FootstepDataListMessage setStepDurations(FootstepDataListMessage message, double swingDuration, double transferDuration)
   {
      if (Double.isFinite(swingDuration) && swingDuration > 0.0)
      {
         for (int i = 0; i < message.getFootstepDataList().size(); i++)
         {
            message.getFootstepDataList().get(i).setSwingDuration(swingDuration);
         }
      }

      if (Double.isFinite(transferDuration) && transferDuration > 0.0)
      {
         for (int i = 0; i < message.getFootstepDataList().size(); i++)
         {
            message.getFootstepDataList().get(i).setTransferDuration(transferDuration);
         }
      }

      return message;
   }

   private void publishFootstepsAndSimulate(DRCRobotModel robotModel, FootstepDataListMessage footsteps) throws Exception
   {
      double walkingDuration = EndToEndTestTools.computeWalkingDuration(footsteps, robotModel.getWalkingControllerParameters());
      simulationTestHelper.publishToController(footsteps);
      assertTrue(simulationTestHelper.simulateNow(1.1 * walkingDuration));
   }

   private static FootstepDataListMessage createSlopeFootsteps(double xSlopeStart,
                                                               double xSlopeEnd,
                                                               double zSlopeStart,
                                                               double slopeAngle,
                                                               double footLength,
                                                               double stanceWidth,
                                                               double maxStepLength)
   {
      FootstepDataListMessage footsteps = new FootstepDataListMessage();
      double margin = 0.035;

      for (RobotSide stepSide : RobotSide.values)
      {
         double x = xSlopeStart - 0.5 * footLength - margin;
         double y = 0.5 * stepSide.negateIfRightSide(stanceWidth);
         double z = zSlopeStart;
         addFootstep(footsteps, stepSide, x, y, z);
      }

      double slopeLength = (xSlopeEnd - xSlopeStart) / Math.cos(slopeAngle);
      int numberOfSteps = (int) Math.ceil((slopeLength - footLength - 2.0 * margin) / maxStepLength) + 1;
      double stepLength = (slopeLength - footLength - 2.0 * margin) / (numberOfSteps - 1.0);
      assertTrue(stepLength <= maxStepLength);

      Pose3D footstepPose = new Pose3D();
      footstepPose.appendTranslation(xSlopeStart, 0.0, zSlopeStart);
      footstepPose.appendPitchRotation(slopeAngle);
      footstepPose.appendTranslation(0.5 * footLength + margin, 0.0, 0.0);

      RobotSide stepSide = RobotSide.LEFT;

      for (int i = 0; i < numberOfSteps; i++)
      {
         footstepPose.getPosition().setY(stepSide.negateIfRightSide(0.5 * stanceWidth));

         addFootstep(footsteps, stepSide, footstepPose);

         footstepPose.appendTranslation(stepLength, 0.0, 0.0);

         stepSide = stepSide.getOppositeSide();
      }

      double zSlopeEnd = zSlopeStart - slopeLength * Math.sin(slopeAngle);

      for (int i = 0; i < 2; i++)
      {
         double x = xSlopeEnd + 0.5 * footLength + margin;
         double y = 0.5 * stepSide.negateIfRightSide(stanceWidth);
         double z = zSlopeEnd;
         addFootstep(footsteps, stepSide, x, y, z);

         stepSide = stepSide.getOppositeSide();
      }

      return footsteps;
   }

   private static FootstepDataListMessage createSlopeSideFootsteps(double xSlopeStart,
                                                                   double xSlopeEnd,
                                                                   double zSlopeStart,
                                                                   double slopeAngle,
                                                                   double footWidth,
                                                                   double stanceWidth,
                                                                   double minStanceWidth,
                                                                   double maxStepLength)
   {
      FootstepDataListMessage footsteps = new FootstepDataListMessage();
      double margin = 0.035;

      // 2 steps up to right before the beginning of the slope
      for (RobotSide stepSide : RobotSide.values)
      {
         double x = xSlopeStart - margin - 0.5 * footWidth - 0.5 * minStanceWidth + stepSide.negateIfRightSide(0.5 * minStanceWidth);
         double y = 0.0;
         double z = zSlopeStart;

         addFootstep(footsteps, stepSide, x, y, z);
      }

      SideDependentList<Pose3D> footPoses = new SideDependentList<>();
      RobotSide stepSide = RobotSide.LEFT;

      Pose3D pose = new Pose3D();
      // left step on the slope, right step right before the slope
      pose.appendTranslation(xSlopeStart, 0.0, zSlopeStart);
      pose.appendPitchRotation(slopeAngle);
      pose.appendTranslation(0.5 * footWidth + margin, 0, 0);
      footPoses.put(stepSide, pose);
      addFootstep(footsteps, stepSide, pose);

      stepSide = stepSide.getOppositeSide();
      pose = new Pose3D();
      pose.appendTranslation(xSlopeStart - 0.5 * footWidth - margin, 0.0, zSlopeStart);
      footPoses.put(stepSide, pose);
      addFootstep(footsteps, stepSide, pose);

      // 2 steps up to right after the beginning of the slope
      stepSide = stepSide.getOppositeSide();
      pose = footPoses.get(stepSide);
      pose.appendTranslation(minStanceWidth, 0, 0);
      addFootstep(footsteps, stepSide, pose);

      stepSide = stepSide.getOppositeSide();
      pose = footPoses.get(stepSide);
      pose.appendTranslation(0.5 * footWidth + margin, 0.0, 0.0);
      pose.appendPitchRotation(slopeAngle);
      pose.appendTranslation(0.5 * footWidth + margin, 0, 0);
      addFootstep(footsteps, stepSide, pose);

      double slopeLength = (xSlopeEnd - xSlopeStart) / Math.cos(slopeAngle);
      int numberOfSteps = (int) Math.ceil((slopeLength - footWidth - 2.0 * margin - minStanceWidth) / maxStepLength);
      double stepLength = (slopeLength - footWidth - 2.0 * margin - minStanceWidth) / (numberOfSteps);
      assertTrue(stepLength <= maxStepLength);

      stepSide = stepSide.getOppositeSide();

      for (int i = 0; i < 2 * numberOfSteps; i++)
      {
         footPoses.get(stepSide).appendTranslation(stepLength, 0.0, 0.0);
         addFootstep(footsteps, stepSide, footPoses.get(stepSide));

         stepSide = stepSide.getOppositeSide();
      }

      double zSlopeEnd = zSlopeStart - slopeLength * Math.sin(slopeAngle);
      addFootstep(footsteps, stepSide, xSlopeEnd + margin + 0.5 * footWidth + minStanceWidth, 0.0, zSlopeEnd);

      stepSide = stepSide.getOppositeSide();
      addFootstep(footsteps, stepSide, xSlopeEnd + margin + 0.5 * footWidth, 0.0, zSlopeEnd);

      // Yaw all the footsteps
      for (int i = 0; i < footsteps.getFootstepDataList().size(); i++)
      {
         footsteps.getFootstepDataList().get(i).getOrientation().appendYawRotation(-0.5 * Math.PI);
      }

      return footsteps;
   }

   private static void addFootstep(FootstepDataListMessage footsteps, RobotSide stepSide, Pose3DReadOnly pose)
   {
      FootstepDataMessage footstep = footsteps.getFootstepDataList().add();
      footstep.setRobotSide(stepSide.toByte());
      footstep.getLocation().set(pose.getPosition());
      footstep.getOrientation().set(pose.getOrientation());
   }

   private static void addFootstep(FootstepDataListMessage footsteps, RobotSide stepSide, double x, double y, double z)
   {
      FootstepDataMessage footstep = footsteps.getFootstepDataList().add();
      footstep.setRobotSide(stepSide.toByte());
      footstep.getLocation().set(x, y, z);
   }

   private static class SlopeEnvironment implements CommonAvatarEnvironmentInterface
   {
      private final CombinedTerrainObject3D terrainObject = new CombinedTerrainObject3D("Slope");

      public SlopeEnvironment(double slopeAngle, double slopeWidth, double slopeLength)
      {
         double startingBlockLength = 1.2;
         double xStart = -0.5 * startingBlockLength;
         double xEnd = 0.5 * startingBlockLength + 0 * slopeLength;
         double yStart = -0.5 * slopeWidth;
         double yEnd = 0.5 * slopeWidth;
         double zStart = -0.10;
         double zEnd = 0.0;
         terrainObject.addBox(xStart, yStart, xEnd, yEnd, zStart, zEnd);

         xStart = 0.5 * startingBlockLength;
         xEnd = 0.5 * startingBlockLength + slopeLength;
         double height = slopeLength * Math.tan(slopeAngle);
         terrainObject.addRamp(xStart, yStart, xEnd, yEnd, height, YoAppearance.Orange());

         xStart = xEnd;
         xEnd += startingBlockLength;
         zEnd = height;
         terrainObject.addBox(xStart, yStart, xEnd, yEnd, zStart, zEnd);

         xStart = xEnd + slopeLength;
         terrainObject.addRamp(xStart, yStart, xEnd, yEnd, height, YoAppearance.AliceBlue());

         xEnd = xStart + startingBlockLength;
         xStart -= slopeLength;
         zEnd = 0.0;
         terrainObject.addBox(xStart, yStart, xEnd, yEnd, zStart, zEnd);
      }

      @Override
      public TerrainObject3D getTerrainObject3D()
      {
         return terrainObject;
      }
   }
}
