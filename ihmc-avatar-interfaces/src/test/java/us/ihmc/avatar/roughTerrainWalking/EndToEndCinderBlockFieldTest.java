package us.ihmc.avatar.roughTerrainWalking;

import static us.ihmc.robotics.Assert.assertTrue;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

import controller_msgs.msg.dds.FootstepDataListMessage;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.testTools.EndToEndTestTools;
import us.ihmc.avatar.testTools.scs2.SCS2AvatarTestingSimulation;
import us.ihmc.avatar.testTools.scs2.SCS2AvatarTestingSimulationFactory;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.scs2.simulation.parameters.ContactParameters;
import us.ihmc.scs2.simulation.physicsEngine.impulseBased.ImpulseBasedPhysicsEngine;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationConstructionSetTools.util.environments.CinderBlockFieldEnvironment;
import us.ihmc.simulationConstructionSetTools.util.environments.CinderBlockFieldEnvironment.CinderBlockStackDescription;
import us.ihmc.simulationConstructionSetTools.util.environments.CinderBlockFieldEnvironment.CinderBlockType;
import us.ihmc.simulationConstructionSetTools.util.environments.CommonAvatarEnvironmentInterface;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;

public abstract class EndToEndCinderBlockFieldTest implements MultiRobotTestInterface
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();

   private boolean useImpulseBasedPhysicsEngine = false;
   private SCS2AvatarTestingSimulation simulationTestHelper;

   public abstract double getStepHeightOffset();

   @BeforeEach
   public void showMemoryUsageBeforeTest()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
      useImpulseBasedPhysicsEngine = false;
   }

   @AfterEach
   public void destroySimulationAndRecycleMemory()
   {
      // Do this here in case a test fails. That way the memory will be recycled.
      if (simulationTestHelper != null)
      {
         simulationTestHelper.finishTest();
         simulationTestHelper = null;
      }

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }

   public boolean getUsePerfectSensors()
   {
      return false;
   }

   private void setupSimulation(CommonAvatarEnvironmentInterface environment)
   {
      SCS2AvatarTestingSimulationFactory simulationFactory = new SCS2AvatarTestingSimulationFactory(getRobotModel(), environment);
      simulationFactory.setDefaultHighLevelHumanoidControllerFactory();
      simulationFactory.setShowGUI(simulationTestingParameters.getCreateGUI());
      simulationFactory.setUsePerfectSensors(getUsePerfectSensors());
      simulationFactory.setRunMultiThreaded(simulationTestingParameters.getRunMultiThreaded());
      simulationFactory.setUseImpulseBasedPhysicsEngine(useImpulseBasedPhysicsEngine);
      simulationTestHelper = simulationFactory.createAvatarTestingSimulation();
   }

   @Test
   @Tag("fast")
   @Tag("video")
   public void testWalkingOverCinderBlockField() throws Exception
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      CinderBlockFieldEnvironment cinderBlockFieldEnvironment = new CinderBlockFieldEnvironment();
      cinderBlockFieldEnvironment.addFlatGround();
      List<List<Pose3D>> cinderBlockPoses = cinderBlockFieldEnvironment.addDRCCinderBlockField();
      FootstepDataListMessage footsteps = generateFootstepsForCinderBlockField(cinderBlockPoses, getStepHeightOffset());

      setupSimulation(cinderBlockFieldEnvironment);
      simulationTestHelper.start();

      ThreadTools.sleep(1000);
      boolean success = simulationTestHelper.simulateAndWait(0.5);
      assertTrue(success);

      FullHumanoidRobotModel fullRobotModel = simulationTestHelper.getControllerFullRobotModel();
      FramePoint3D pelvisPosition = new FramePoint3D(fullRobotModel.getPelvis().getBodyFixedFrame());
      pelvisPosition.changeFrame(ReferenceFrame.getWorldFrame());
      pelvisPosition.add(0.0, 0.0, getPelvisOffsetHeight());
      double desiredHeight = pelvisPosition.getZ();
      simulationTestHelper.publishToController(HumanoidMessageTools.createPelvisHeightTrajectoryMessage(0.5, desiredHeight));

      simulationTestHelper.publishToController(footsteps);

      WalkingControllerParameters walkingControllerParameters = getRobotModel().getWalkingControllerParameters();
      double stepTime = walkingControllerParameters.getDefaultSwingTime() + walkingControllerParameters.getDefaultTransferTime();
      double initialFinalTransfer = walkingControllerParameters.getDefaultInitialTransferTime();

      success = simulationTestHelper.simulateAndWait(footsteps.getFootstepDataList().size() * stepTime + 2.0 * initialFinalTransfer + 1.0);
      assertTrue(success);

      Point3D step1 = footsteps.getFootstepDataList().get(footsteps.getFootstepDataList().size() - 1).getLocation();
      Point3D step2 = footsteps.getFootstepDataList().get(footsteps.getFootstepDataList().size() - 2).getLocation();
      Point3D expectedPelvis = new Point3D();
      expectedPelvis.interpolate(step1, step2, 0.5);
      expectedPelvis.setZ(desiredHeight);
      Vector3D margin = new Vector3D(0.25, 0.25, 0.25);
      Point3D min = new Point3D(expectedPelvis);
      Point3D max = new Point3D(expectedPelvis);
      min.sub(margin);
      max.add(margin);
      simulationTestHelper.assertRobotsRootJointIsInBoundingBox(new BoundingBox3D(min, max));

      simulationTestHelper.createVideo(getSimpleRobotName(), 2);
   }

   public void testSteppingStonesA() throws Exception
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());
      simulationTestingParameters.setUsePefectSensors(getUsePerfectSensors());

      CinderBlockFieldEnvironment cinderBlockFieldEnvironment = new CinderBlockFieldEnvironment();
      cinderBlockFieldEnvironment.addFlatGround();
      List<List<Pose3D>> cinderBlockPoses = cinderBlockFieldEnvironment.addCustomCinderBlockField2D(steppingStonesA(new RigidBodyTransform(new Quaternion(),
                                                                                                                                           new Vector3D(0.5,
                                                                                                                                                        0.0,
                                                                                                                                                        0.0))));
      FootstepDataListMessage footsteps = generateFootstepsForSteppingStonesA(cinderBlockPoses, getStepHeightOffset());

      DRCRobotModel robotModel = getRobotModel();
      setupSimulation(cinderBlockFieldEnvironment);
      simulationTestHelper.start();
      simulationTestHelper.setCameraFocusPosition(1.6, 0.0, 1.0);
      simulationTestHelper.setCameraPosition(1.6, -6.0, 2.4);

      assertTrue(simulationTestHelper.simulateAndWait(0.5));
      simulationTestHelper.setBufferInPointIndexToCurrent();

      WalkingControllerParameters walkingControllerParameters = robotModel.getWalkingControllerParameters();
      simulationTestHelper.publishToController(footsteps);
      double simulationTime = 1.1 * EndToEndTestTools.computeWalkingDuration(footsteps, walkingControllerParameters);
      assertTrue(simulationTestHelper.simulateAndWait(simulationTime));
   }

   public void testSteppingStonesB() throws Exception
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());
      simulationTestingParameters.setUsePefectSensors(getUsePerfectSensors());

      CinderBlockFieldEnvironment cinderBlockFieldEnvironment = new CinderBlockFieldEnvironment();
      cinderBlockFieldEnvironment.addFlatGround();
      List<List<Pose3D>> cinderBlockPoses = cinderBlockFieldEnvironment.addCustomCinderBlockField2D(steppingStonesB(new RigidBodyTransform(new Quaternion(),
                                                                                                                                           new Vector3D(0.1,
                                                                                                                                                        0.8,
                                                                                                                                                        0.0))));
      FootstepDataListMessage footsteps = generateFootstepsForSteppingStonesB(cinderBlockPoses, getStepHeightOffset());

      DRCRobotModel robotModel = getRobotModel();
      useImpulseBasedPhysicsEngine = true;
      setupSimulation(cinderBlockFieldEnvironment);
      ImpulseBasedPhysicsEngine physicsEngine = (ImpulseBasedPhysicsEngine) simulationTestHelper.getSimulationSession().getPhysicsEngine();
      ContactParameters contactParameters = ContactParameters.defaultIneslasticContactParameters(true);
      contactParameters.setCoefficientOfFriction(0.80);
      contactParameters.setCoulombMomentFrictionRatio(0.6);
      physicsEngine.setGlobalContactParameters(contactParameters);
      simulationTestHelper.start();

      simulationTestHelper.setCameraFocusPosition(2.0, 1.3, 1.0);
      simulationTestHelper.setCameraPosition(6.0, 7.0, 3.25);

      assertTrue(simulationTestHelper.simulateAndWait(0.5));
      simulationTestHelper.setBufferInPointIndexToCurrent();

      WalkingControllerParameters walkingControllerParameters = robotModel.getWalkingControllerParameters();
      EndToEndTestTools.setStepDurations(footsteps, 1.5 * walkingControllerParameters.getDefaultSwingTime(), Double.NaN);
      simulationTestHelper.publishToController(footsteps);
      double simulationTime = 1.1 * EndToEndTestTools.computeWalkingDuration(footsteps, walkingControllerParameters);
      assertTrue(simulationTestHelper.simulateAndWait(simulationTime));
   }

   public void testSlantedCinderBlockField(boolean varyHeight) throws Exception
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());
      simulationTestingParameters.setUsePefectSensors(getUsePerfectSensors());

      Random random = new Random(674);

      CinderBlockFieldEnvironment cinderBlockFieldEnvironment = new CinderBlockFieldEnvironment();
      cinderBlockFieldEnvironment.addFlatGround();
      List<List<Pose3D>> cinderBlockPoses = cinderBlockFieldEnvironment.addCustomCinderBlockField2D(slantedCinderBlockLeveledField(random,
                                                                                                                                   new RigidBodyTransform(new Quaternion(),
                                                                                                                                                          new Vector3D(0.1,
                                                                                                                                                                       -0.2,
                                                                                                                                                                       0.0)),
                                                                                                                                   varyHeight));
      FootstepDataListMessage footsteps = generateFootstepsForSlantedCinderBlockLeveledField(cinderBlockPoses, getStepHeightOffset(), varyHeight);

      DRCRobotModel robotModel = getRobotModel();
      useImpulseBasedPhysicsEngine = true;
      setupSimulation(cinderBlockFieldEnvironment);
      ContactParameters contactParameters = ContactParameters.defaultIneslasticContactParameters(true);
      contactParameters.setCoefficientOfFriction(0.80);
      contactParameters.setCoulombMomentFrictionRatio(0.6);
      ImpulseBasedPhysicsEngine physicsEngine = (ImpulseBasedPhysicsEngine) simulationTestHelper.getSimulationSession().getPhysicsEngine();
      physicsEngine.setGlobalContactParameters(contactParameters);
      simulationTestHelper.start();

      simulationTestHelper.setCameraFocusPosition(0.0, 0.0, 0.9);
      simulationTestHelper.setCameraPosition(0.0, -6.0, 2.25);
      simulationTestHelper.requestCameraRigidBodyTracking(getSimpleRobotName(), simulationTestHelper.getControllerFullRobotModel().getPelvis().getName());

      assertTrue(simulationTestHelper.simulateAndWait(0.5));
      simulationTestHelper.setBufferInPointIndexToCurrent();

      WalkingControllerParameters walkingControllerParameters = robotModel.getWalkingControllerParameters();
      EndToEndTestTools.setStepDurations(footsteps, 1.5 * walkingControllerParameters.getDefaultSwingTime(), Double.NaN);
      for (int i = 0; i < footsteps.getFootstepDataList().size(); i++)
      {
         footsteps.getFootstepDataList().get(i).setSwingHeight(0.15);
      }
      simulationTestHelper.publishToController(footsteps);
      double simulationTime = 1.1 * EndToEndTestTools.computeWalkingDuration(footsteps, walkingControllerParameters);
      assertTrue(simulationTestHelper.simulateAndWait(simulationTime));
   }

   public void testSlantedCinderBlockAnkleRollLimit() throws Exception
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());
      simulationTestingParameters.setUsePefectSensors(getUsePerfectSensors());

      Random random = new Random(674);

      CinderBlockFieldEnvironment cinderBlockFieldEnvironment = new CinderBlockFieldEnvironment();
      cinderBlockFieldEnvironment.addFlatGround();
      List<List<Pose3D>> cinderBlockPoses = cinderBlockFieldEnvironment.addCustomCinderBlockField2D(slantedCinderBlockLeveledFieldForAnkleRollLimit(random,
                                                                                                                                                    new RigidBodyTransform(new Quaternion(),
                                                                                                                                                                           new Vector3D(0.1,
                                                                                                                                                                                        -0.2,
                                                                                                                                                                                        0.0)),
                                                                                                                                                    false));
      FootstepDataListMessage footsteps = generateFootstepsForSlantedCinderBlockLeveledField(cinderBlockPoses, getStepHeightOffset(), false);

      DRCRobotModel robotModel = getRobotModel();
      useImpulseBasedPhysicsEngine = true;
      setupSimulation(cinderBlockFieldEnvironment);
      ContactParameters contactParameters = ContactParameters.defaultIneslasticContactParameters(true);
      contactParameters.setCoefficientOfFriction(0.80);
      contactParameters.setCoulombMomentFrictionRatio(0.6);
      ImpulseBasedPhysicsEngine physicsEngine = (ImpulseBasedPhysicsEngine) simulationTestHelper.getSimulationSession().getPhysicsEngine();
      physicsEngine.setGlobalContactParameters(contactParameters);
      simulationTestHelper.start();

      simulationTestHelper.setCameraFocusPosition(0.0, 0.0, 0.9);
      simulationTestHelper.setCameraPosition(0.0, -6.0, 2.25);
      simulationTestHelper.requestCameraRigidBodyTracking(getSimpleRobotName(), simulationTestHelper.getControllerFullRobotModel().getPelvis().getName());

      assertTrue(simulationTestHelper.simulateAndWait(0.5));
      simulationTestHelper.setBufferInPointIndexToCurrent();

      WalkingControllerParameters walkingControllerParameters = robotModel.getWalkingControllerParameters();
      EndToEndTestTools.setStepDurations(footsteps, 1.5 * walkingControllerParameters.getDefaultSwingTime(), Double.NaN);
      for (int i = 0; i < footsteps.getFootstepDataList().size(); i++)
      {
         footsteps.getFootstepDataList().get(i).setSwingHeight(0.15);
      }
      simulationTestHelper.publishToController(footsteps);
      double simulationTime = 1.1 * EndToEndTestTools.computeWalkingDuration(footsteps, walkingControllerParameters);
      assertTrue(simulationTestHelper.simulateAndWait(simulationTime));
   }

   public abstract double getPelvisOffsetHeight();

   private static FootstepDataListMessage generateFootstepsForCinderBlockField(List<? extends List<? extends Pose3DReadOnly>> cinderBlockPoses, double zOffset)
   {
      FootstepDataListMessage footsteps = new FootstepDataListMessage();

      int numberOfColumns = cinderBlockPoses.get(0).size();

      int indexForLeftSide = (numberOfColumns - 1) / 2;
      int indexForRightSide = indexForLeftSide + 1;
      SideDependentList<List<Pose3DReadOnly>> columns = extractColumns(cinderBlockPoses, indexForLeftSide, indexForRightSide);

      for (int row = 0; row < cinderBlockPoses.size(); row++)
      {
         for (RobotSide robotSide : RobotSide.values)
         {
            addFootstepFromCBPose(footsteps, robotSide, columns.get(robotSide).get(row), 0.0, robotSide.negateIfLeftSide(0.06), zOffset, 0.0);
         }
      }

      return footsteps;
   }

   public static List<List<CinderBlockStackDescription>> steppingStonesA(RigidBodyTransformReadOnly startPose)
   {
      // @formatter:off
      int[][] stackSizes = new int[][]
      {
         {1, 1},
         {2, 0},
         {0, 1},
         {0, 1},
         {2, 0},
         {0, 2},
         {1, 1},
         {0, 0}
      };
      // @formatter:on

      CinderBlockType FLAT = CinderBlockType.FLAT;
      CinderBlockType SLFW = CinderBlockType.SLANTED_FORWARD;
      CinderBlockType SLBK = CinderBlockType.SLANTED_BACK;

      // @formatter:off
      CinderBlockType[][] types = new CinderBlockType[][]
      {
         {SLBK, SLBK},
         {FLAT, null},
         {null, FLAT},
         {null, FLAT},
         {FLAT, null},
         {null, FLAT},
         {SLFW, SLFW},
         {null, null},
      };
      // @formatter:on

      RigidBodyTransform centerBasePose = new RigidBodyTransform(startPose);
      centerBasePose.appendTranslation(0.5 * stackSizes.length * CinderBlockFieldEnvironment.cinderBlockLength, 0.0, 0.0);
      return CinderBlockStackDescription.grid2D(centerBasePose, stackSizes, types);
   }

   private static FootstepDataListMessage generateFootstepsForSteppingStonesA(List<? extends List<? extends Pose3DReadOnly>> cinderBlockPoses, double zOffset)
   {
      FootstepDataListMessage footsteps = new FootstepDataListMessage();

      SideDependentList<List<Pose3DReadOnly>> columns = extractColumns(cinderBlockPoses, 0, 1);
      List<Pose3DReadOnly> leftCinderBlocks = columns.get(RobotSide.LEFT);
      List<Pose3DReadOnly> rightCinderBlocks = columns.get(RobotSide.RIGHT);

      addFootstepFromCBPose(footsteps, RobotSide.RIGHT, rightCinderBlocks.get(0), 0.0, 0.08, zOffset, 0.0);
      addFootstepFromCBPose(footsteps, RobotSide.LEFT, leftCinderBlocks.get(0), 0.0, -0.08, zOffset, 0.0);
      addFootstepFromCBPose(footsteps, RobotSide.LEFT, leftCinderBlocks.get(1), 0.0, -0.08, zOffset, 0.0);
      addFootstepFromCBPose(footsteps, RobotSide.RIGHT, rightCinderBlocks.get(2), 0.15, 0.08, zOffset, 0.0);
      addFootstepFromCBPose(footsteps, RobotSide.LEFT, leftCinderBlocks.get(4), -0.04, -0.08, zOffset, 0.0);
      addFootstepFromCBPose(footsteps, RobotSide.RIGHT, rightCinderBlocks.get(5), 0.0, 0.08, zOffset, 0.0);
      addFootstepFromCBPose(footsteps, RobotSide.LEFT, leftCinderBlocks.get(6), 0.0, -0.08, zOffset, 0.0);
      addFootstepFromCBPose(footsteps, RobotSide.RIGHT, rightCinderBlocks.get(6), 0.0, 0.08, zOffset, 0.0);
      addFootstepFromCBPose(footsteps, RobotSide.LEFT, leftCinderBlocks.get(7), 0.0, -0.08, zOffset, 0.0);
      addFootstepFromCBPose(footsteps, RobotSide.RIGHT, rightCinderBlocks.get(7), 0.0, 0.08, zOffset, 0.0);

      return footsteps;
   }

   public static List<List<CinderBlockStackDescription>> steppingStonesB(RigidBodyTransformReadOnly startPose)
   {
      // @formatter:off
      int[][] stackSizes = new int[][]
      {
         {0, 0, 0, 0, 0, 0, 0},
         {1, 1, 0, 0, 0, 1, 1},
         {2, 0, 0, 0, 0, 2, 0},
         {0, 1, 0, 0, 2, 0, 2},
         {0, 1, 0, 0, 3, 2, 0},
         {2, 0, 0, 0, 0, 0, 2},
         {0, 2, 0, 3, 0, 2, 0},
         {1, 1, 2, 2, 2, 0, 2},
         {2, 0, 1, 0, 0, 1, 0}
      };
      // @formatter:on

      CinderBlockType FLAT = CinderBlockType.FLAT;
      CinderBlockType SLLE = CinderBlockType.SLANTED_LEFT;
      CinderBlockType SLFW = CinderBlockType.SLANTED_FORWARD;
      CinderBlockType SLRI = CinderBlockType.SLANTED_RIGHT;
      CinderBlockType SLBK = CinderBlockType.SLANTED_BACK;

      // @formatter:off
      CinderBlockType[][] types = new CinderBlockType[][]
      {
         {null, null, null, null, null, null, null},
         {SLBK, SLBK, null, null, null, SLBK, SLBK},
         {FLAT, null, null, null, null, SLRI, null},
         {null, FLAT, null, null, null, null, SLLE},
         {null, FLAT, null, null, null, SLBK, null},
         {FLAT, null, null, null, null, null, FLAT},
         {null, FLAT, SLFW, null, null, SLFW, null},
         {SLFW, SLFW, FLAT, FLAT, SLFW, null, SLLE},
         {FLAT, null, SLBK, null, null, FLAT, null},
      };
      // @formatter:on

      RigidBodyTransform centerBasePose = new RigidBodyTransform(startPose);
      centerBasePose.appendTranslation(0.5 * stackSizes.length * CinderBlockFieldEnvironment.cinderBlockLength, 0.0, 0.0);
      return CinderBlockStackDescription.grid2D(centerBasePose, stackSizes, types);
   }

   private static FootstepDataListMessage generateFootstepsForSteppingStonesB(List<? extends List<? extends Pose3DReadOnly>> cinderBlockPoses, double zOffset)
   {
      FootstepDataListMessage footsteps = new FootstepDataListMessage();

      double yawOffset = 0.0;
      addFootstepFromCBPose(footsteps, RobotSide.LEFT, cinderBlockPoses.get(1).get(5), 0.0, -0.08, zOffset, yawOffset);
      addFootstepFromCBPose(footsteps, RobotSide.RIGHT, cinderBlockPoses.get(1).get(6), 0.0, 0.08, zOffset, yawOffset);
      addFootstepFromCBPose(footsteps, RobotSide.LEFT, cinderBlockPoses.get(2).get(5), 0.0, -0.08, zOffset, yawOffset);
      addFootstepFromCBPose(footsteps, RobotSide.RIGHT, cinderBlockPoses.get(3).get(6), 0.0, 0.08, zOffset, yawOffset);
      addFootstepFromCBPose(footsteps, RobotSide.LEFT, cinderBlockPoses.get(4).get(5), 0.0, -0.08, zOffset, yawOffset);
      addFootstepFromCBPose(footsteps, RobotSide.RIGHT, cinderBlockPoses.get(5).get(6), 0.0, 0.08, zOffset, yawOffset);
      yawOffset = 0.25 * Math.PI;
      addFootstepFromCBPose(footsteps, RobotSide.LEFT, cinderBlockPoses.get(6).get(5), 0.0, 0.0, zOffset, yawOffset);
      yawOffset = 0.50 * Math.PI;
      addFootstepFromCBPose(footsteps, RobotSide.RIGHT, cinderBlockPoses.get(7).get(4), 0.0, 0.08, zOffset, yawOffset);
      addFootstepFromCBPose(footsteps, RobotSide.LEFT, cinderBlockPoses.get(6).get(3), 0.0, -0.08, zOffset, yawOffset);
      addFootstepFromCBPose(footsteps, RobotSide.RIGHT, cinderBlockPoses.get(7).get(2), 0.0, 0.08, zOffset, yawOffset);
      yawOffset = 0.75 * Math.PI;
      addFootstepFromCBPose(footsteps, RobotSide.LEFT, cinderBlockPoses.get(6).get(1), 0.0, 0.04, zOffset, yawOffset);
      yawOffset = Math.PI;
      addFootstepFromCBPose(footsteps, RobotSide.RIGHT, cinderBlockPoses.get(5).get(0), 0.0, 0.08, zOffset, yawOffset);

      addFootstepFromCBPose(footsteps, RobotSide.LEFT, cinderBlockPoses.get(4).get(1), 0.0, 0.08, zOffset, yawOffset);
      yawOffset = 0.85 * Math.PI;
      addFootstepFromCBPose(footsteps, RobotSide.RIGHT, cinderBlockPoses.get(3).get(1), 0.02, -0.08, zOffset, yawOffset);
      addFootstepFromCBPose(footsteps, RobotSide.LEFT, cinderBlockPoses.get(2).get(0), 0.0, 0.12, zOffset, yawOffset);
      addFootstepFromCBPose(footsteps, RobotSide.RIGHT, cinderBlockPoses.get(2).get(0), 0.0, -0.12, zOffset, yawOffset);
      yawOffset = Math.PI;
      addFootstepFromCBPose(footsteps, RobotSide.LEFT, cinderBlockPoses.get(1).get(1), 0.0, -0.08, zOffset, yawOffset);
      addFootstepFromCBPose(footsteps, RobotSide.RIGHT, cinderBlockPoses.get(1).get(0), 0.0, 0.08, zOffset, yawOffset);
      addFootstepFromCBPose(footsteps, RobotSide.LEFT, cinderBlockPoses.get(0).get(1), 0.0, -0.08, zOffset, yawOffset);
      addFootstepFromCBPose(footsteps, RobotSide.RIGHT, cinderBlockPoses.get(0).get(0), 0.0, 0.08, zOffset, yawOffset);

      return footsteps;
   }

   public static List<List<CinderBlockStackDescription>> slantedCinderBlockLeveledField(Random random, RigidBodyTransformReadOnly startPose, boolean varyHeight)
   {
      int width = 2;
      int length = 20;
      int[][] stackSizes = new int[length][width];
      CinderBlockType[][] types = new CinderBlockType[length][width];
      CinderBlockType[] allSlantedTypes = {CinderBlockType.SLANTED_LEFT, CinderBlockType.SLANTED_FORWARD, CinderBlockType.SLANTED_RIGHT,
            CinderBlockType.SLANTED_BACK};

      for (int i = 0; i < stackSizes.length; i++)
      {
         for (int j = 0; j < stackSizes[i].length; j++)
         {
            if (i == 0 || i == length - 1)
               stackSizes[i][j] = 0;
            else if (i == 1 || i == length - 2)
               stackSizes[i][j] = 1;
            else
               stackSizes[i][j] = 1 + (varyHeight ? random.nextInt(2) : 0);

            if (i == 1)
               types[i][j] = CinderBlockType.SLANTED_BACK;
            else if (i == length - 2)
               types[i][j] = CinderBlockType.SLANTED_FORWARD;
            else
               types[i][j] = allSlantedTypes[random.nextInt(allSlantedTypes.length)];
         }
      }

      RigidBodyTransform centerBasePose = new RigidBodyTransform(startPose);
      centerBasePose.appendTranslation(0.5 * stackSizes.length * CinderBlockFieldEnvironment.cinderBlockLength, 0.0, 0.0);
      return CinderBlockStackDescription.grid2D(centerBasePose, stackSizes, types);
   }

   public static List<List<CinderBlockStackDescription>> slantedCinderBlockLeveledFieldForAnkleRollLimit(Random random,
                                                                                                         RigidBodyTransformReadOnly startPose,
                                                                                                         boolean varyHeight)
   {
      int width = 2;
      int length = 20;
      int[][] stackSizes = new int[length][width];
      CinderBlockType[][] types = new CinderBlockType[length][width];
      CinderBlockType[] allSlantedTypes = {CinderBlockType.SLANTED_LEFT, CinderBlockType.SLANTED_RIGHT};

      for (int i = 0; i < stackSizes.length; i++)
      {
         for (int j = 0; j < stackSizes[i].length; j++)
         {
            if (i == 0 || i == length - 1)
               stackSizes[i][j] = 0;
            else if (i == 1 || i == length - 2)
               stackSizes[i][j] = 1;
            else
               stackSizes[i][j] = 1 + (varyHeight ? random.nextInt(2) : 0);

            if (i == 1)
               types[i][j] = CinderBlockType.SLANTED_BACK;
            else if (i == length - 2)
               types[i][j] = CinderBlockType.SLANTED_FORWARD;
            else
               types[i][j] = allSlantedTypes[random.nextInt(allSlantedTypes.length)];
         }
      }

      RigidBodyTransform centerBasePose = new RigidBodyTransform(startPose);
      centerBasePose.appendTranslation(0.5 * stackSizes.length * CinderBlockFieldEnvironment.cinderBlockLength, 0.0, 0.0);
      return CinderBlockStackDescription.grid2D(centerBasePose, stackSizes, types);
   }

   private static FootstepDataListMessage generateFootstepsForSlantedCinderBlockLeveledField(List<? extends List<? extends Pose3DReadOnly>> cinderBlockPoses,
                                                                                             double zOffset,
                                                                                             boolean slow)
   {
      FootstepDataListMessage footsteps = new FootstepDataListMessage();

      double yOffset = -0.06;
      SideDependentList<List<Pose3DReadOnly>> columns = extractColumns(cinderBlockPoses, 0, 1);

      for (RobotSide stepSide : RobotSide.values)
         addFootstepFromCBPose(footsteps, stepSide, columns.get(stepSide).get(0), 0, stepSide.negateIfRightSide(yOffset), zOffset, 0.0);
      for (RobotSide stepSide : RobotSide.values)
         addFootstepFromCBPose(footsteps, stepSide, columns.get(stepSide).get(1), 0, stepSide.negateIfRightSide(yOffset), zOffset, 0.0);

      RobotSide stepSide = RobotSide.LEFT;

      for (int i = 2; i < cinderBlockPoses.size() - 1; i++)
      {
         if (slow)
         {
            addFootstepFromCBPose(footsteps, stepSide, columns.get(stepSide).get(i), 0, stepSide.negateIfRightSide(yOffset), zOffset, 0.0);
            stepSide = stepSide.getOppositeSide();
            addFootstepFromCBPose(footsteps, stepSide, columns.get(stepSide).get(i), 0, stepSide.negateIfRightSide(yOffset), zOffset, 0.0);
            stepSide = stepSide.getOppositeSide();
         }
         else
         {
            addFootstepFromCBPose(footsteps, stepSide, columns.get(stepSide).get(i), 0, stepSide.negateIfRightSide(yOffset), zOffset, 0.0);
            stepSide = stepSide.getOppositeSide();
         }
      }

      for (int i = 0; i < 2; i++)
      {
         addFootstepFromCBPose(footsteps,
                               stepSide,
                               columns.get(stepSide).get(cinderBlockPoses.size() - 1),
                               0,
                               stepSide.negateIfRightSide(yOffset),
                               zOffset,
                               0.0);
         stepSide = stepSide.getOppositeSide();
      }

      return footsteps;
   }

   private static void addFootstepFromCBPose(FootstepDataListMessage footsteps,
                                             RobotSide stepSide,
                                             Pose3DReadOnly cinderBlockPose,
                                             double xOffset,
                                             double yOffset,
                                             double zOffset,
                                             double yawOffset)
   {
      Pose3D footstepPose = new Pose3D(cinderBlockPose);
      footstepPose.appendYawRotation(yawOffset);
      footstepPose.appendTranslation(xOffset, yOffset, zOffset);
      footsteps.getFootstepDataList().add().set(HumanoidMessageTools.createFootstepDataMessage(stepSide, footstepPose));
   }

   private static SideDependentList<List<Pose3DReadOnly>> extractColumns(List<? extends List<? extends Pose3DReadOnly>> cinderBlockPoses,
                                                                         int indexForLeftSide,
                                                                         int indexForRightSide)
   {
      SideDependentList<Integer> columnIndices = new SideDependentList<>(indexForLeftSide, indexForRightSide);
      SideDependentList<List<Pose3DReadOnly>> sideDependentColumns = new SideDependentList<>(new ArrayList<>(), new ArrayList<>());

      for (RobotSide robotSide : RobotSide.values)
      {
         int column = columnIndices.get(robotSide);

         for (int row = 0; row < cinderBlockPoses.size(); row++)
            sideDependentColumns.get(robotSide).add(cinderBlockPoses.get(row).get(column));
      }

      return sideDependentColumns;
   }
}
