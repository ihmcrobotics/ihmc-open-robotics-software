package us.ihmc.avatar.roughTerrainWalking;

import static us.ihmc.robotics.Assert.assertTrue;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import controller_msgs.msg.dds.ContinuousStepGeneratorInputMessage;
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
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.plugin.ContinuousStepGeneratorInputCommand;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
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
import us.ihmc.simulationConstructionSetTools.util.environments.planarRegionEnvironments.CinderBlockFieldPlanarRegionEnvironment;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;

public abstract class EndToEndPlanarCinderBlockFieldTest implements MultiRobotTestInterface
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();

   private boolean useImpulseBasedPhysicsEngine = false;
   private SCS2AvatarTestingSimulation simulationTestHelper;

   public double getSwingHeight()
   {
      return getRobotModel().getWalkingControllerParameters().getSteppingParameters().getDefaultSwingHeightFromStanceFoot();
   }

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


   private void setupSimulation(CommonAvatarEnvironmentInterface environment)
   {
      SCS2AvatarTestingSimulationFactory simulationFactory = new SCS2AvatarTestingSimulationFactory(getRobotModel(), environment);
      simulationFactory.setDefaultHighLevelHumanoidControllerFactory();
      simulationFactory.setShowGUI(simulationTestingParameters.getCreateGUI());
      simulationFactory.setRunMultiThreaded(simulationTestingParameters.getRunMultiThreaded());
      simulationFactory.setUseImpulseBasedPhysicsEngine(useImpulseBasedPhysicsEngine);
      simulationTestHelper = simulationFactory.createAvatarTestingSimulation();
   }

   @Test
   @Tag("fast")
   public void testWalkingOverCinderBlockField() throws Exception
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      CinderBlockFieldPlanarRegionEnvironment cinderBlockFieldEnvironment = new CinderBlockFieldPlanarRegionEnvironment();

      setupSimulation(cinderBlockFieldEnvironment);
      simulationTestHelper.setKeepSCSUp(true);
      simulationTestHelper.start();

      ThreadTools.sleep(1000);
      boolean success = simulationTestHelper.simulateNow(0.5);
      assertTrue(success);

      simulationTestHelper.publishToController(PlanarRegionMessageConverter.convertToPlanarRegionsListMessage(cinderBlockFieldEnvironment.getPlanarRegionsList()));

      ContinuousStepGeneratorInputMessage stepGeneratorMessage = new ContinuousStepGeneratorInputMessage();
      stepGeneratorMessage.setForwardVelocity(0.4);
      stepGeneratorMessage.setWalk(true);

      simulationTestHelper.publishToController(stepGeneratorMessage);

      success = simulationTestHelper.simulateNow(20.0);
      assertTrue(success);


//      simulationTestHelper.assertRobotsRootJointIsInBoundingBox(new BoundingBox3D(min, max));

   }


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
