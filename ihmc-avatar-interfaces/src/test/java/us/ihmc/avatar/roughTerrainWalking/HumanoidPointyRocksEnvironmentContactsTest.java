package us.ihmc.avatar.roughTerrainWalking;

import static org.junit.Assert.assertTrue;

import java.util.ArrayList;

import org.junit.After;
import org.junit.Before;

import controller_msgs.msg.dds.ArmTrajectoryMessage;
import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepDataMessage;
import controller_msgs.msg.dds.OneDoFJointTrajectoryMessage;
import controller_msgs.msg.dds.TrajectoryPoint1DMessage;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationConstructionSetTools.util.environments.CommonAvatarEnvironmentInterface;
import us.ihmc.simulationConstructionSetTools.util.environments.PointyRocksWorld;
import us.ihmc.simulationConstructionSetTools.util.environments.PointyRocksWorld.PointyRocksType;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public abstract class HumanoidPointyRocksEnvironmentContactsTest implements MultiRobotTestInterface
{
   private SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();
   private DRCSimulationTestHelper drcSimulationTestHelper;

   private YoBoolean doFootExplorationInTransferToStanding;
   private YoDouble percentageChickenSupport;
   private YoDouble timeBeforeExploring;
   private SideDependentList<YoBoolean> autoCropToLineAfterExploration = new SideDependentList<>();
   private SideDependentList<YoBoolean> holdFlat = new SideDependentList<>(); // new supportState
   private YoBoolean allowUpperBodyMomentumInSingleSupport;
   private YoBoolean allowUpperBodyMomentumInDoubleSupport;
   private YoBoolean allowUsingHighMomentumWeight;
   private SideDependentList<YoBoolean> requestExploration = new SideDependentList<>();
   private SideDependentList<YoBoolean> doPartialDetection = new SideDependentList<>();
   private SideDependentList<YoBoolean> cropToConvexHullOfCoPs = new SideDependentList<>();

   protected abstract DRCRobotModel getRobotModel(int xContactPoints, int yContactPoints, boolean createOnlyEdgePoints);

   public void testWalkingOnLinesInEnvironment() throws SimulationExceededMaximumTimeException
   {
      PointyRocksWorld world = new PointyRocksWorld(PointyRocksType.LINES, 6);
      setupTest(world, true);

      Point3D cameraFix = new Point3D();
      Point3D cameraPosition = new Point3D();
      world.setupCamera(cameraFix, cameraPosition);
      drcSimulationTestHelper.setupCameraForUnitTest(cameraFix, cameraPosition);

      armsUp();

      // enable the use of body momentum in the controller
      allowUpperBodyMomentumInSingleSupport.set(true);
      allowUpperBodyMomentumInDoubleSupport.set(true);
      allowUsingHighMomentumWeight.set(true);

      // change the walking parameters
      for (RobotSide robotSide : RobotSide.values)
      {
         autoCropToLineAfterExploration.get(robotSide).set(true);
         holdFlat.get(robotSide).set(true);
         doPartialDetection.get(robotSide).set(true);
      }

      doFootExplorationInTransferToStanding.set(true);
      percentageChickenSupport.set(0.4);
      timeBeforeExploring.set(1.0);

      double swingTime = 0.8;
      double transferTime = 0.15;

      ArrayList<FramePoint3D> stepLocations = world.getStepLocations();
      for (int i = 0; i < stepLocations.size(); i++)
      {
         if (i == stepLocations.size()-2)
         {
            percentageChickenSupport.set(0.5);
            doFootExplorationInTransferToStanding.set(false);
         }

         FootstepDataListMessage message = HumanoidMessageTools.createFootstepDataListMessage(swingTime, transferTime);
         FootstepDataMessage footstepData = new FootstepDataMessage();

         Point3D position = new Point3D(stepLocations.get(i));
         RobotSide robotSide = position.getY() > 0.0 ? RobotSide.LEFT : RobotSide.RIGHT;
         footstepData.getLocation().set(position);
         footstepData.getOrientation().set(new Quaternion(0.0, 0.0, 0.0, 1.0));
         footstepData.setRobotSide(robotSide.toByte());
         message.getFootstepDataList().add().set(footstepData);

         drcSimulationTestHelper.send(message);
         boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(6.0);
         assertTrue(success);
      }

      drcSimulationTestHelper.createVideo(getSimpleRobotName(), 1);
      drcSimulationTestHelper.checkNothingChanged();
   }

   public void testWalkingOnPointInEnvironment() throws SimulationExceededMaximumTimeException
   {
      PointyRocksWorld world = new PointyRocksWorld(PointyRocksType.POINT, 0);
      setupTest(world, false);

      Point3D cameraFix = new Point3D();
      Point3D cameraPosition = new Point3D();
      world.setupCamera(cameraFix, cameraPosition);
      drcSimulationTestHelper.setupCameraForUnitTest(cameraFix, cameraPosition);

      armsUp();

      // enable the use of body momentum in the controller
      allowUpperBodyMomentumInSingleSupport.set(true);
      allowUpperBodyMomentumInDoubleSupport.set(true);
      allowUsingHighMomentumWeight.set(true);

      // change the walking parameters
      for (RobotSide robotSide : RobotSide.values)
      {
         autoCropToLineAfterExploration.get(robotSide).set(false);
         holdFlat.get(robotSide).set(true);
         doPartialDetection.get(robotSide).set(true);
      }

      doFootExplorationInTransferToStanding.set(true);
      percentageChickenSupport.set(0.3);
      timeBeforeExploring.set(1.0);

      double swingTime = 0.8;
      double transferTime = 0.15;

      ArrayList<FramePoint3D> stepLocations = world.getStepLocations();
      for (int i = 0; i < stepLocations.size(); i++)
      {
         if (i == stepLocations.size()-2)
         {
            percentageChickenSupport.set(0.7);
            doFootExplorationInTransferToStanding.set(false);
         }

         FootstepDataListMessage message = HumanoidMessageTools.createFootstepDataListMessage(swingTime, transferTime);
         FootstepDataMessage footstepData = new FootstepDataMessage();

         Point3D position = new Point3D(stepLocations.get(i));
         RobotSide robotSide = position.getY() > 0.0 ? RobotSide.LEFT : RobotSide.RIGHT;
         footstepData.getLocation().set(position);
         footstepData.getOrientation().set(new Quaternion(0.0, 0.0, 0.0, 1.0));
         footstepData.setRobotSide(robotSide.toByte());
         message.getFootstepDataList().add().set(footstepData);

         drcSimulationTestHelper.send(message);
         boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(4.0);
         assertTrue(success);
      }

      drcSimulationTestHelper.createVideo(getSimpleRobotName(), 1);
      drcSimulationTestHelper.checkNothingChanged();
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

      simulationTestingParameters = null;
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }

   private void setupTest(CommonAvatarEnvironmentInterface environment, boolean onlyEdgeContacts)
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      // create simulation test helper
      String className = getClass().getSimpleName();
      DRCRobotModel robotModel = getRobotModel(15, 8, onlyEdgeContacts);
      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, robotModel, environment);
      drcSimulationTestHelper.createSimulation(className);
      drcSimulationTestHelper.getSimulationConstructionSet().hideAllYoGraphics();

      // increase ankle damping to match the real robot better
      YoDouble damping_l_akx = (YoDouble) drcSimulationTestHelper.getYoVariable("b_damp_l_leg_akx");
      YoDouble damping_l_aky = (YoDouble) drcSimulationTestHelper.getYoVariable("b_damp_l_leg_aky");
      YoDouble damping_r_akx = (YoDouble) drcSimulationTestHelper.getYoVariable("b_damp_r_leg_akx");
      YoDouble damping_r_aky = (YoDouble) drcSimulationTestHelper.getYoVariable("b_damp_r_leg_aky");
      damping_l_akx.set(1.0);
      damping_l_aky.set(1.0);
      damping_r_akx.set(1.0);
      damping_r_aky.set(1.0);

      // get a bunch of relevant variables
      doFootExplorationInTransferToStanding = (YoBoolean) drcSimulationTestHelper.getYoVariable("doFootExplorationInTransferToStanding");
      percentageChickenSupport = (YoDouble) drcSimulationTestHelper.getYoVariable("icpPlannerCoPTrajectoryGeneratorPercentageChickenSupport");
      timeBeforeExploring = (YoDouble) drcSimulationTestHelper.getYoVariable("ExplorationState_TimeBeforeExploring");
      for (RobotSide robotSide : RobotSide.values)
      {
         String footName = drcSimulationTestHelper.getControllerFullRobotModel().getFoot(robotSide).getName();
         String longFootName = robotSide.getLowerCaseName() + "Foot";
         String footControlNamespace = robotSide.getLowerCaseName() + "FootControlModule";
         String partialfootControlNamespace = footName + "PartialFootholdControlModule";

         YoBoolean autoCrop = (YoBoolean) drcSimulationTestHelper.getYoVariable(footName + "ExpectingLineContact");
         autoCropToLineAfterExploration.put(robotSide, autoCrop);
         YoBoolean requestExplorationForFoot = (YoBoolean) drcSimulationTestHelper.getYoVariable(footControlNamespace, longFootName + "RequestExploration");
         requestExploration.put(robotSide, requestExplorationForFoot);
         YoBoolean doPartialDetectionForFoot = (YoBoolean) drcSimulationTestHelper.getYoVariable(partialfootControlNamespace, footName + "DoPartialFootholdDetection");
         doPartialDetection.put(robotSide, doPartialDetectionForFoot);
         YoBoolean cropToConvexHullOfCoPsForFoot = (YoBoolean) drcSimulationTestHelper.getYoVariable(partialfootControlNamespace, footName + "CropToConvexHullOfCoPs");
         cropToConvexHullOfCoPs.put(robotSide, cropToConvexHullOfCoPsForFoot);

         // new support state:
         YoBoolean holdFlat = (YoBoolean) drcSimulationTestHelper.getYoVariable(longFootName + "SupportState", longFootName + "HoldFlatOrientation");
         this.holdFlat.put(robotSide, holdFlat);
      }
      allowUpperBodyMomentumInSingleSupport = (YoBoolean) drcSimulationTestHelper.getYoVariable("allowUpperBodyMomentumInSingleSupport");
      allowUpperBodyMomentumInDoubleSupport = (YoBoolean) drcSimulationTestHelper.getYoVariable("allowUpperBodyMomentumInDoubleSupport");
      allowUsingHighMomentumWeight = (YoBoolean) drcSimulationTestHelper.getYoVariable("allowUsingHighMomentumWeight");

      ThreadTools.sleep(1000);
   }

   private static final double[] rightHandStraightSideJointAngles = new double[] {-0.5067668142160446, -0.3659876546358431, 1.7973796317575155, -1.2398714600960365, -0.005510224629709242, 0.6123343067479899, 0.12524505635696856};
   private static final double[] leftHandStraightSideJointAngles = new double[] {0.61130147334225, 0.22680071472282162, 1.6270339908033258, 1.2703560974484844, 0.10340544060719102, -0.6738299572358809, 0.13264785356924128};
   private static final SideDependentList<double[]> straightArmConfigs = new SideDependentList<>();
   static
   {
      straightArmConfigs.put(RobotSide.LEFT, leftHandStraightSideJointAngles);
      straightArmConfigs.put(RobotSide.RIGHT, rightHandStraightSideJointAngles);
   }

   private void armsUp() throws SimulationExceededMaximumTimeException
   {
      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.1);

      // bring the arms in a stretched position
      for (RobotSide robotSide : RobotSide.values)
      {
         ArmTrajectoryMessage armTrajectoryMessage = new ArmTrajectoryMessage();
         armTrajectoryMessage.setRobotSide(robotSide.toByte());
         double[] armConfig = straightArmConfigs.get(robotSide);
         for (int i = 0; i < armConfig.length; i++)
         {
            TrajectoryPoint1DMessage trajectoryPoint = new TrajectoryPoint1DMessage();
            trajectoryPoint.setPosition(armConfig[i]);
            trajectoryPoint.setTime(0.5);
            OneDoFJointTrajectoryMessage jointTrajectory = new OneDoFJointTrajectoryMessage();
            jointTrajectory.getTrajectoryPoints().add().set(trajectoryPoint);
            armTrajectoryMessage.getJointspaceTrajectory().getJointTrajectoryMessages().add().set(jointTrajectory);
         }
         drcSimulationTestHelper.send(armTrajectoryMessage);
      }

      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.6);
   }
}
