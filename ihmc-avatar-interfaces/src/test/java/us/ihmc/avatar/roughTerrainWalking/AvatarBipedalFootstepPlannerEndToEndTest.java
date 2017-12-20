package us.ihmc.avatar.roughTerrainWalking;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;
import us.ihmc.avatar.DRCStartingLocation;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.initialSetup.OffsetAndYawRobotInitialSetup;
import us.ihmc.avatar.networkProcessor.DRCNetworkModuleParameters;
import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.communication.packets.PlanarRegionsListMessage;
import us.ihmc.communication.packets.ToolboxStateMessage;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations;
import us.ihmc.continuousIntegration.ContinuousIntegrationTools;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.footstepPlanning.polygonSnapping.PlanarRegionsListExamples;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicCoordinateSystem;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsList;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.footstepPlanning.FootstepPlannerType;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepPlanningRequestPacket;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepPlanningToolboxOutputStatus;
import us.ihmc.humanoidRobotics.communication.packets.walking.WalkingStatusMessage;
import us.ihmc.humanoidRobotics.communication.subscribers.HumanoidRobotDataReceiver;
import us.ihmc.humanoidRobotics.kryo.IHMCCommunicationKryoNetClassList;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.controllers.ControllerFailureException;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.math.frames.YoFramePose;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.sensors.ForceSensorDataHolder;
import us.ihmc.sensorProcessing.communication.packets.dataobjects.RobotConfigurationData;
import us.ihmc.simulationConstructionSetTools.util.environments.CommonAvatarEnvironmentInterface;
import us.ihmc.simulationConstructionSetTools.util.environments.PlanarRegionsListDefinedEnvironment;
import us.ihmc.simulationconstructionset.FloatingJoint;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;
import us.ihmc.commons.thread.ThreadTools;

import java.io.IOException;
import java.util.Arrays;
import java.util.concurrent.atomic.AtomicReference;

import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

@ContinuousIntegrationAnnotations.ContinuousIntegrationPlan(categories = {IntegrationCategory.IN_DEVELOPMENT})
public abstract class AvatarBipedalFootstepPlannerEndToEndTest implements MultiRobotTestInterface
{
   private SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();
   private static final boolean keepSCSUp = false;
   private static final int timeout = 120000; // to easily keep scs up. unfortunately can't be set programmatically, has to be a constant

   private DRCSimulationTestHelper drcSimulationTestHelper;
   private DRCNetworkModuleParameters networkModuleParameters;
   private HumanoidRobotDataReceiver humanoidRobotDataReceiver;

   private PacketCommunicator toolboxCommunicator;
   private PlanarRegionsList cinderBlockField;
   private PlanarRegionsList steppingStoneField;

   public static final double CINDER_BLOCK_START_X = 0.0;
   public static final double CINDER_BLOCK_START_Y = 0.0;
   public static final double CINDER_BLOCK_HEIGHT = 0.1;
   public static final double CINDER_BLOCK_SIZE = 0.4;
   public static final int CINDER_BLOCK_COURSE_WIDTH_X_IN_NUMBER_OF_BLOCKS = 5;
   public static final int CINDER_BLOCK_COURSE_LENGTH_Y_IN_NUMBER_OF_BLOCKS = 6;
   public static final double CINDER_BLOCK_HEIGHT_VARIATION = 0.0;
   public static final double CINDER_BLOCK_FIELD_PLATFORM_LENGTH = 0.6;

   public static final double STEPPING_STONE_PATH_RADIUS = 3.5;

   private volatile boolean planCompleted = false;
   private AtomicReference<FootstepPlanningToolboxOutputStatus> outputStatus;
   private BlockingSimulationRunner blockingSimulationRunner;

   @Before
   public void setup()
   {
      cinderBlockField = PlanarRegionsListExamples.generateCinderBlockField(CINDER_BLOCK_START_X, CINDER_BLOCK_START_Y, CINDER_BLOCK_SIZE, CINDER_BLOCK_HEIGHT,
                                                                            CINDER_BLOCK_COURSE_WIDTH_X_IN_NUMBER_OF_BLOCKS,
                                                                            CINDER_BLOCK_COURSE_LENGTH_Y_IN_NUMBER_OF_BLOCKS, CINDER_BLOCK_HEIGHT_VARIATION);
      steppingStoneField = PlanarRegionsListExamples.generateSteppingStonesEnvironment(STEPPING_STONE_PATH_RADIUS);

      networkModuleParameters = new DRCNetworkModuleParameters();
      networkModuleParameters.enableFootstepPlanningToolbox(true);
      networkModuleParameters.enableLocalControllerCommunicator(true);
      networkModuleParameters.enableNetworkProcessor(true);

      toolboxCommunicator = PacketCommunicator.createIntraprocessPacketCommunicator(NetworkPorts.FOOTSTEP_PLANNING_TOOLBOX_MODULE_PORT, new IHMCCommunicationKryoNetClassList());

      FullHumanoidRobotModel fullHumanoidRobotModel = getRobotModel().createFullRobotModel();
      ForceSensorDataHolder forceSensorDataHolder = new ForceSensorDataHolder(Arrays.asList(fullHumanoidRobotModel.getForceSensorDefinitions()));
      humanoidRobotDataReceiver = new HumanoidRobotDataReceiver(fullHumanoidRobotModel, forceSensorDataHolder);
      planCompleted = false;

      simulationTestingParameters.setKeepSCSUp(keepSCSUp && !ContinuousIntegrationTools.isRunningOnContinuousIntegrationServer());
   }

   @After
   public void tearDown()
   {
      cinderBlockField = null;
      steppingStoneField = null;
      networkModuleParameters = null;

      toolboxCommunicator.closeConnection();
      toolboxCommunicator.disconnect();
      toolboxCommunicator = null;
      planCompleted = false;

      humanoidRobotDataReceiver = null;
      blockingSimulationRunner = null;

      // Do this here in case a test fails. That way the memory will be recycled.
      if (drcSimulationTestHelper != null)
      {
         drcSimulationTestHelper.destroySimulation();
         drcSimulationTestHelper = null;
      }

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }

   @ContinuousIntegrationAnnotations.ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = timeout)
   public void testShortCinderBlockFieldWithAStar() throws IOException
   {
      double courseLength = CINDER_BLOCK_COURSE_WIDTH_X_IN_NUMBER_OF_BLOCKS * CINDER_BLOCK_SIZE + CINDER_BLOCK_FIELD_PLATFORM_LENGTH;
      DRCStartingLocation startingLocation = () -> new OffsetAndYawRobotInitialSetup(0.0, 0.0, 0.007);
      FramePose goalPose = new FramePose(ReferenceFrame.getWorldFrame(), new Pose3D(courseLength, 0.0, 0.0, 0.0, 0.0, 0.0));

      runEndToEndTestAndKeepSCSUpIfRequested(FootstepPlannerType.A_STAR, cinderBlockField, startingLocation, goalPose);
   }

   @ContinuousIntegrationAnnotations.ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = timeout)
   public void testShortCinderBlockFieldWithPlanarRegionBipedalPlanner() throws IOException
   {
      double courseLength = CINDER_BLOCK_COURSE_WIDTH_X_IN_NUMBER_OF_BLOCKS * CINDER_BLOCK_SIZE + CINDER_BLOCK_FIELD_PLATFORM_LENGTH;
      DRCStartingLocation startingLocation = () -> new OffsetAndYawRobotInitialSetup(0.0, 0.0, 0.007);
      FramePose goalPose = new FramePose(ReferenceFrame.getWorldFrame(), new Pose3D(courseLength, 0.0, 0.0, 0.0, 0.0, 0.0));

      runEndToEndTestAndKeepSCSUpIfRequested(FootstepPlannerType.PLANAR_REGION_BIPEDAL, cinderBlockField, startingLocation, goalPose);
   }

   @ContinuousIntegrationAnnotations.ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = timeout)
   public void testSteppingStonesWithAStar() throws IOException
   {
      DRCStartingLocation startingLocation = () -> new OffsetAndYawRobotInitialSetup(0.0, -0.75, 0.007, 0.5 * Math.PI);
      FramePose goalPose = new FramePose(ReferenceFrame.getWorldFrame(), new Pose3D(STEPPING_STONE_PATH_RADIUS + 0.5, STEPPING_STONE_PATH_RADIUS, 0.0, 0.0, 0.0, 0.0));

      runEndToEndTestAndKeepSCSUpIfRequested(FootstepPlannerType.A_STAR, steppingStoneField, startingLocation, goalPose);
   }

   @ContinuousIntegrationAnnotations.ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = timeout)
   public void testSteppingStonesWithPlanarRegionBipedalPlanner() throws IOException
   {
      DRCStartingLocation startingLocation = () -> new OffsetAndYawRobotInitialSetup(0.0, -0.75, 0.007, 0.5 * Math.PI);
      FramePose goalPose = new FramePose(ReferenceFrame.getWorldFrame(), new Pose3D(STEPPING_STONE_PATH_RADIUS + 0.5, STEPPING_STONE_PATH_RADIUS, 0.0, 0.0, 0.0, 0.0));

      runEndToEndTestAndKeepSCSUpIfRequested(FootstepPlannerType.PLANAR_REGION_BIPEDAL, steppingStoneField, startingLocation, goalPose);
   }

   private void runEndToEndTestAndKeepSCSUpIfRequested(FootstepPlannerType plannerType, PlanarRegionsList planarRegionsList,
                                                       DRCStartingLocation startingLocation, FramePose goalPose)
   {
      try
      {
         runEndToEndTest(plannerType, planarRegionsList, startingLocation, goalPose);
      }
      catch(Exception e)
      {
         e.printStackTrace();
         if(simulationTestingParameters.getKeepSCSUp())
         {
            ThreadTools.sleepForever();
         }
         else
         {
            fail(e.getMessage());
         }
      }
   }

   private void runEndToEndTest(FootstepPlannerType plannerType, PlanarRegionsList planarRegionsList,
                                DRCStartingLocation startingLocation, FramePose goalPose) throws Exception
   {
      outputStatus = new AtomicReference<>();
      outputStatus.set(null);

      if(drcSimulationTestHelper != null)
      {
         drcSimulationTestHelper.destroySimulation();
      }

      CommonAvatarEnvironmentInterface steppingStonesEnvironment = createCommonAvatarInterface(planarRegionsList);
      DRCRobotModel robotModel = getRobotModel();
      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, robotModel);
      drcSimulationTestHelper.setTestEnvironment(steppingStonesEnvironment);
      drcSimulationTestHelper.setNetworkProcessorParameters(networkModuleParameters);
      drcSimulationTestHelper.setStartingLocation(startingLocation);
      drcSimulationTestHelper.createSimulation("steppingStonesTestHelper");

      toolboxCommunicator.connect();
      toolboxCommunicator.attachListener(FootstepPlanningToolboxOutputStatus.class, this::setOutputStatus);

      drcSimulationTestHelper.getControllerCommunicator().connect();
      drcSimulationTestHelper.getControllerCommunicator().attachListener(RobotConfigurationData.class, humanoidRobotDataReceiver);
      drcSimulationTestHelper.getControllerCommunicator().attachListener(WalkingStatusMessage.class, this::listenForWalkingComplete);

      blockingSimulationRunner = drcSimulationTestHelper.getBlockingSimulationRunner();
      ToolboxStateMessage wakeUpMessage = new ToolboxStateMessage(ToolboxStateMessage.ToolboxState.WAKE_UP);
      toolboxCommunicator.send(wakeUpMessage);

      while(!humanoidRobotDataReceiver.framesHaveBeenSetUp())
      {
         simulate(1.0);
         humanoidRobotDataReceiver.updateRobotModel();
      }

      ReferenceFrame soleFrame = humanoidRobotDataReceiver.getReferenceFrames().getSoleFrame(RobotSide.LEFT);
      FramePose initialStancePose = new FramePose(soleFrame, new Point3D(0.0, 0.0, 0.001), new AxisAngle());
      initialStancePose.changeFrame(ReferenceFrame.getWorldFrame());
      RobotSide initialStanceSide = RobotSide.LEFT;

      YoGraphicsListRegistry graphicsListRegistry = createStartAndGoalGraphics(initialStancePose, goalPose);
      drcSimulationTestHelper.getSimulationConstructionSet().addYoGraphicsListRegistry(graphicsListRegistry);

      FootstepPlanningRequestPacket requestPacket = new FootstepPlanningRequestPacket(initialStancePose, initialStanceSide, goalPose, plannerType);
      PlanarRegionsListMessage planarRegionsListMessage = PlanarRegionMessageConverter.convertToPlanarRegionsListMessage(planarRegionsList);
      requestPacket.setPlanarRegionsListMessage(planarRegionsListMessage);
      toolboxCommunicator.send(requestPacket);

      while(outputStatus.get() == null)
      {
         simulate(1.0);
      }

      FootstepPlanningToolboxOutputStatus outputStatus = this.outputStatus.get();
      if(!outputStatus.planningResult.validForExecution())
      {
         throw new RuntimeException("Footstep plan not valid for execution: " + outputStatus.planningResult);
      }

      planCompleted = false;
      if(outputStatus.footstepDataList.size() > 0)
      {
         drcSimulationTestHelper.send(outputStatus.footstepDataList);

         while(!planCompleted)
         {
            simulate(1.0);
         }
      }

      FloatingJoint rootJoint = drcSimulationTestHelper.getRobot().getRootJoint();
      Point3D rootJointPosition = new Point3D();
      rootJoint.getPosition(rootJointPosition);

      double errorThreshold = 0.3;
      double xPositionErrorMagnitude = Math.abs(rootJointPosition.getX() - goalPose.getX());
      double yPositionErrorMagnitude = Math.abs(rootJointPosition.getY() - goalPose.getY());
      assertTrue(xPositionErrorMagnitude < errorThreshold);
      assertTrue(yPositionErrorMagnitude < errorThreshold);
   }

   private YoGraphicsListRegistry createStartAndGoalGraphics(FramePose initialStancePose, FramePose goalPose)
   {
      YoGraphicsListRegistry graphicsListRegistry = new YoGraphicsListRegistry();
      YoGraphicsList graphicsList = new YoGraphicsList("testViz");

      YoFramePose yoInitialStancePose = new YoFramePose("initialStancePose", initialStancePose.getReferenceFrame(), drcSimulationTestHelper.getYoVariableRegistry());
      yoInitialStancePose.set(initialStancePose);

      YoFramePose yoGoalPose = new YoFramePose("goalStancePose", goalPose.getReferenceFrame(), drcSimulationTestHelper.getYoVariableRegistry());
      yoGoalPose.set(goalPose);

      YoGraphicCoordinateSystem startPoseGraphics = new YoGraphicCoordinateSystem("startPose", yoInitialStancePose, 13.0);
      YoGraphicCoordinateSystem goalPoseGraphics = new YoGraphicCoordinateSystem("goalPose", yoGoalPose, 13.0);

      graphicsList.add(startPoseGraphics);
      graphicsList.add(goalPoseGraphics);
      return graphicsListRegistry;
   }

   private void simulate(double time) throws BlockingSimulationRunner.SimulationExceededMaximumTimeException, ControllerFailureException
   {
      if(keepSCSUp)
         blockingSimulationRunner.simulateAndBlock(1.0);
      else
         blockingSimulationRunner.simulateAndBlock(1.0);
   }

   private static CommonAvatarEnvironmentInterface createCommonAvatarInterface(PlanarRegionsList planarRegionsList)
   {
      double allowablePenetrationThickness = 0.05;
      boolean generateGroundPlane = false;
      return new PlanarRegionsListDefinedEnvironment("testEnvironment", planarRegionsList,
                                                     allowablePenetrationThickness, generateGroundPlane);
   }

   private void listenForWalkingComplete(WalkingStatusMessage walkingStatusMessage)
   {
      if(walkingStatusMessage.status == WalkingStatusMessage.Status.COMPLETED)
      {
         planCompleted = true;
      }
   }

   private void setOutputStatus(FootstepPlanningToolboxOutputStatus packet)
   {
      outputStatus.set(packet);
   }
}
