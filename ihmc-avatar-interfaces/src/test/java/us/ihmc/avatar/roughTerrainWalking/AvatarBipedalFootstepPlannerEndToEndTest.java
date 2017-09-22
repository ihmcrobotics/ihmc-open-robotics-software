package us.ihmc.avatar.roughTerrainWalking;

import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import java.io.IOException;
import java.util.Arrays;
import java.util.concurrent.atomic.AtomicReference;

import org.junit.After;
import org.junit.Before;

import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.networkProcessor.DRCNetworkModuleParameters;
import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.communication.packets.PlanarRegionsListMessage;
import us.ihmc.communication.packets.ToolboxStateMessage;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.footstepPlanning.polygonSnapping.PlanarRegionsListExamples;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicCoordinateSystem;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsList;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
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

@ContinuousIntegrationAnnotations.ContinuousIntegrationPlan(categories = {IntegrationCategory.IN_DEVELOPMENT})
public abstract class AvatarBipedalFootstepPlannerEndToEndTest implements MultiRobotTestInterface
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromEnvironmentVariables();

   private DRCSimulationTestHelper drcSimulationTestHelper;
   private DRCNetworkModuleParameters networkModuleParameters;
   private HumanoidRobotDataReceiver humanoidRobotDataReceiver;

   private PacketCommunicator toolboxCommunicator;
   private PlanarRegionsList cinderBlockField;
   public static final double CINDER_BLOCK_START_X = 0.0;
   public static final double CINDER_BLOCK_START_Y = 0.0;
   public static final double CINDER_BLOCK_HEIGHT = 0.1;
   public static final double CINDER_BLOCK_SIZE = 0.4;
   public static final int CINDER_BLOCK_COURSE_WIDTH_X_IN_NUMBER_OF_BLOCKS = 5;
   public static final int CINDER_BLOCK_COURSE_LENGTH_Y_IN_NUMBER_OF_BLOCKS = 6;
   public static final double CINDER_BLOCK_HEIGHT_VARIATION = 0.0;
   public static final double CINDER_BLOCK_FIELD_PLATFORM_LENGTH = 0.6;

   private volatile boolean planCompleted = false;
   private AtomicReference<FootstepPlanningToolboxOutputStatus> outputStatus;

   @Before
   public void setup()
   {
      cinderBlockField = PlanarRegionsListExamples
            .generateCinderBlockField(CINDER_BLOCK_START_X, CINDER_BLOCK_START_Y, CINDER_BLOCK_SIZE, CINDER_BLOCK_HEIGHT, CINDER_BLOCK_COURSE_WIDTH_X_IN_NUMBER_OF_BLOCKS,
                                      CINDER_BLOCK_COURSE_LENGTH_Y_IN_NUMBER_OF_BLOCKS, CINDER_BLOCK_HEIGHT_VARIATION);

      networkModuleParameters = new DRCNetworkModuleParameters();
      networkModuleParameters.enableFootstepPlanningToolbox(true);
      networkModuleParameters.enableLocalControllerCommunicator(true);
      networkModuleParameters.enableNetworkProcessor(true);

      toolboxCommunicator = PacketCommunicator.createIntraprocessPacketCommunicator(NetworkPorts.FOOTSTEP_PLANNING_TOOLBOX_MODULE_PORT, new IHMCCommunicationKryoNetClassList());

      FullHumanoidRobotModel fullHumanoidRobotModel = getRobotModel().createFullRobotModel();
      ForceSensorDataHolder forceSensorDataHolder = new ForceSensorDataHolder(Arrays.asList(fullHumanoidRobotModel.getForceSensorDefinitions()));
      humanoidRobotDataReceiver = new HumanoidRobotDataReceiver(fullHumanoidRobotModel, forceSensorDataHolder);
      planCompleted = false;
   }

   @After
   public void tearDown()
   {
      cinderBlockField = null;
      networkModuleParameters = null;

      toolboxCommunicator.closeConnection();
      toolboxCommunicator.disconnect();
      toolboxCommunicator = null;
      planCompleted = false;
   }

   public void testSteppingStones() throws IOException
   {
      outputStatus = new AtomicReference<>();
      outputStatus.set(null);

      if(drcSimulationTestHelper != null)
      {
         drcSimulationTestHelper.destroySimulation();
      }

      CommonAvatarEnvironmentInterface steppingStonesEnvironment = createSteppingStonesEnvironment();

      DRCRobotModel robotModel = getRobotModel();
      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, robotModel);
      drcSimulationTestHelper.setTestEnvironment(steppingStonesEnvironment);
      drcSimulationTestHelper.setNetworkProcessorParameters(networkModuleParameters);
      drcSimulationTestHelper.createSimulation("steppingStonesTestHelper");

      toolboxCommunicator.connect();
      toolboxCommunicator.attachListener(FootstepPlanningToolboxOutputStatus.class, this::setOutputStatus);

      drcSimulationTestHelper.getControllerCommunicator().connect();
      drcSimulationTestHelper.getControllerCommunicator().attachListener(RobotConfigurationData.class, humanoidRobotDataReceiver);
      drcSimulationTestHelper.getControllerCommunicator().attachListener(WalkingStatusMessage.class, this::listenForWalkingComplete);

      BlockingSimulationRunner blockingSimulationRunner = drcSimulationTestHelper.getBlockingSimulationRunner();
      ToolboxStateMessage wakeUpMessage = new ToolboxStateMessage(ToolboxStateMessage.ToolboxState.WAKE_UP);
      toolboxCommunicator.send(wakeUpMessage);

      while(!humanoidRobotDataReceiver.framesHaveBeenSetUp())
      {
         try
         {
            blockingSimulationRunner.simulateAndBlock(1.0);
         }
         catch(BlockingSimulationRunner.SimulationExceededMaximumTimeException | ControllerFailureException e)
         {
            e.printStackTrace();
            fail(e.getMessage());
         }

         humanoidRobotDataReceiver.updateRobotModel();
      }

      ReferenceFrame soleFrame = humanoidRobotDataReceiver.getReferenceFrames().getSoleFrame(RobotSide.LEFT);
      FramePose initialStancePose = new FramePose(soleFrame, new Point3D(0.0, 0.0, 0.001), new AxisAngle());
      initialStancePose.changeFrame(ReferenceFrame.getWorldFrame());
      RobotSide initialStanceSide = RobotSide.LEFT;
      double courseLength = CINDER_BLOCK_COURSE_WIDTH_X_IN_NUMBER_OF_BLOCKS * CINDER_BLOCK_SIZE + CINDER_BLOCK_FIELD_PLATFORM_LENGTH;
      FramePose goalPose = new FramePose(ReferenceFrame.getWorldFrame(), new Pose3D(courseLength, 0.0, 0.001, 0.0, 0.0, 0.0));

      YoGraphicsListRegistry graphicsListRegistry = createStartAndGoalGraphics(initialStancePose, goalPose);
      drcSimulationTestHelper.getSimulationConstructionSet().addYoGraphicsListRegistry(graphicsListRegistry);

      FootstepPlanningRequestPacket requestPacket = new FootstepPlanningRequestPacket(initialStancePose, initialStanceSide, goalPose);
      requestPacket.setAssumeFlatGround(false);
      toolboxCommunicator.send(requestPacket);

      try
      {
         blockingSimulationRunner.simulateAndBlock(1.0);
      }
      catch(BlockingSimulationRunner.SimulationExceededMaximumTimeException | ControllerFailureException e)
      {
         e.printStackTrace();
         fail(e.getMessage());
      }

      PlanarRegionsListMessage planarRegionsListMessage = PlanarRegionMessageConverter.convertToPlanarRegionsListMessage(cinderBlockField);
      toolboxCommunicator.send(planarRegionsListMessage);

      double timeout = 30.0;
      long startTime = System.currentTimeMillis();
      while(outputStatus.get() == null)
      {
         long time = System.currentTimeMillis();
         if (timeout < (time - startTime) / 1000.0)
         {
            fail("Timed out.");
         }

         try
         {
            blockingSimulationRunner.simulateAndBlock(1.0);
         }
         catch(BlockingSimulationRunner.SimulationExceededMaximumTimeException | ControllerFailureException e)
         {
            e.printStackTrace();
            fail(e.getMessage());
         }
      }

      planCompleted = false;
      if(outputStatus.get().footstepDataList.size() > 0)
      {
         drcSimulationTestHelper.send(outputStatus.get().footstepDataList);

         while(!planCompleted)
         {
            try
            {
               blockingSimulationRunner.simulateAndBlock(1.0);
            }
            catch(BlockingSimulationRunner.SimulationExceededMaximumTimeException | ControllerFailureException e)
            {
               e.printStackTrace();
               fail(e.getMessage());
            }
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

   private CommonAvatarEnvironmentInterface createSteppingStonesEnvironment()
   {
      double allowablePenetrationThickness = 0.01;
      boolean generateGroundPlane = false;
      return new PlanarRegionsListDefinedEnvironment("cinderBlockFieldEnvironment", cinderBlockField,
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
