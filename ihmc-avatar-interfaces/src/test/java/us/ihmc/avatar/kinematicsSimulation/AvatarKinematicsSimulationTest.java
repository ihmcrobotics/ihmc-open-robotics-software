package us.ihmc.avatar.kinematicsSimulation;

import controller_msgs.msg.dds.*;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.jointAnglesWriter.JointAnglesWriter;
import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.HumanoidKinematicsToolboxControllerTest;
import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.KinematicsToolboxControllerTest;
import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.graphicsDescription.appearance.YoAppearanceRGBColor;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.FootstepDataListCommand;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.idl.IDLSequence.Object;
import us.ihmc.log.LogTools;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotDescription.RobotDescription;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.sensorProcessing.simulatedSensors.DRCPerfectSensorReaderFactory;
import us.ihmc.sensorProcessing.simulatedSensors.SensorDataContext;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationConstructionSetTools.util.HumanoidFloatingRootJointRobot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.gui.tools.SimulationOverheadPlotterFactory;
import us.ihmc.simulationconstructionset.util.ControllerFailureException;
import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;

import java.awt.*;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;

public abstract class AvatarKinematicsSimulationTest implements MultiRobotTestInterface
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final double EPSILON = 1.0e-12;
   private static final YoAppearanceRGBColor ghostApperance = new YoAppearanceRGBColor(Color.YELLOW, 0.75);
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();

   private DRCSimulationTestHelper drcSimulationTestHelper;
   private SimulationConstructionSet scs;

   private YoVariableRegistry toolboxMainRegistry;
   private YoGraphicsListRegistry yoGraphicsListRegistry;
   private AvatarKinematicsSimulationController avatarKinematicsSimulationController;

   private YoBoolean enableToolboxUpdater;
   private YoBoolean pauseToolboxUpdater;
   private YoBoolean initializationSucceeded;

   private HumanoidFloatingRootJointRobot ghost;
   private RobotController toolboxUpdater;

   /**
    * Returns a separate instance of the robot model that will be modified in this test to create a
    * ghost robot.
    */
   public abstract DRCRobotModel getGhostRobotModel();

   public void setup(double integrationDT)
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");

      toolboxMainRegistry = new YoVariableRegistry("toolboxMain");
      enableToolboxUpdater = new YoBoolean("enableToolboxUpdater", toolboxMainRegistry);
      pauseToolboxUpdater = new YoBoolean("pauseToolboxUpdater", toolboxMainRegistry);
      initializationSucceeded = new YoBoolean("initializationSucceeded", toolboxMainRegistry);
      yoGraphicsListRegistry = new YoGraphicsListRegistry();

      DRCRobotModel robotModel = getRobotModel();
      avatarKinematicsSimulationController = new AvatarKinematicsSimulationController(robotModel, integrationDT, yoGraphicsListRegistry, toolboxMainRegistry);

      DRCRobotModel ghostRobotModel = getGhostRobotModel();
      RobotDescription robotDescription = ghostRobotModel.getRobotDescription();
      robotDescription.setName("Ghost");
      KinematicsToolboxControllerTest.recursivelyModifyGraphics(robotDescription.getChildrenJoints().get(0), ghostApperance);
      ghost = ghostRobotModel.createHumanoidFloatingRootJointRobot(false);
      ghost.setDynamic(false);
      ghost.setGravity(0);
   }

   @AfterEach
   public void teardown()
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

      if (scs != null)
      {
         scs.closeAndDispose();
         scs = null;
      }

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }

   @Test
   public void testWalkingPreviewAlone() throws SimulationExceededMaximumTimeException, ControllerFailureException
   {
      double dt = 0.02;
      setup(dt);
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      HumanoidFloatingRootJointRobot robot = getRobotModel().createHumanoidFloatingRootJointRobot(false);
      robot.setDynamic(false);
      toolboxUpdater = createToolboxUpdater(robot);
      robot.setController(toolboxUpdater);

      scs = new SimulationConstructionSet(robot, simulationTestingParameters);
      scs.setDT(avatarKinematicsSimulationController.getIntegrationDT(), 1);
      scs.addYoGraphicsListRegistry(yoGraphicsListRegistry, true);
      scs.setCameraFix(0.0, 0.0, 1.0);
      scs.setCameraPosition(8.0, 0.0, 3.0);
      scs.startOnAThread();
      if (simulationTestingParameters.getCreateGUI())
      {
         SimulationOverheadPlotterFactory plotterFactory = scs.createSimulationOverheadPlotterFactory();
         plotterFactory.setShowOnStart(true);
         plotterFactory.addYoGraphicsListRegistries(yoGraphicsListRegistry);
         plotterFactory.createOverheadPlotter();
      }

      getRobotModel().getDefaultRobotInitialSetup(0.0, 0.0).initializeRobot(robot, getRobotModel().getJointMap());
      FullHumanoidRobotModel fullRobotModelAtInitialConfiguration = createFullRobotModelAtInitialConfiguration(2.0);
      RobotConfigurationData robotConfigurationData = HumanoidKinematicsToolboxControllerTest.extractRobotConfigurationData(fullRobotModelAtInitialConfiguration);
      avatarKinematicsSimulationController.updateRobotConfigurationData(robotConfigurationData);

      SideDependentList<Pose3DReadOnly> footPoses = new SideDependentList<>();

      for (RobotSide robotSide : RobotSide.values)
      {
         FramePose3D footPose = new FramePose3D(fullRobotModelAtInitialConfiguration.getSoleFrame(robotSide));
         footPose.changeFrame(worldFrame);
         footPoses.put(robotSide, footPose);
      }

      FootstepDataListMessage footstepDataListMessage = new FootstepDataListMessage();
      Object<FootstepDataMessage> footstepDataList = footstepDataListMessage.getFootstepDataList();
      int numberOfFootsteps = 10;

      for (int i = 0; i < numberOfFootsteps; i++)
      {
         RobotSide side = RobotSide.values[i % 2];
         footstepDataList.add().set(HumanoidMessageTools.createFootstepDataMessage(side,
                                                                                   footPoses.get(side).getPosition(),
                                                                                   footPoses.get(side).getOrientation()));
      }

      FootstepDataListCommand footstepDataListCommand = new FootstepDataListCommand();
      footstepDataListCommand.setFromMessage(footstepDataListMessage);

      LogTools.error("SUBMITTING MESSAGE FootstepList size: {}", footstepDataListCommand.getFootsteps().size());
      avatarKinematicsSimulationController.getInputManager().submitCommand(footstepDataListCommand);

      enableToolboxUpdater.set(true);

//      MutableObject<?> latestOutput = new MutableObject<>();
//      avatarKinematicsSimulationController.getWalkingOutputManager().attachGlobalStatusMessageListener(obj -> latestOutput.setValue(obj));
      int expectedNumberOfFrames = 0;

      for (int i = 0; i < 1000; i++)
      {
         toolboxUpdater.doControl();

         if (avatarKinematicsSimulationController.isWalkingControllerResetDone())
            expectedNumberOfFrames++;

         scs.tickAndUpdate();

         if (avatarKinematicsSimulationController.isDone())
            break;
      }

      enableToolboxUpdater.set(false);

//      assertNotNull(latestOutput.getValue());
//      assertEquals(dt, latestOutput.getValue().getFrameDt(), EPSILON);
//      int actualNumberOfFrames = latestOutput.getValue().getRobotConfigurations().size();
//      assertEquals(expectedNumberOfFrames, actualNumberOfFrames);
   }

   private RobotController createToolboxUpdater(us.ihmc.simulationconstructionset.Robot robotToUpdate)
   {
      return new RobotController()
      {
         private final JointAnglesWriter jointAnglesWriter = new JointAnglesWriter(robotToUpdate, avatarKinematicsSimulationController.getFullRobotModel());

         @Override
         public void doControl()
         {
            if (enableToolboxUpdater.getValue())
            {
               if (!initializationSucceeded.getBooleanValue())
                  initializationSucceeded.set(avatarKinematicsSimulationController.initialize());

               if (initializationSucceeded.getBooleanValue() && !pauseToolboxUpdater.getValue())
               {
                  avatarKinematicsSimulationController.updateInternal();
                  jointAnglesWriter.updateRobotConfigurationBasedOnFullRobotModel();
               }
            }
         }

         @Override
         public void initialize()
         {
         }

         @Override
         public YoVariableRegistry getYoVariableRegistry()
         {
            return toolboxMainRegistry;
         }

         @Override
         public String getName()
         {
            return toolboxMainRegistry.getName();
         }
      };
   }

   private FullHumanoidRobotModel createFullRobotModelAtInitialConfiguration(double initialYaw)
   {
      DRCRobotModel robotModel = getRobotModel();
      FullHumanoidRobotModel initialFullRobotModel = robotModel.createFullRobotModel();
      HumanoidFloatingRootJointRobot robot = robotModel.createHumanoidFloatingRootJointRobot(false);
      robotModel.getDefaultRobotInitialSetup(0.0, initialYaw).initializeRobot(robot, robotModel.getJointMap());
      DRCPerfectSensorReaderFactory drcPerfectSensorReaderFactory = new DRCPerfectSensorReaderFactory(robot, 0);
      drcPerfectSensorReaderFactory.build(initialFullRobotModel.getRootJoint(), null, null, null, null);
      SensorDataContext sensorDataContext = new SensorDataContext();
      long timestamp = drcPerfectSensorReaderFactory.getSensorReader().read(sensorDataContext);
      drcPerfectSensorReaderFactory.getSensorReader().compute(timestamp, sensorDataContext);
      return initialFullRobotModel;
   }
}
