package us.ihmc.valkyrie.externalForceEstimation;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;

import controller_msgs.msg.dds.ExternalForceEstimationConfigurationMessage;
import controller_msgs.msg.dds.ExternalForceEstimationOutputStatus;
import controller_msgs.msg.dds.RobotConfigurationData;
import controller_msgs.msg.dds.RobotDesiredConfigurationData;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.networkProcessor.externalForceEstimationToolboxModule.ExternalForceEstimationToolboxController;
import us.ihmc.avatar.networkProcessor.externalForceEstimationToolboxModule.ExternalForceEstimationToolboxModule;
import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicVector;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsList;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.simulationConstructionSetTools.util.HumanoidFloatingRootJointRobot;
import us.ihmc.simulationConstructionSetTools.util.environments.FlatGroundEnvironment;
import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.Joint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.valkyrie.ValkyrieRobotModel;
import us.ihmc.valkyrie.configuration.ValkyrieRobotVersion;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;

public class ValkyrieExternalForceEstimationTest
{
   protected static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();
   private static final double forceGraphicScale = 0.05;

   private static final Random random = new Random(3290);
   private static final int iterations = 4;
   private static final double epsilon = 0.35;
   private static final double forceMagnitude = 15.0;

   private YoRegistry registry;
   private DRCRobotModel robotModel;
   private DRCSimulationTestHelper drcSimulationTestHelper;
   private ExternalForceEstimationToolboxController toolboxController;

   private List<TestConfig> testConfigs;
   private AtomicInteger testIndex = new AtomicInteger();

   private class TestConfig
   {
      String endEffectorName;
      Vector3D efpOffset;
      ExternalForcePoint externalForcePoint;
      RigidBodyBasics endEffector;
      YoFrameVector3D estimatedForce;

      public TestConfig(String endEffectorName, Vector3D efpOffset)
      {
         this.endEffectorName = endEffectorName;
         this.efpOffset = efpOffset;
         this.externalForcePoint = new ExternalForcePoint("efp_" + endEffectorName, efpOffset, registry);
         this.estimatedForce = new YoFrameVector3D(endEffectorName + "estimatedForce", ReferenceFrame.getWorldFrame(), registry);
      }
   }

   public DRCRobotModel newRobotModel()
   {
      return new ValkyrieRobotModel(RobotTarget.SCS, ValkyrieRobotVersion.ARM_MASS_SIM);
   }

   @Test
   public void testExternalForceEstimation() throws Exception
   {
      registry = new YoRegistry(getClass().getSimpleName());

      testConfigs = new ArrayList<>();
      testConfigs.add(new TestConfig("rightElbowPitch", new Vector3D(0.0, -0.25, 0.0)));
      testConfigs.add(new TestConfig("leftShoulderPitch", new Vector3D(0.1, 0.1, 0.0)));
      testConfigs.add(new TestConfig("torsoRoll", new Vector3D(0.0, 0.0, 0.2)));

      runTest();
   }

   private void runTest() throws Exception
   {
      robotModel = newRobotModel();
      FlatGroundEnvironment testEnvironment = new FlatGroundEnvironment();
      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, newRobotModel(), testEnvironment);

      AtomicBoolean initializeToolbox = new AtomicBoolean(true);
      AtomicBoolean updateToolbox = new AtomicBoolean();

      Robot dummyRobot = new Robot("dummyRobot");
      dummyRobot.setController(new RobotController()
      {
         @Override
         public void doControl()
         {
            if(toolboxController == null)
               return;
            if(initializeToolbox.getAndSet(false))
               toolboxController.initialize();
            if(updateToolbox.get())
               toolboxController.update();
         }

         @Override
         public void initialize()
         {

         }

         @Override
         public YoRegistry getYoRegistry()
         {
            return new YoRegistry("toolboxUpdater");
         }
      }, (int) (ExternalForceEstimationToolboxModule.UPDATE_PERIOD_MILLIS / (1000 * robotModel.getSimulateDT())));
      testEnvironment.addEnvironmentRobot(dummyRobot);

      drcSimulationTestHelper.createSimulation("external_force_estimation_test", false, true);
      drcSimulationTestHelper.getYoVariableRegistry().addChild(registry);

      YoGraphicsListRegistry graphicsListRegistry = new YoGraphicsListRegistry();
      YoRegistry scsRootRegistry = drcSimulationTestHelper.getAvatarSimulation().getSimulationConstructionSet().getRootRegistry();

      HumanoidFloatingRootJointRobot scsRobot = drcSimulationTestHelper.getRobot();
      FullHumanoidRobotModel fullRobotModel = robotModel.createFullRobotModel();
      YoGraphicsList externalForcePointViz = new YoGraphicsList("externalForceVectors");

      for (int i = 0; i < testConfigs.size(); i++)
      {
         TestConfig testConfig = testConfigs.get(i);

         Joint scsEndEffector = scsRobot.getJoint(testConfig.endEffectorName);
         scsEndEffector.addExternalForcePoint(testConfig.externalForcePoint);
         JointBasics joint = fullRobotModel.getOneDoFJointByName(testConfig.endEffectorName);
         testConfig.endEffector = joint.getSuccessor();

         AppearanceDefinition simulatedForceColor = YoAppearance.Red();
         AppearanceDefinition estimatedForceColor = YoAppearance.Green();

         YoGraphicVector simulatedForceVector = new YoGraphicVector("simulatedForceVector",
                                                                    testConfig.externalForcePoint.getYoPosition(),
                                                                    testConfig.externalForcePoint.getYoForce(),
                                                                    forceGraphicScale,
                                                                    simulatedForceColor);

         YoGraphicVector estimatedForceVector = new YoGraphicVector("estimatedForceVector",
                                                                    testConfig.externalForcePoint.getYoPosition(),
                                                                    testConfig.estimatedForce,
                                                                    forceGraphicScale,
                                                                    estimatedForceColor);

         YoGraphicPosition simulatedForcePoint = new YoGraphicPosition("simulatedForcePoint", testConfig.externalForcePoint.getYoPosition(), 0.025, simulatedForceColor);

         externalForcePointViz.add(simulatedForceVector);
         externalForcePointViz.add(estimatedForceVector);
         externalForcePointViz.add(simulatedForcePoint);
      }

      graphicsListRegistry.registerYoGraphicsList(externalForcePointViz);

      drcSimulationTestHelper.getAvatarSimulation().getSimulationConstructionSet().addYoGraphicsListRegistry(graphicsListRegistry);

      CommandInputManager commandInputManager = new CommandInputManager(getClass().getSimpleName(),
                                                                        ExternalForceEstimationToolboxModule.getSupportedCommands());
      StatusMessageOutputManager statusOutputManager = new StatusMessageOutputManager(ExternalForceEstimationToolboxModule.getSupportedStatuses());
      toolboxController = new ExternalForceEstimationToolboxController(robotModel,
                                                                       robotModel.createFullRobotModel(),
                                                                       commandInputManager,
                                                                       statusOutputManager,
                                                                       null,
                                                                       ExternalForceEstimationToolboxModule.UPDATE_PERIOD_MILLIS,
                                                                       scsRootRegistry);
      drcSimulationTestHelper.createSubscriberFromController(RobotConfigurationData.class, toolboxController::updateRobotConfigurationData);
      drcSimulationTestHelper.createSubscriberFromController(RobotDesiredConfigurationData.class, toolboxController::updateRobotDesiredConfigurationData);
      statusOutputManager.attachStatusMessageListener(ExternalForceEstimationOutputStatus.class, status ->
      {
         for (int i = 0; i < testConfigs.size(); i++)
         {
            if(i == testIndex.get())
               testConfigs.get(testIndex.get()).estimatedForce.set(status.getEstimatedExternalForces().get(0));
            else
               testConfigs.get(i).estimatedForce.set(0.0, 0.0, 0.0);
         }
      });

      drcSimulationTestHelper.getAvatarSimulation().start();

      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      Assertions.assertTrue(success);

      for (int i = 0; i < testConfigs.size(); i++)
      {
         testIndex.set(i);
         TestConfig testConfig = testConfigs.get(i);

         ExternalForceEstimationConfigurationMessage configurationMessage = new ExternalForceEstimationConfigurationMessage();
         configurationMessage.setEstimatorGain(0.8);
         configurationMessage.getRigidBodyHashCodes().add(testConfig.endEffector.hashCode());
         configurationMessage.getContactPointPositions().add().set(testConfig.efpOffset);
         configurationMessage.setSolverAlpha(0.001);
         configurationMessage.setCalculateRootJointWrench(false);
         commandInputManager.submitMessage(configurationMessage);

         for (int j = 0; j < iterations; j++)
         {
            Vector3D force = EuclidCoreRandomTools.nextVector3DWithFixedLength(random, j == 0 ? 0.0 : forceMagnitude);
            testConfigs.get(i).externalForcePoint.setForce(force);

            initializeToolbox.set(true);
            updateToolbox.set(true);
            drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(5.0);
            Assertions.assertTrue(force.epsilonEquals(testConfig.estimatedForce, epsilon),
                                  "Estimator failed to estimate force applied on " + testConfig.endEffectorName + ", simulated force: " + force
                                  + ", estimated force: " + testConfig.estimatedForce);
         }

         testConfigs.get(i).externalForcePoint.setForce(0.0, 0.0, 0.0);
      }
   }

   @AfterEach
   public void tearDown()
   {
      drcSimulationTestHelper.destroySimulation();

      robotModel = null;
      drcSimulationTestHelper = null;
      toolboxController = null;
      testConfigs = null;
      registry = null;
   }
}
