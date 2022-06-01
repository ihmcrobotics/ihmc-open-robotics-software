package us.ihmc.valkyrie.externalForceEstimation;

import static us.ihmc.avatar.testTools.scs2.YoGraphicDefinitionFactory.newYoGraphicArrow3DDefinition;
import static us.ihmc.avatar.testTools.scs2.YoGraphicDefinitionFactory.newYoGraphicPoint3DDefinition;

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
import us.ihmc.avatar.testTools.scs2.SCS2AvatarTestingSimulation;
import us.ihmc.avatar.testTools.scs2.SCS2AvatarTestingSimulationFactory;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.scs2.definition.controller.interfaces.Controller;
import us.ihmc.scs2.definition.robot.ExternalWrenchPointDefinition;
import us.ihmc.scs2.definition.visual.ColorDefinition;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicArrow3DDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicGroupDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicPoint3DDefinition;
import us.ihmc.scs2.simulation.TimeConsumer;
import us.ihmc.scs2.simulation.robot.Robot;
import us.ihmc.scs2.simulation.robot.multiBodySystem.interfaces.SimJointBasics;
import us.ihmc.scs2.simulation.robot.trackers.ExternalWrenchPoint;
import us.ihmc.simulationConstructionSetTools.util.environments.FlatGroundEnvironment;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.valkyrie.ValkyrieRobotModel;
import us.ihmc.valkyrie.configuration.ValkyrieRobotVersion;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
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
   private SCS2AvatarTestingSimulation simulationTestHelper;
   private ExternalForceEstimationToolboxController toolboxController;

   private List<TestConfig> testConfigs;
   private AtomicInteger testIndex = new AtomicInteger();

   private class TestConfig
   {
      String endEffectorName;
      Vector3D efpOffset;
      Vector3D desiredSimulatedForceInWorld = new Vector3D();
      ExternalWrenchPointDefinition externalWrenchPointDefinition;
      RigidBodyBasics endEffector;
      YoFrameVector3D estimatedForce;

      ExternalWrenchPoint externalWrenchPoint;

      public TestConfig(String endEffectorName, Vector3D efpOffset)
      {
         this.endEffectorName = endEffectorName;
         this.efpOffset = efpOffset;
         this.externalWrenchPointDefinition = new ExternalWrenchPointDefinition("efp_" + endEffectorName, efpOffset);
         this.estimatedForce = new YoFrameVector3D(endEffectorName + "estimatedForce", ReferenceFrame.getWorldFrame(), registry);
      }

      public void applyDesiredForce()
      {
         externalWrenchPoint.getWrench().getLinearPart().setMatchingFrame(externalWrenchPoint.getFrame().getRootFrame(), desiredSimulatedForceInWorld);
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
      simulationTestHelper = SCS2AvatarTestingSimulationFactory.createDefaultTestSimulation(robotModel, testEnvironment, simulationTestingParameters);

      AtomicBoolean initializeToolbox = new AtomicBoolean(true);
      AtomicBoolean updateToolbox = new AtomicBoolean();

      simulationTestHelper.getRobot().addThrottledController(new Controller()
      {
         @Override
         public void doControl()
         {
            if (toolboxController == null)
               return;
            if (initializeToolbox.getAndSet(false))
               toolboxController.initialize();
            if (updateToolbox.get())
               toolboxController.update();
         }
      }, ExternalForceEstimationToolboxModule.UPDATE_PERIOD_MILLIS * 1.0e-3);

      simulationTestHelper.start();
      simulationTestHelper.getRootRegistry().addChild(registry);

      YoGraphicsListRegistry graphicsListRegistry = new YoGraphicsListRegistry();
      YoRegistry scsRootRegistry = simulationTestHelper.getRootRegistry();

      Robot scsRobot = simulationTestHelper.getRobot();
      FullHumanoidRobotModel fullRobotModel = robotModel.createFullRobotModel();

      for (int i = 0; i < testConfigs.size(); i++)
      {
         TestConfig testConfig = testConfigs.get(i);

         SimJointBasics scsEndEffector = scsRobot.getJoint(testConfig.endEffectorName);
         testConfig.externalWrenchPoint = scsEndEffector.getAuxialiryData().addExternalWrenchPoint(testConfig.externalWrenchPointDefinition);
         JointBasics joint = fullRobotModel.getOneDoFJointByName(testConfig.endEffectorName);
         testConfig.endEffector = joint.getSuccessor();

         ColorDefinition simulatedForceColor = ColorDefinitions.Red();
         ColorDefinition estimatedForceColor = ColorDefinitions.Green();

         YoFramePoint3D efp_point = testConfig.externalWrenchPoint.getOffset().getPosition();
         YoFrameVector3D simulatedForce = testConfig.externalWrenchPoint.getWrench().getLinearPart();
         YoGraphicArrow3DDefinition simulatedFoceViz = newYoGraphicArrow3DDefinition("simulatedForceVector"
               + i, efp_point, simulatedForce, forceGraphicScale, simulatedForceColor);
         YoGraphicArrow3DDefinition estimatedForceViz = newYoGraphicArrow3DDefinition("estimatedForceVector"
               + i, efp_point, testConfig.estimatedForce, forceGraphicScale, estimatedForceColor);
         YoGraphicPoint3DDefinition simulatedPointViz = newYoGraphicPoint3DDefinition("simulatedForcePoint" + i, efp_point, 0.025, simulatedForceColor);
         simulationTestHelper.addYoGraphicDefinition(new YoGraphicGroupDefinition("externalForceVectors",
                                                                                  simulatedFoceViz,
                                                                                  estimatedForceViz,
                                                                                  simulatedPointViz));
      }

      simulationTestHelper.addYoGraphicsListRegistry(graphicsListRegistry);

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
      simulationTestHelper.createSubscriberFromController(RobotConfigurationData.class, toolboxController::updateRobotConfigurationData);
      simulationTestHelper.createSubscriberFromController(RobotDesiredConfigurationData.class, toolboxController::updateRobotDesiredConfigurationData);
      statusOutputManager.attachStatusMessageListener(ExternalForceEstimationOutputStatus.class, status ->
      {
         for (int i = 0; i < testConfigs.size(); i++)
         {
            if (i == testIndex.get())
               testConfigs.get(testIndex.get()).estimatedForce.set(status.getEstimatedExternalForces().get(0));
            else
               testConfigs.get(i).estimatedForce.set(0.0, 0.0, 0.0);
         }
      });

      boolean success = simulationTestHelper.simulateNow(1.0);
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
            testConfigs.get(i).desiredSimulatedForceInWorld.set(force);
            TimeConsumer forceUpdater = time -> testConfig.applyDesiredForce();
            simulationTestHelper.getSimulationConstructionSet().addBeforePhysicsCallback(forceUpdater);

            initializeToolbox.set(true);
            updateToolbox.set(true);
            simulationTestHelper.simulateNow(5.0);
            Assertions.assertTrue(force.epsilonEquals(testConfig.estimatedForce, epsilon),
                                  "Estimator failed to estimate force applied on " + testConfig.endEffectorName + ", simulated force: " + force
                                        + ", estimated force: " + testConfig.estimatedForce);

            simulationTestHelper.getSimulationConstructionSet().removeBeforePhysicsCallback(forceUpdater);
         }

         testConfigs.get(i).desiredSimulatedForceInWorld.setToZero();
      }
   }

   @AfterEach
   public void tearDown()
   {
      simulationTestHelper.finishTest();

      robotModel = null;
      simulationTestHelper = null;
      toolboxController = null;
      testConfigs = null;
      registry = null;
   }
}
