package us.ihmc.valkyrie.externalForceEstimation;

import static us.ihmc.avatar.scs2.YoGraphicDefinitionFactory.newYoGraphicArrow3DDefinition;
import static us.ihmc.avatar.scs2.YoGraphicDefinitionFactory.newYoGraphicPoint3DDefinition;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicReference;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;

import controller_msgs.msg.dds.ExternalForceEstimationConfigurationMessage;
import controller_msgs.msg.dds.ExternalForceEstimationOutputStatus;
import controller_msgs.msg.dds.RobotConfigurationData;
import controller_msgs.msg.dds.RobotDesiredConfigurationData;
import gnu.trove.map.hash.TIntObjectHashMap;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.networkProcessor.externalForceEstimationToolboxModule.ExternalForceEstimationToolboxController;
import us.ihmc.avatar.networkProcessor.externalForceEstimationToolboxModule.ExternalForceEstimationToolboxModule;
import us.ihmc.avatar.testTools.scs2.SCS2AvatarTestingSimulation;
import us.ihmc.avatar.testTools.scs2.SCS2AvatarTestingSimulationFactory;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsList;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.scs2.definition.controller.interfaces.Controller;
import us.ihmc.scs2.definition.robot.ExternalWrenchPointDefinition;
import us.ihmc.scs2.definition.visual.ColorDefinition;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.simulation.robot.Robot;
import us.ihmc.scs2.simulation.robot.multiBodySystem.interfaces.SimJointBasics;
import us.ihmc.scs2.simulation.robot.trackers.ExternalWrenchPoint;
import us.ihmc.simulationConstructionSetTools.util.environments.FlatGroundEnvironment;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.valkyrie.ValkyrieRobotModel;
import us.ihmc.valkyrie.configuration.ValkyrieRobotVersion;
import us.ihmc.yoVariables.registry.YoRegistry;

public class ValkyrieExternalContactEstimationTest
{
   protected static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();
   private static final double forceGraphicScale = 0.05;
   private static final double distanceThreshold = 0.08;

   private YoRegistry registry;
   private DRCRobotModel robotModel;
   private SCS2AvatarTestingSimulation simulationTestHelper;
   private ExternalForceEstimationToolboxController toolboxController;

   private List<TestConfig> testConfigs;
   private AtomicInteger testIndex = new AtomicInteger();

   public DRCRobotModel newRobotModel()
   {
      return new ValkyrieRobotModel(RobotTarget.SCS, ValkyrieRobotVersion.FINGERLESS);
   }

   @Test
   public void testExternalContactLocalization() throws Exception
   {
      registry = new YoRegistry(getClass().getSimpleName());

      testConfigs = new ArrayList<>();
      testConfigs.add(new TestConfig("torsoRoll", 0, new Vector3D(0.137, 0.050, 0.329), new Vector3D(-8.0, 0.0, -4.0), registry));
      testConfigs.add(new TestConfig("torsoRoll", 1, new Vector3D(0.156, 0.093, 0.197), new Vector3D(-7.0, -3.0, 0.0), registry));
      testConfigs.add(new TestConfig("leftForearmYaw", 0, new Vector3D(-0.068, 0.170, -0.033), new Vector3D(1.0, -3.0, 1.0), registry));
      testConfigs.add(new TestConfig("rightForearmYaw", 0, new Vector3D(-0.068, -0.170, -0.033), new Vector3D(1.0, 3.0, 1.0), registry));

      runTest();
   }

   private static class TestConfig
   {
      private final String endEffectorJointName;
      private final ExternalWrenchPointDefinition externalWrenchPointDefinition;
      private final Vector3D forceToApply;

      private ExternalWrenchPoint externalWrenchPoint;

      public TestConfig(String endEffectorJointName, int endEffectorIndex, Vector3D efpOffset, Vector3D forceToApply, YoRegistry registry)
      {
         this.endEffectorJointName = endEffectorJointName;
         this.externalWrenchPointDefinition = new ExternalWrenchPointDefinition("efp_" + endEffectorJointName + endEffectorIndex, efpOffset);
         this.forceToApply = forceToApply;
      }

      void enable()
      {
         externalWrenchPoint.getWrench().getLinearPart().setMatchingFrame(externalWrenchPoint.getFrame().getRootFrame(), forceToApply);
      }

      void disable()
      {
         externalWrenchPoint.getWrench().setToZero();
      }
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

         @Override
         public YoRegistry getYoRegistry()
         {
            return new YoRegistry("toolboxUpdater");
         }
      }, ExternalForceEstimationToolboxModule.UPDATE_PERIOD_MILLIS * 1.0e-3);

      simulationTestHelper.start();
      simulationTestHelper.getRootRegistry().addChild(registry);

      YoGraphicsListRegistry graphicsListRegistry = new YoGraphicsListRegistry();
      YoRegistry scsRootRegistry = simulationTestHelper.getRootRegistry();

      Robot scsRobot = simulationTestHelper.getRobot();
      FullHumanoidRobotModel fullRobotModel = robotModel.createFullRobotModel();
      YoGraphicsList externalForcePointViz = new YoGraphicsList("externalForceVectors");

      TIntObjectHashMap<RigidBodyBasics> rigidBodyHashMap = new TIntObjectHashMap<>();
      MultiBodySystemTools.getRootBody(fullRobotModel.getElevator()).subtreeIterable()
                          .forEach(rigidBody -> rigidBodyHashMap.put(rigidBody.hashCode(), rigidBody));

      for (int i = 0; i < testConfigs.size(); i++)
      {
         TestConfig testConfig = testConfigs.get(i);

         SimJointBasics scsEndEffector = scsRobot.getJoint(testConfig.endEffectorJointName);
         testConfig.externalWrenchPoint = scsEndEffector.getAuxialiryData().addExternalWrenchPoint(testConfig.externalWrenchPointDefinition);

         ColorDefinition simulatedForceColor = ColorDefinitions.Red();
         simulationTestHelper.addYoGraphicDefinition(newYoGraphicArrow3DDefinition("simulatedForceVector",
                                                                                   testConfig.externalWrenchPoint.getOffset().getPosition(),
                                                                                   testConfig.externalWrenchPoint.getWrench().getLinearPart(),
                                                                                   forceGraphicScale,
                                                                                   simulatedForceColor));
         simulationTestHelper.addYoGraphicDefinition(newYoGraphicPoint3DDefinition("simulatedForcePoint",
                                                                                   testConfig.externalWrenchPoint.getOffset().getPosition(),
                                                                                   0.025,
                                                                                   simulatedForceColor));
      }

      graphicsListRegistry.registerYoGraphicsList(externalForcePointViz);
      CommandInputManager commandInputManager = new CommandInputManager(getClass().getSimpleName(),
                                                                        ExternalForceEstimationToolboxModule.getSupportedCommands());
      StatusMessageOutputManager statusOutputManager = new StatusMessageOutputManager(ExternalForceEstimationToolboxModule.getSupportedStatuses());
      toolboxController = new ExternalForceEstimationToolboxController(robotModel,
                                                                       fullRobotModel,
                                                                       commandInputManager,
                                                                       statusOutputManager,
                                                                       graphicsListRegistry,
                                                                       ExternalForceEstimationToolboxModule.UPDATE_PERIOD_MILLIS,
                                                                       scsRootRegistry);

      simulationTestHelper.addYoGraphicsListRegistry(graphicsListRegistry);
      simulationTestHelper.createSubscriberFromController(RobotConfigurationData.class, toolboxController::updateRobotConfigurationData);
      simulationTestHelper.createSubscriberFromController(RobotDesiredConfigurationData.class, toolboxController::updateRobotDesiredConfigurationData);

      AtomicReference<ExternalForceEstimationOutputStatus> outputStatusReference = new AtomicReference<>();
      statusOutputManager.attachStatusMessageListener(ExternalForceEstimationOutputStatus.class, outputStatusReference::set);

      boolean success = simulationTestHelper.simulateNow(1.0);
      Assertions.assertTrue(success);

      for (int i = 0; i < testConfigs.size(); i++)
      {
         testIndex.set(i);
         TestConfig testConfig = testConfigs.get(i);

         ExternalForceEstimationConfigurationMessage configurationMessage = new ExternalForceEstimationConfigurationMessage();
         configurationMessage.setEstimatorGain(0.8);
         configurationMessage.setSolverAlpha(0.001);
         configurationMessage.setEstimateContactLocation(true);
         commandInputManager.submitMessage(configurationMessage);

         testConfigs.get(i).enable();

         initializeToolbox.set(true);
         updateToolbox.set(true);
         simulationTestHelper.simulateNow(10.0);

         Point3D actualContactPointInWorld = new Point3D(testConfig.externalWrenchPoint.getFrame().getTransformToRoot().getTranslation());

         Point3D estimatedContactPoint = outputStatusReference.get().getContactPoint();
         int estimatedHashcode = outputStatusReference.get().getRigidBodyHashCode();

         RigidBodyBasics estimatedLink = rigidBodyHashMap.get(estimatedHashcode);
         if (estimatedLink == null)
            Assertions.fail("Estimator did not converge for " + testConfig.endEffectorJointName);

         FramePoint3D estimatedFramePoint = new FramePoint3D(estimatedLink.getParentJoint().getFrameAfterJoint(), estimatedContactPoint);
         estimatedFramePoint.changeFrame(ReferenceFrame.getWorldFrame());

         double distance = actualContactPointInWorld.distance(estimatedFramePoint);
         Assertions.assertTrue(distance < distanceThreshold, "Estimated contact point too far away for " + testConfig.endEffectorJointName);

         testConfigs.get(i).disable();
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
