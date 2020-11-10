package us.ihmc.valkyrie.externalForceEstimation;

import controller_msgs.msg.dds.ExternalForceEstimationConfigurationMessage;
import controller_msgs.msg.dds.ExternalForceEstimationOutputStatus;
import controller_msgs.msg.dds.RobotConfigurationData;
import controller_msgs.msg.dds.RobotDesiredConfigurationData;
import gnu.trove.list.array.TFloatArrayList;
import gnu.trove.map.hash.TIntObjectHashMap;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.networkProcessor.externalForceEstimationToolboxModule.ExternalForceEstimationToolboxController;
import us.ihmc.avatar.networkProcessor.externalForceEstimationToolboxModule.ExternalForceEstimationToolboxModule;
import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicVector;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsList;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.tools.MultiBodySystemTools;
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
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicReference;

public class ValkyrieExternalContactEstimationTest
{
   protected static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();
   private static final double forceGraphicScale = 0.05;
   private static final double distanceThreshold = 0.08;

   private YoRegistry registry;
   private DRCRobotModel robotModel;
   private DRCSimulationTestHelper drcSimulationTestHelper;
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
      private final String endEffectorName;
      private final ExternalForcePoint externalForcePoint;
      private final Vector3D forceToApply;

      private RigidBodyBasics endEffector;

      public TestConfig(String endEffectorName, int endEffectorIndex, Vector3D efpOffset, Vector3D forceToApply, YoRegistry registry)
      {
         this.endEffectorName = endEffectorName;
         this.externalForcePoint = new ExternalForcePoint("efp_" + endEffectorName + endEffectorIndex, efpOffset, registry);
         this.forceToApply = forceToApply;
      }

      void enable()
      {
         externalForcePoint.setForce(forceToApply);
      }

      void disable()
      {
         externalForcePoint.setForce(0.0, 0.0, 0.0);
      }
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

      TIntObjectHashMap<RigidBodyBasics> rigidBodyHashMap = new TIntObjectHashMap<>();
      MultiBodySystemTools.getRootBody(fullRobotModel.getElevator())
                          .subtreeIterable()
                          .forEach(rigidBody -> rigidBodyHashMap.put(rigidBody.hashCode(), rigidBody));

      for (int i = 0; i < testConfigs.size(); i++)
      {
         TestConfig testConfig = testConfigs.get(i);

         Joint scsEndEffector = scsRobot.getJoint(testConfig.endEffectorName);
         scsEndEffector.addExternalForcePoint(testConfig.externalForcePoint);
         JointBasics joint = fullRobotModel.getOneDoFJointByName(testConfig.endEffectorName);
         testConfig.endEffector = joint.getSuccessor();

         AppearanceDefinition simulatedForceColor = YoAppearance.Red();
         YoGraphicVector simulatedForceVector = new YoGraphicVector("simulatedForceVector",
                                                                    testConfig.externalForcePoint.getYoPosition(),
                                                                    testConfig.externalForcePoint.getYoForce(),
                                                                    forceGraphicScale,
                                                                    simulatedForceColor);
         YoGraphicPosition simulatedForcePoint = new YoGraphicPosition("simulatedForcePoint",
                                                                       testConfig.externalForcePoint.getYoPosition(),
                                                                       0.025,
                                                                       simulatedForceColor);

         externalForcePointViz.add(simulatedForceVector);
         externalForcePointViz.add(simulatedForcePoint);
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

      drcSimulationTestHelper.getAvatarSimulation().getSimulationConstructionSet().addYoGraphicsListRegistry(graphicsListRegistry);
      drcSimulationTestHelper.createSubscriberFromController(RobotConfigurationData.class, toolboxController::updateRobotConfigurationData);
      drcSimulationTestHelper.createSubscriberFromController(RobotDesiredConfigurationData.class, toolboxController::updateRobotDesiredConfigurationData);
      drcSimulationTestHelper.getAvatarSimulation().start();

      AtomicReference<ExternalForceEstimationOutputStatus> outputStatusReference = new AtomicReference<>();
      statusOutputManager.attachStatusMessageListener(ExternalForceEstimationOutputStatus.class, outputStatusReference::set);

      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);
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
         drcSimulationTestHelper.simulateAndBlock(10.0);

         Point3D actualContactPointInWorld = testConfig.externalForcePoint.getPositionCopy();

         Point3D estimatedContactPoint = outputStatusReference.get().getContactPoint();
         int estimatedHashcode = outputStatusReference.get().getRigidBodyHashCode();

         RigidBodyBasics estimatedLink = rigidBodyHashMap.get(estimatedHashcode);
         if (estimatedLink == null)
            Assertions.fail("Estimator did not converge for " + testConfig.endEffectorName);

         FramePoint3D estimatedFramePoint = new FramePoint3D(estimatedLink.getParentJoint().getFrameAfterJoint(), estimatedContactPoint);
         estimatedFramePoint.changeFrame(ReferenceFrame.getWorldFrame());

         double distance = actualContactPointInWorld.distance(estimatedFramePoint);
         Assertions.assertTrue(distance < distanceThreshold, "Estimated contact point too far away for " + testConfig.endEffectorName);

         testConfigs.get(i).disable();
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
