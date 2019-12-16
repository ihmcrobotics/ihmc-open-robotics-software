package us.ihmc.valkyrie.torquespeedcurve;

import java.util.ArrayList;
import java.util.List;

import org.junit.jupiter.api.Test;

import us.ihmc.avatar.SimulatedLowLevelOutputWriter;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.initialSetup.DRCSCSInitialSetup;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ContactableBodiesFactory;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.contactable.ContactablePlaneBody;
import us.ihmc.robotics.partNames.ArmJointName;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.partNames.SpineJointName;
import us.ihmc.robotics.robotController.InitializingRobotController;
import us.ihmc.robotics.robotController.ModularRobotController;
import us.ihmc.robotics.robotController.RawOutputWriter;
import us.ihmc.robotics.robotController.RawSensorReader;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.sensorProcessing.simulatedSensors.DRCPerfectSensorReader;
import us.ihmc.sensorProcessing.simulatedSensors.DRCPerfectSensorReaderFactory;
import us.ihmc.simulationConstructionSetTools.util.HumanoidFloatingRootJointRobot;
import us.ihmc.simulationConstructionSetTools.util.environments.FlatGroundEnvironment;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.gui.tools.SimulationOverheadPlotterFactory;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.valkyrie.ValkyrieRobotModel;
import us.ihmc.valkyrie.configuration.ValkyrieRobotVersion;
import us.ihmc.valkyrie.parameters.ValkyrieJointMap;
import us.ihmc.wholeBodyController.DRCRobotJointMap;
import us.ihmc.wholeBodyController.RobotContactPointParameters;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class ValkyrieMultiContactStaticPoseEndToEndTest
{
   @Test
   public void testCrawl1()
   {
      ValkyrieRobotModel robotModel = new ValkyrieRobotModel(RobotTarget.SCS, ValkyrieRobotVersion.FINGERLESS);
      ValkyrieMultiContactPointParameters contactPointParameters = new ValkyrieMultiContactPointParameters(robotModel.getJointMap(),
                                                                                                           robotModel.getRobotPhysicalProperties());
      addUpperKneeContactPoints(robotModel.getJointMap(), contactPointParameters);
      addToeFrontContactPoints(robotModel.getJointMap(), contactPointParameters);
      addElbowContactPoints(robotModel.getJointMap(), contactPointParameters);
      robotModel.setContactPointParameters(contactPointParameters);

      FullHumanoidRobotModel fullRobotModel = robotModel.createFullRobotModel();
      HumanoidFloatingRootJointRobot robot = robotModel.createHumanoidFloatingRootJointRobot(false);

      YoGraphicsListRegistry yoGraphicsListRegistry = setupController(robotModel, fullRobotModel, robot);
      SimulationConstructionSet scs = setupSCS(robotModel, robot, yoGraphicsListRegistry);

      setRobotToCrawl1Configuration(robot, robotModel.getJointMap());
      scs.startOnAThread();

      ThreadTools.sleepForever();
   }

   @Test
   public void testCrawl2()
   {
      ValkyrieRobotModel robotModel = new ValkyrieRobotModel(RobotTarget.SCS, ValkyrieRobotVersion.FINGERLESS);
      ValkyrieMultiContactPointParameters contactPointParameters = new ValkyrieMultiContactPointParameters(robotModel.getJointMap(),
                                                                                                           robotModel.getRobotPhysicalProperties());
      addLowerKneeContactPoints(robotModel.getJointMap(), contactPointParameters);
      addToeFrontContactPoints(robotModel.getJointMap(), contactPointParameters);
      addHandFist1ContactPoints(robotModel.getJointMap(), contactPointParameters);
      robotModel.setContactPointParameters(contactPointParameters);

      FullHumanoidRobotModel fullRobotModel = robotModel.createFullRobotModel();
      HumanoidFloatingRootJointRobot robot = robotModel.createHumanoidFloatingRootJointRobot(false);

      YoGraphicsListRegistry yoGraphicsListRegistry = setupController(robotModel, fullRobotModel, robot);
      SimulationConstructionSet scs = setupSCS(robotModel, robot, yoGraphicsListRegistry);

      setRobotToCrawl2Configuration(robot, robotModel.getJointMap());
      scs.startOnAThread();

      ThreadTools.sleepForever();
   }

   @Test
   public void testKneel1()
   {
      ValkyrieRobotModel robotModel = new ValkyrieRobotModel(RobotTarget.SCS, ValkyrieRobotVersion.FINGERLESS);
      ValkyrieMultiContactPointParameters contactPointParameters = new ValkyrieMultiContactPointParameters(robotModel.getJointMap(),
                                                                                                           robotModel.getRobotPhysicalProperties());
      addLowerKneeContactPoints(robotModel.getJointMap(), contactPointParameters);
      addToeTopContactPoints(robotModel.getJointMap(), contactPointParameters);
      robotModel.setContactPointParameters(contactPointParameters);

      FullHumanoidRobotModel fullRobotModel = robotModel.createFullRobotModel();
      HumanoidFloatingRootJointRobot robot = robotModel.createHumanoidFloatingRootJointRobot(false);

      YoGraphicsListRegistry yoGraphicsListRegistry = setupController(robotModel, fullRobotModel, robot);
      SimulationConstructionSet scs = setupSCS(robotModel, robot, yoGraphicsListRegistry);

      setRobotToKneel1Configuration(robot, robotModel.getJointMap());
      scs.startOnAThread();

      ThreadTools.sleepForever();
   }

   @Test
   public void testKneel2()
   {
      ValkyrieRobotModel robotModel = new ValkyrieRobotModel(RobotTarget.SCS, ValkyrieRobotVersion.FINGERLESS);
      ValkyrieMultiContactPointParameters contactPointParameters = new ValkyrieMultiContactPointParameters(robotModel.getJointMap(),
                                                                                                           robotModel.getRobotPhysicalProperties());
      addLowerKneeContactPoints(robotModel.getJointMap(), contactPointParameters);
      addToeFrontContactPoints(robotModel.getJointMap(), contactPointParameters);
      robotModel.setContactPointParameters(contactPointParameters);

      FullHumanoidRobotModel fullRobotModel = robotModel.createFullRobotModel();
      HumanoidFloatingRootJointRobot robot = robotModel.createHumanoidFloatingRootJointRobot(false);

      double simulationDT = 2.0e-4;
      YoGraphicsListRegistry yoGraphicsListRegistry = setupController(robotModel, fullRobotModel, robot, simulationDT);
      SimulationConstructionSet scs = setupSCS(robotModel, robot, simulationDT, yoGraphicsListRegistry);

      setRobotToKneel2Configuration(robot, robotModel.getJointMap());
      scs.startOnAThread();

      ThreadTools.sleepForever();
   }

   private SimulationConstructionSet setupSCS(ValkyrieRobotModel robotModel, HumanoidFloatingRootJointRobot robot,
                                              YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      return setupSCS(robotModel, robot, robotModel.getSimulateDT(), yoGraphicsListRegistry);
   }

   private SimulationConstructionSet setupSCS(ValkyrieRobotModel robotModel, HumanoidFloatingRootJointRobot robot, double simulationDT,
                                              YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      FlatGroundEnvironment flatGroundEnvironment = new FlatGroundEnvironment();
      SimulationConstructionSet scs = new SimulationConstructionSet(robot, SimulationTestingParameters.createFromSystemProperties());

      SimulationOverheadPlotterFactory simulationOverheadPlotterFactory = scs.createSimulationOverheadPlotterFactory();
      simulationOverheadPlotterFactory.addYoGraphicsListRegistries(yoGraphicsListRegistry);
      simulationOverheadPlotterFactory.setShowOnStart(true);
      simulationOverheadPlotterFactory.createOverheadPlotter();

      scs.setGroundVisible(false);
      scs.addStaticLinkGraphics(flatGroundEnvironment.getTerrainObject3D().getLinkGraphics());
      scs.addYoGraphicsListRegistry(yoGraphicsListRegistry);

      DRCSCSInitialSetup initialSetup = new DRCSCSInitialSetup(flatGroundEnvironment, simulationDT);
      initialSetup.setDrawGroundProfile(false);
      initialSetup.setRecordFrequency((int) (robotModel.getControllerDT() / simulationDT));
      initialSetup.initializeSimulation(scs);
      initialSetup.initializeRobot(robot, robotModel, yoGraphicsListRegistry);
      return scs;
   }

   private YoGraphicsListRegistry setupController(ValkyrieRobotModel robotModel, FullHumanoidRobotModel fullRobotModel, HumanoidFloatingRootJointRobot robot)
   {
      return setupController(robotModel, fullRobotModel, robot, robotModel.getSimulateDT());
   }

   private YoGraphicsListRegistry setupController(ValkyrieRobotModel robotModel, FullHumanoidRobotModel fullRobotModel, HumanoidFloatingRootJointRobot robot,
                                                  double simulationDT)
   {
      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();
      List<? extends ContactablePlaneBody> contactablePlaneBodies = createContactablePlaneBodies(robotModel.getContactPointParameters(), fullRobotModel);
      MultiContactStaticController controller = new MultiContactStaticController(robotModel.getControllerDT(),
                                                                                 9.81,
                                                                                 fullRobotModel,
                                                                                 new ValkyrieMultiContactMomentumOptimizationSettings(robotModel.getJointMap()),
                                                                                 robotModel,
                                                                                 contactablePlaneBodies,
                                                                                 yoGraphicsListRegistry);

      DRCPerfectSensorReaderFactory sensorReaderFactory = new DRCPerfectSensorReaderFactory(robot, 0);
      sensorReaderFactory.build(fullRobotModel.getRootJoint(), fullRobotModel.getIMUDefinitions(), fullRobotModel.getForceSensorDefinitions(), null, null);
      DRCPerfectSensorReader sensorReader = sensorReaderFactory.getSensorReader();
      SimulatedLowLevelOutputWriter outputWriter = new SimulatedLowLevelOutputWriter(robot, true);
      outputWriter.setJointDesiredOutputList(controller.getJointDesiredOutputList());

      ModularRobotController modularRobotController = new ModularRobotController("MainController");
      modularRobotController.setRawSensorReader(toRawSensorReader(sensorReader));
      modularRobotController.setRawOutputWriter(toRawOutputWriter(outputWriter));
      modularRobotController.addRobotController(controller);
      robot.setController(new InitializingRobotController(modularRobotController), (int) (robotModel.getControllerDT() / simulationDT));

      return yoGraphicsListRegistry;
   }

   private RawSensorReader toRawSensorReader(DRCPerfectSensorReader sensorReader)
   {
      return new RawSensorReader()
      {
         @Override
         public void initialize()
         {
            read();
         }

         @Override
         public void read()
         {
            sensorReader.read(null);
         }

         @Override
         public YoVariableRegistry getYoVariableRegistry()
         {
            return sensorReader.getYoVariableRegistry();
         }
      };
   }

   private RawOutputWriter toRawOutputWriter(SimulatedLowLevelOutputWriter outputWriter)
   {
      return new RawOutputWriter()
      {
         @Override
         public void initialize()
         {
         }

         @Override
         public YoVariableRegistry getYoVariableRegistry()
         {
            return outputWriter.getYoVariableRegistry();
         }

         @Override
         public void write()
         {
            outputWriter.writeBefore(0);
         }
      };
   }

   private List<? extends ContactablePlaneBody> createContactablePlaneBodies(RobotContactPointParameters<RobotSide> contactPointParameters,
                                                                             FullHumanoidRobotModel fullRobotModel)
   {
      ContactableBodiesFactory<RobotSide> contactableBodiesFactory = new ContactableBodiesFactory<RobotSide>();
      ArrayList<String> additionalContactRigidBodyNames = contactPointParameters.getAdditionalContactRigidBodyNames();
      ArrayList<String> additionaContactNames = contactPointParameters.getAdditionalContactNames();
      ArrayList<RigidBodyTransform> additionalContactTransforms = contactPointParameters.getAdditionalContactTransforms();
      contactableBodiesFactory.setFullRobotModel(fullRobotModel);
      for (int i = 0; i < contactPointParameters.getAdditionalContactNames().size(); i++)
         contactableBodiesFactory.addAdditionalContactPoint(additionalContactRigidBodyNames.get(i),
                                                            additionaContactNames.get(i),
                                                            additionalContactTransforms.get(i));
      List<? extends ContactablePlaneBody> contactablePlaneBodies = contactableBodiesFactory.createAdditionalContactPoints();
      contactableBodiesFactory.disposeFactory();
      return contactablePlaneBodies;
   }

   private static void setRobotToCrawl1Configuration(HumanoidFloatingRootJointRobot robot, DRCRobotJointMap jointMap)
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(robotSide, LegJointName.HIP_YAW)).setQ(robotSide.negateIfRightSide(0.5));
         robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(robotSide, LegJointName.HIP_PITCH)).setQ(-1.0);
         robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(robotSide, LegJointName.KNEE_PITCH)).setQ(1.2);
         robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(robotSide, LegJointName.ANKLE_PITCH)).setQ(0.2);

         robot.getOneDegreeOfFreedomJoint(jointMap.getArmJointName(robotSide, ArmJointName.SHOULDER_PITCH)).setQ(-2.0);
         robot.getOneDegreeOfFreedomJoint(jointMap.getArmJointName(robotSide, ArmJointName.SHOULDER_ROLL)).setQ(robotSide.negateIfLeftSide(1.2));
         robot.getOneDegreeOfFreedomJoint(jointMap.getArmJointName(robotSide, ArmJointName.ELBOW_PITCH)).setQ(robotSide.negateIfLeftSide(1.2));
      }

      RigidBodyTransform rootJointPose = new RigidBodyTransform();
      rootJointPose.setRotationPitch(0.5 * Math.PI);
      rootJointPose.setTranslation(0.0, 0.0, 0.39);
      robot.getRootJoint().setRotationAndTranslation(rootJointPose);
   }

   private static void setRobotToCrawl2Configuration(HumanoidFloatingRootJointRobot robot, DRCRobotJointMap jointMap)
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(robotSide, LegJointName.HIP_YAW)).setQ(robotSide.negateIfRightSide(0.0));
         robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(robotSide, LegJointName.HIP_PITCH)).setQ(-1.25);
         robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(robotSide, LegJointName.KNEE_PITCH)).setQ(1.75);
         robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(robotSide, LegJointName.ANKLE_PITCH)).setQ(-0.8);

         robot.getOneDegreeOfFreedomJoint(jointMap.getArmJointName(robotSide, ArmJointName.SHOULDER_PITCH)).setQ(-1.5);
         robot.getOneDegreeOfFreedomJoint(jointMap.getArmJointName(robotSide, ArmJointName.SHOULDER_ROLL)).setQ(robotSide.negateIfLeftSide(0.65));
         robot.getOneDegreeOfFreedomJoint(jointMap.getArmJointName(robotSide, ArmJointName.SHOULDER_YAW)).setQ(1.5);
         robot.getOneDegreeOfFreedomJoint(jointMap.getArmJointName(robotSide, ArmJointName.ELBOW_PITCH)).setQ(robotSide.negateIfLeftSide(1.4));
      }

      robot.getOneDegreeOfFreedomJoint(jointMap.getSpineJointName(SpineJointName.SPINE_PITCH)).setQ(0.1);

      RigidBodyTransform rootJointPose = new RigidBodyTransform();
      rootJointPose.setRotation(new Quaternion(0.0, 0.4, 0.0, 0.5));
      rootJointPose.setTranslation(0.0, 0.0, 0.56);
      robot.getRootJoint().setRotationAndTranslation(rootJointPose);
   }

   private static void setRobotToKneel1Configuration(HumanoidFloatingRootJointRobot robot, DRCRobotJointMap jointMap)
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(robotSide, LegJointName.HIP_YAW)).setQ(robotSide.negateIfRightSide(0.0));
         robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(robotSide, LegJointName.HIP_PITCH)).setQ(-0.3);
         robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(robotSide, LegJointName.KNEE_PITCH)).setQ(1.9);
         robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(robotSide, LegJointName.ANKLE_PITCH)).setQ(0.8);

         robot.getOneDegreeOfFreedomJoint(jointMap.getArmJointName(robotSide, ArmJointName.SHOULDER_PITCH)).setQ(0.7);
         robot.getOneDegreeOfFreedomJoint(jointMap.getArmJointName(robotSide, ArmJointName.SHOULDER_ROLL)).setQ(robotSide.negateIfLeftSide(1.2));
         robot.getOneDegreeOfFreedomJoint(jointMap.getArmJointName(robotSide, ArmJointName.ELBOW_PITCH)).setQ(robotSide.negateIfLeftSide(1.7));
      }

      RigidBodyTransform rootJointPose = new RigidBodyTransform();
      rootJointPose.setTranslation(0.0, 0.0, 0.74);
      robot.getRootJoint().setRotationAndTranslation(rootJointPose);
   }

   private static void setRobotToKneel2Configuration(HumanoidFloatingRootJointRobot robot, DRCRobotJointMap jointMap)
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(robotSide, LegJointName.HIP_YAW)).setQ(robotSide.negateIfRightSide(0.0));
         robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(robotSide, LegJointName.HIP_PITCH)).setQ(0.75);
         robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(robotSide, LegJointName.KNEE_PITCH)).setQ(2.05);
         robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(robotSide, LegJointName.ANKLE_PITCH)).setQ(-0.8);

         robot.getOneDegreeOfFreedomJoint(jointMap.getArmJointName(robotSide, ArmJointName.SHOULDER_PITCH)).setQ(0.7);
         robot.getOneDegreeOfFreedomJoint(jointMap.getArmJointName(robotSide, ArmJointName.SHOULDER_ROLL)).setQ(robotSide.negateIfLeftSide(1.2));
         robot.getOneDegreeOfFreedomJoint(jointMap.getArmJointName(robotSide, ArmJointName.ELBOW_PITCH)).setQ(robotSide.negateIfLeftSide(1.7));
      }

      robot.getOneDegreeOfFreedomJoint(jointMap.getSpineJointName(SpineJointName.SPINE_PITCH)).setQ(0.4);

      RigidBodyTransform rootJointPose = new RigidBodyTransform();
      rootJointPose.setRotation(new Quaternion(0.0, -0.4, 0.0, 0.8));
      rootJointPose.setTranslation(0.0, 0.0, 0.64);
      robot.getRootJoint().setRotationAndTranslation(rootJointPose);
   }

   private static void addUpperKneeContactPoints(ValkyrieJointMap jointMap, ValkyrieMultiContactPointParameters contactPointParameters)
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         Point3D position = new Point3D(0.06, robotSide.negateIfRightSide(0.07), -0.46);
         RigidBodyTransform rigidBodyTransform = new RigidBodyTransform();
         rigidBodyTransform.setTranslation(position);
         String parentJointName = jointMap.getLegJointName(robotSide, LegJointName.HIP_PITCH);
         String bodyName = parentJointName + "Link";
         String contactName = robotSide.getCamelCaseName() + "KneeCP";
         contactPointParameters.addSingleContactPoint(parentJointName, bodyName, contactName, rigidBodyTransform);
      }
   }

   private static void addLowerKneeContactPoints(ValkyrieJointMap jointMap, ValkyrieMultiContactPointParameters contactPointParameters)
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         Point3D position = new Point3D(0.075, robotSide.negateIfRightSide(0.07), -0.01);
         RigidBodyTransform rigidBodyTransform = new RigidBodyTransform();
         rigidBodyTransform.setTranslation(position);
         String parentJointName = jointMap.getLegJointName(robotSide, LegJointName.KNEE_PITCH);
         String bodyName = parentJointName + "Link";
         String contactName = robotSide.getCamelCaseName() + "KneeCP";
         contactPointParameters.addSingleContactPoint(parentJointName, bodyName, contactName, rigidBodyTransform);
      }
   }

   private static void addToeFrontContactPoints(ValkyrieJointMap jointMap, ValkyrieMultiContactPointParameters contactPointParameters)
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         Point3D position = new Point3D(0.19, 0.0, -0.05);
         RigidBodyTransform rigidBodyTransform = new RigidBodyTransform();
         rigidBodyTransform.setTranslation(position);
         String parentJointName = jointMap.getJointBeforeFootName(robotSide);
         String bodyName = jointMap.getFootName(robotSide);
         String contactName = robotSide.getCamelCaseName() + "ToeCP";
         contactPointParameters.addSingleContactPoint(parentJointName, bodyName, contactName, rigidBodyTransform);
      }
   }

   private static void addToeTopContactPoints(ValkyrieJointMap jointMap, ValkyrieMultiContactPointParameters contactPointParameters)
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         Point3D position = new Point3D(0.18, 0.0, -0.035);
         RigidBodyTransform rigidBodyTransform = new RigidBodyTransform();
         rigidBodyTransform.setTranslation(position);
         String parentJointName = jointMap.getJointBeforeFootName(robotSide);
         String bodyName = jointMap.getFootName(robotSide);
         String contactName = robotSide.getCamelCaseName() + "ToeCP";
         contactPointParameters.addSingleContactPoint(parentJointName, bodyName, contactName, rigidBodyTransform);
      }
   }

   private static void addElbowContactPoints(ValkyrieJointMap jointMap, ValkyrieMultiContactPointParameters contactPointParameters)
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         Point3D position = new Point3D(-0.11, robotSide.negateIfRightSide(0.05), 0.0);
         RigidBodyTransform rigidBodyTransform = new RigidBodyTransform();
         rigidBodyTransform.setTranslation(position);
         String parentJointName = jointMap.getArmJointName(robotSide, ArmJointName.ELBOW_PITCH);
         String bodyName = parentJointName + "Link";
         String contactName = robotSide.getCamelCaseName() + "ElbowCP";
         contactPointParameters.addSingleContactPoint(parentJointName, bodyName, contactName, rigidBodyTransform);
      }
   }

   private static void addHandFist1ContactPoints(ValkyrieJointMap jointMap, ValkyrieMultiContactPointParameters contactPointParameters)
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         Point3D position = new Point3D(-0.025, robotSide.negateIfRightSide(0.10), -0.01);
         RigidBodyTransform rigidBodyTransform = new RigidBodyTransform();
         rigidBodyTransform.setTranslation(position);
         String parentJointName = jointMap.getJointBeforeHandName(robotSide);
         String bodyName = jointMap.getHandName(robotSide);
         String contactName = robotSide.getCamelCaseName() + "FistCP";
         contactPointParameters.addSingleContactPoint(parentJointName, bodyName, contactName, rigidBodyTransform);
      }
   }
}
