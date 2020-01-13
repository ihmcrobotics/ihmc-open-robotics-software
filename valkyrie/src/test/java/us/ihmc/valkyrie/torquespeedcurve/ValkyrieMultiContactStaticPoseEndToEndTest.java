package us.ihmc.valkyrie.torquespeedcurve;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.function.Function;
import java.util.stream.Collectors;

import org.junit.jupiter.api.Test;

import us.ihmc.avatar.SimulatedLowLevelOutputWriter;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.initialSetup.DRCSCSInitialSetup;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ListOfPointsContactablePlaneBody;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.SimpleContactPointPlaneBody;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
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
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class ValkyrieMultiContactStaticPoseEndToEndTest
{
   @Test
   public void testCrawl1()
   {
      ValkyrieRobotModel robotModel = new ValkyrieRobotModel(RobotTarget.SCS, ValkyrieRobotVersion.FINGERLESS);
      ValkyrieMultiContactPointParameters contactPointParameters = new ValkyrieMultiContactPointParameters(robotModel.getJointMap(),
                                                                                                           robotModel.getRobotPhysicalProperties());
      addKneeUpperContactPoints(robotModel.getJointMap(), contactPointParameters);
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
      addKneeLowerContactPoints(robotModel.getJointMap(), contactPointParameters);
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
      addKneeLowerContactPoints(robotModel.getJointMap(), contactPointParameters);
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
      addKneeLowerContactPoints(robotModel.getJointMap(), contactPointParameters);
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

   @Test
   public void testSit1()
   {
      ValkyrieRobotModel robotModel = new ValkyrieRobotModel(RobotTarget.SCS, ValkyrieRobotVersion.FINGERLESS);
      ValkyrieMultiContactPointParameters contactPointParameters = new ValkyrieMultiContactPointParameters(robotModel.getJointMap(),
                                                                                                           robotModel.getRobotPhysicalProperties());
      addButt1ContactPoint(robotModel.getJointMap(), contactPointParameters);
      addHeelBottomContactPoints(robotModel.getJointMap(), contactPointParameters);
      //      addToeBottomContactPoints(robotModel.getJointMap(), contactPointParameters);
      addHandFist1ContactPoints(robotModel.getJointMap(), contactPointParameters);
      robotModel.setContactPointParameters(contactPointParameters);

      FullHumanoidRobotModel fullRobotModel = robotModel.createFullRobotModel();
      HumanoidFloatingRootJointRobot robot = robotModel.createHumanoidFloatingRootJointRobot(false);

      double simulationDT = 2.0e-4;
      YoGraphicsListRegistry yoGraphicsListRegistry = setupController(robotModel, fullRobotModel, robot, simulationDT);
      SimulationConstructionSet scs = setupSCS(robotModel, robot, simulationDT, yoGraphicsListRegistry);

      setRobotToSit1Configuration(robot, robotModel.getJointMap());
      scs.startOnAThread();

      ThreadTools.sleepForever();
   }

   @Test
   public void testSit2()
   {
      ValkyrieRobotModel robotModel = new ValkyrieRobotModel(RobotTarget.SCS, ValkyrieRobotVersion.FINGERLESS);
      ValkyrieMultiContactPointParameters contactPointParameters = new ValkyrieMultiContactPointParameters(robotModel.getJointMap(),
                                                                                                           robotModel.getRobotPhysicalProperties());
      addButt2ContactPoint(robotModel.getJointMap(), contactPointParameters);
      addHeelBottomContactPoints(robotModel.getJointMap(), contactPointParameters);
      addHandFist1ContactPoints(robotModel.getJointMap(), contactPointParameters);
      robotModel.setContactPointParameters(contactPointParameters);

      FullHumanoidRobotModel fullRobotModel = robotModel.createFullRobotModel();
      HumanoidFloatingRootJointRobot robot = robotModel.createHumanoidFloatingRootJointRobot(false);

      double simulationDT = 2.0e-4;
      YoGraphicsListRegistry yoGraphicsListRegistry = setupController(robotModel, fullRobotModel, robot, simulationDT);
      SimulationConstructionSet scs = setupSCS(robotModel, robot, simulationDT, yoGraphicsListRegistry);

      setRobotToSit2Configuration(robot, robotModel.getJointMap());
      scs.startOnAThread();

      ThreadTools.sleepForever();
   }

   @Test
   public void testSit3()
   {
      ValkyrieRobotModel robotModel = new ValkyrieRobotModel(RobotTarget.SCS, ValkyrieRobotVersion.FINGERLESS);
      ValkyrieMultiContactPointParameters contactPointParameters = new ValkyrieMultiContactPointParameters(robotModel.getJointMap(),
                                                                                                           robotModel.getRobotPhysicalProperties());
      addButt2ContactPoint(robotModel.getJointMap(), contactPointParameters);
      addHeelBottomContactPoints(robotModel.getJointMap(), contactPointParameters);
      addHandFist1ContactPoints(robotModel.getJointMap(), contactPointParameters);
      robotModel.setContactPointParameters(contactPointParameters);

      FullHumanoidRobotModel fullRobotModel = robotModel.createFullRobotModel();
      HumanoidFloatingRootJointRobot robot = robotModel.createHumanoidFloatingRootJointRobot(false);

      double simulationDT = 2.0e-4;
      YoGraphicsListRegistry yoGraphicsListRegistry = setupController(robotModel, fullRobotModel, robot, simulationDT);
      SimulationConstructionSet scs = setupSCS(robotModel, robot, simulationDT, yoGraphicsListRegistry);

      setRobotToSit3Configuration(robot, robotModel.getJointMap());
      scs.startOnAThread();

      ThreadTools.sleepForever();
   }

   @Test
   public void testRollingA1()
   {
      ValkyrieRobotModel robotModel = new ValkyrieRobotModel(RobotTarget.SCS, ValkyrieRobotVersion.FINGERLESS);
      ValkyrieJointMap jointMap = robotModel.getJointMap();
      ValkyrieMultiContactPointParameters contactPointParameters = new ValkyrieMultiContactPointParameters(jointMap, robotModel.getRobotPhysicalProperties());
      addKneeOuterContactPoint(RobotSide.RIGHT, jointMap, contactPointParameters);
      addToeOuterContactPoint(RobotSide.RIGHT, jointMap, contactPointParameters);
      addElbowContactPoint(RobotSide.RIGHT, jointMap, contactPointParameters);
      addHandFist1ContactPoint(RobotSide.LEFT, jointMap, contactPointParameters);
      addHeelBottomContactPoint(RobotSide.LEFT, jointMap, contactPointParameters);
      robotModel.setContactPointParameters(contactPointParameters);

      FullHumanoidRobotModel fullRobotModel = robotModel.createFullRobotModel();
      HumanoidFloatingRootJointRobot robot = robotModel.createHumanoidFloatingRootJointRobot(false);

      double simulationDT = 2.0e-4;
      YoGraphicsListRegistry yoGraphicsListRegistry = setupController(robotModel, fullRobotModel, robot, simulationDT);
      SimulationConstructionSet scs = setupSCS(robotModel, robot, simulationDT, yoGraphicsListRegistry);

      setRobotToRollingA1Configuration(robot, jointMap);
      scs.startOnAThread();

      ThreadTools.sleepForever();
   }

   @Test
   public void testRollingA2()
   {
      ValkyrieRobotModel robotModel = new ValkyrieRobotModel(RobotTarget.SCS, ValkyrieRobotVersion.FINGERLESS);
      ValkyrieJointMap jointMap = robotModel.getJointMap();
      ValkyrieMultiContactPointParameters contactPointParameters = new ValkyrieMultiContactPointParameters(jointMap, robotModel.getRobotPhysicalProperties());
      addKneeOuterContactPoint(RobotSide.RIGHT, jointMap, contactPointParameters);
      addToeOuterContactPoint(RobotSide.RIGHT, jointMap, contactPointParameters);
      addElbowContactPoint(RobotSide.RIGHT, jointMap, contactPointParameters);
      addHandFist1ContactPoint(RobotSide.LEFT, jointMap, contactPointParameters);
      addHeelBottomContactPoint(RobotSide.LEFT, jointMap, contactPointParameters);
      robotModel.setContactPointParameters(contactPointParameters);

      FullHumanoidRobotModel fullRobotModel = robotModel.createFullRobotModel();
      HumanoidFloatingRootJointRobot robot = robotModel.createHumanoidFloatingRootJointRobot(false);

      double simulationDT = 2.0e-4;
      YoGraphicsListRegistry yoGraphicsListRegistry = setupController(robotModel, fullRobotModel, robot, simulationDT);
      SimulationConstructionSet scs = setupSCS(robotModel, robot, simulationDT, yoGraphicsListRegistry);

      setRobotToRollingA2Configuration(robot, jointMap);
      scs.startOnAThread();

      ThreadTools.sleepForever();
   }

   @Test
   public void testRollingA3()
   {
      ValkyrieRobotModel robotModel = new ValkyrieRobotModel(RobotTarget.SCS, ValkyrieRobotVersion.FINGERLESS);
      ValkyrieJointMap jointMap = robotModel.getJointMap();
      ValkyrieMultiContactPointParameters contactPointParameters = new ValkyrieMultiContactPointParameters(jointMap, robotModel.getRobotPhysicalProperties());
      addKneeOuterContactPoint(RobotSide.RIGHT, jointMap, contactPointParameters);
      addToeOuterContactPoint(RobotSide.RIGHT, jointMap, contactPointParameters);
      addElbowContactPoint(RobotSide.RIGHT, jointMap, contactPointParameters);
      addHandFist1ContactPoint(RobotSide.LEFT, jointMap, contactPointParameters);
      addHeelBottomContactPoint(RobotSide.LEFT, jointMap, contactPointParameters);
      robotModel.setContactPointParameters(contactPointParameters);

      FullHumanoidRobotModel fullRobotModel = robotModel.createFullRobotModel();
      HumanoidFloatingRootJointRobot robot = robotModel.createHumanoidFloatingRootJointRobot(false);

      double simulationDT = 2.0e-4;
      YoGraphicsListRegistry yoGraphicsListRegistry = setupController(robotModel, fullRobotModel, robot, simulationDT);
      SimulationConstructionSet scs = setupSCS(robotModel, robot, simulationDT, yoGraphicsListRegistry);

      setRobotToRollingA3Configuration(robot, jointMap);
      scs.startOnAThread();

      ThreadTools.sleepForever();
   }

   @Test
   public void testRollingA4()
   {
      ValkyrieRobotModel robotModel = new ValkyrieRobotModel(RobotTarget.SCS, ValkyrieRobotVersion.FINGERLESS);
      ValkyrieJointMap jointMap = robotModel.getJointMap();
      ValkyrieMultiContactPointParameters contactPointParameters = new ValkyrieMultiContactPointParameters(jointMap, robotModel.getRobotPhysicalProperties());
      addKneeOuterContactPoint(RobotSide.RIGHT, jointMap, contactPointParameters);
      addToeOuterContactPoint(RobotSide.RIGHT, jointMap, contactPointParameters);
      addElbowContactPoint(RobotSide.RIGHT, jointMap, contactPointParameters);
      addHeelBottomContactPoint(RobotSide.LEFT, jointMap, contactPointParameters);
      robotModel.setContactPointParameters(contactPointParameters);

      FullHumanoidRobotModel fullRobotModel = robotModel.createFullRobotModel();
      HumanoidFloatingRootJointRobot robot = robotModel.createHumanoidFloatingRootJointRobot(false);

      double simulationDT = 2.0e-4;
      YoGraphicsListRegistry yoGraphicsListRegistry = setupController(robotModel, fullRobotModel, robot, simulationDT);
      SimulationConstructionSet scs = setupSCS(robotModel, robot, simulationDT, yoGraphicsListRegistry);

      setRobotToRollingA4Configuration(robot, jointMap);
      scs.startOnAThread();

      ThreadTools.sleepForever();
   }

   @Test
   public void testRollingA5()
   {
      ValkyrieRobotModel robotModel = new ValkyrieRobotModel(RobotTarget.SCS, ValkyrieRobotVersion.FINGERLESS);
      ValkyrieJointMap jointMap = robotModel.getJointMap();
      ValkyrieMultiContactPointParameters contactPointParameters = new ValkyrieMultiContactPointParameters(jointMap, robotModel.getRobotPhysicalProperties());
      addKneeOuterContactPoint(RobotSide.RIGHT, jointMap, contactPointParameters);
      addToeOuterContactPoint(RobotSide.RIGHT, jointMap, contactPointParameters);
      addElbowContactPoint(RobotSide.RIGHT, jointMap, contactPointParameters);
      addHeelInnerBottomContactPoint(RobotSide.LEFT, jointMap, contactPointParameters);
      robotModel.setContactPointParameters(contactPointParameters);

      FullHumanoidRobotModel fullRobotModel = robotModel.createFullRobotModel();
      HumanoidFloatingRootJointRobot robot = robotModel.createHumanoidFloatingRootJointRobot(false);

      double simulationDT = 2.0e-4;
      YoGraphicsListRegistry yoGraphicsListRegistry = setupController(robotModel, fullRobotModel, robot, simulationDT);
      SimulationConstructionSet scs = setupSCS(robotModel, robot, simulationDT, yoGraphicsListRegistry);

      setRobotToRollingA5Configuration(robot, jointMap);
      scs.startOnAThread();

      ThreadTools.sleepForever();
   }

   @Test
   public void testRollingA6()
   {
      ValkyrieRobotModel robotModel = new ValkyrieRobotModel(RobotTarget.SCS, ValkyrieRobotVersion.FINGERLESS);
      ValkyrieJointMap jointMap = robotModel.getJointMap();
      ValkyrieMultiContactPointParameters contactPointParameters = new ValkyrieMultiContactPointParameters(jointMap, robotModel.getRobotPhysicalProperties());
      addKneeOuterContactPoint(RobotSide.RIGHT, jointMap, contactPointParameters);
      addToeOuterContactPoint(RobotSide.RIGHT, jointMap, contactPointParameters);
      addElbowContactPoint(RobotSide.RIGHT, jointMap, contactPointParameters);
      robotModel.setContactPointParameters(contactPointParameters);

      FullHumanoidRobotModel fullRobotModel = robotModel.createFullRobotModel();
      HumanoidFloatingRootJointRobot robot = robotModel.createHumanoidFloatingRootJointRobot(false);

      double simulationDT = 2.0e-4;
      YoGraphicsListRegistry yoGraphicsListRegistry = setupController(robotModel, fullRobotModel, robot, simulationDT);
      SimulationConstructionSet scs = setupSCS(robotModel, robot, simulationDT, yoGraphicsListRegistry);

      setRobotToRollingA6Configuration(robot, jointMap);
      scs.startOnAThread();

      ThreadTools.sleepForever();
   }

   @Test
   public void testRollingA7()
   {
      ValkyrieRobotModel robotModel = new ValkyrieRobotModel(RobotTarget.SCS, ValkyrieRobotVersion.FINGERLESS);
      ValkyrieJointMap jointMap = robotModel.getJointMap();
      ValkyrieMultiContactPointParameters contactPointParameters = new ValkyrieMultiContactPointParameters(jointMap, robotModel.getRobotPhysicalProperties());
      addKneeOuterContactPoint(RobotSide.RIGHT, jointMap, contactPointParameters);
      addToeOuterContactPoint(RobotSide.RIGHT, jointMap, contactPointParameters);
      addElbowContactPoint(RobotSide.RIGHT, jointMap, contactPointParameters);
      addToeInnerBottomContactPoint(RobotSide.LEFT, jointMap, contactPointParameters);
      robotModel.setContactPointParameters(contactPointParameters);

      FullHumanoidRobotModel fullRobotModel = robotModel.createFullRobotModel();
      HumanoidFloatingRootJointRobot robot = robotModel.createHumanoidFloatingRootJointRobot(false);

      double simulationDT = 2.0e-4;
      YoGraphicsListRegistry yoGraphicsListRegistry = setupController(robotModel, fullRobotModel, robot, simulationDT);
      SimulationConstructionSet scs = setupSCS(robotModel, robot, simulationDT, yoGraphicsListRegistry);

      setRobotToRollingA7Configuration(robot, jointMap);
      scs.startOnAThread();

      ThreadTools.sleepForever();
   }

   @Test
   public void testLieDown1()
   {
      ValkyrieRobotModel robotModel = new ValkyrieRobotModel(RobotTarget.SCS, ValkyrieRobotVersion.FINGERLESS);
      ValkyrieJointMap jointMap = robotModel.getJointMap();
      ValkyrieMultiContactPointParameters contactPointParameters = new ValkyrieMultiContactPointParameters(jointMap, robotModel.getRobotPhysicalProperties());
      addHeelBottomContactPoints(jointMap, contactPointParameters);
      addBackpackFourContactPoints(jointMap, contactPointParameters);
      robotModel.setContactPointParameters(contactPointParameters);

      FullHumanoidRobotModel fullRobotModel = robotModel.createFullRobotModel();
      HumanoidFloatingRootJointRobot robot = robotModel.createHumanoidFloatingRootJointRobot(false);

      double simulationDT = 2.0e-4;
      YoGraphicsListRegistry yoGraphicsListRegistry = setupController(robotModel, fullRobotModel, robot, simulationDT);
      SimulationConstructionSet scs = setupSCS(robotModel, robot, simulationDT, yoGraphicsListRegistry);

      setRobotToLieDown1Configuration(robot, jointMap);
      scs.startOnAThread();

      ThreadTools.sleepForever();
   }

   @Test
   public void testLieDown2()
   {
      ValkyrieRobotModel robotModel = new ValkyrieRobotModel(RobotTarget.SCS, ValkyrieRobotVersion.FINGERLESS);
      ValkyrieJointMap jointMap = robotModel.getJointMap();
      ValkyrieMultiContactPointParameters contactPointParameters = new ValkyrieMultiContactPointParameters(jointMap, robotModel.getRobotPhysicalProperties());
      addHeelBottomContactPoints(jointMap, contactPointParameters);
      addBackpackFourContactPoints(jointMap, contactPointParameters);
      addHandFist1ContactPoint(RobotSide.LEFT, jointMap, contactPointParameters);
      robotModel.setContactPointParameters(contactPointParameters);

      FullHumanoidRobotModel fullRobotModel = robotModel.createFullRobotModel();
      HumanoidFloatingRootJointRobot robot = robotModel.createHumanoidFloatingRootJointRobot(false);

      double simulationDT = 2.0e-4;
      YoGraphicsListRegistry yoGraphicsListRegistry = setupController(robotModel, fullRobotModel, robot, simulationDT);
      SimulationConstructionSet scs = setupSCS(robotModel, robot, simulationDT, yoGraphicsListRegistry);

      setRobotToLieDown2Configuration(robot, jointMap);
      scs.startOnAThread();

      ThreadTools.sleepForever();
   }

   @Test
   public void testRollingB1()
   {
      ValkyrieRobotModel robotModel = new ValkyrieRobotModel(RobotTarget.SCS, ValkyrieRobotVersion.FINGERLESS);
      ValkyrieJointMap jointMap = robotModel.getJointMap();
      ValkyrieMultiContactPointParameters contactPointParameters = new ValkyrieMultiContactPointParameters(jointMap, robotModel.getRobotPhysicalProperties());
      addHandFist1ContactPoint(RobotSide.LEFT, jointMap, contactPointParameters);
      addHeelBottomContactPoint(RobotSide.LEFT, jointMap, contactPointParameters);
      addFootOuterEdgeContactPoints(RobotSide.RIGHT, jointMap, contactPointParameters);
      addBackpackSideEdgeContactPoints(RobotSide.RIGHT, jointMap, contactPointParameters);
      robotModel.setContactPointParameters(contactPointParameters);

      FullHumanoidRobotModel fullRobotModel = robotModel.createFullRobotModel();
      HumanoidFloatingRootJointRobot robot = robotModel.createHumanoidFloatingRootJointRobot(false);

      double simulationDT = 2.0e-4;
      YoGraphicsListRegistry yoGraphicsListRegistry = setupController(robotModel, fullRobotModel, robot, simulationDT);
      SimulationConstructionSet scs = setupSCS(robotModel, robot, simulationDT, yoGraphicsListRegistry);

      setRobotToRollingB1Configuration(robot, jointMap);
      scs.startOnAThread();

      ThreadTools.sleepForever();
   }

   @Test
   public void testRollingB2()
   {
      ValkyrieRobotModel robotModel = new ValkyrieRobotModel(RobotTarget.SCS, ValkyrieRobotVersion.FINGERLESS);
      ValkyrieJointMap jointMap = robotModel.getJointMap();
      ValkyrieMultiContactPointParameters contactPointParameters = new ValkyrieMultiContactPointParameters(jointMap, robotModel.getRobotPhysicalProperties());
      addHandFist1ContactPoint(RobotSide.LEFT, jointMap, contactPointParameters);
      addHeelBottomContactPoint(RobotSide.LEFT, jointMap, contactPointParameters);
      addFootOuterEdgeContactPoints(RobotSide.RIGHT, jointMap, contactPointParameters);
      addBackpackSideEdgeContactPoints(RobotSide.RIGHT, jointMap, contactPointParameters);
      addShoulderPitchBackContactPoint(RobotSide.RIGHT, jointMap, contactPointParameters);
      robotModel.setContactPointParameters(contactPointParameters);

      FullHumanoidRobotModel fullRobotModel = robotModel.createFullRobotModel();
      HumanoidFloatingRootJointRobot robot = robotModel.createHumanoidFloatingRootJointRobot(false);

      double simulationDT = 2.0e-4;
      YoGraphicsListRegistry yoGraphicsListRegistry = setupController(robotModel, fullRobotModel, robot, simulationDT);
      SimulationConstructionSet scs = setupSCS(robotModel, robot, simulationDT, yoGraphicsListRegistry);

      setRobotToRollingB2Configuration(robot, jointMap);
      scs.startOnAThread();

      ThreadTools.sleepForever();
   }

   @Test
   public void testRollingB3()
   {
      ValkyrieRobotModel robotModel = new ValkyrieRobotModel(RobotTarget.SCS, ValkyrieRobotVersion.FINGERLESS);
      ValkyrieJointMap jointMap = robotModel.getJointMap();
      ValkyrieMultiContactPointParameters contactPointParameters = new ValkyrieMultiContactPointParameters(jointMap, robotModel.getRobotPhysicalProperties());
      addHandFist1ContactPoint(RobotSide.LEFT, jointMap, contactPointParameters);
      addFootInnerEdgeContactPoints(RobotSide.LEFT, jointMap, contactPointParameters);
      addKneeOuterContactPoint(RobotSide.RIGHT, jointMap, contactPointParameters);
      addToeOuterContactPoint(RobotSide.RIGHT, jointMap, contactPointParameters);
      addShoulderPitchBackContactPoint(RobotSide.RIGHT, jointMap, contactPointParameters);
      robotModel.setContactPointParameters(contactPointParameters);

      FullHumanoidRobotModel fullRobotModel = robotModel.createFullRobotModel();
      HumanoidFloatingRootJointRobot robot = robotModel.createHumanoidFloatingRootJointRobot(false);

      double simulationDT = 2.0e-4;
      YoGraphicsListRegistry yoGraphicsListRegistry = setupController(robotModel, fullRobotModel, robot, simulationDT);
      SimulationConstructionSet scs = setupSCS(robotModel, robot, simulationDT, yoGraphicsListRegistry);

      setRobotToRollingB3Configuration(robot, jointMap);
      scs.startOnAThread();

      ThreadTools.sleepForever();
   }

   @Test
   public void testRollingB4()
   {
      ValkyrieRobotModel robotModel = new ValkyrieRobotModel(RobotTarget.SCS, ValkyrieRobotVersion.FINGERLESS);
      ValkyrieJointMap jointMap = robotModel.getJointMap();
      ValkyrieMultiContactPointParameters contactPointParameters = new ValkyrieMultiContactPointParameters(jointMap, robotModel.getRobotPhysicalProperties());
      addFootInnerEdgeContactPoints(RobotSide.LEFT, jointMap, contactPointParameters);
      addKneeOuterContactPoint(RobotSide.RIGHT, jointMap, contactPointParameters);
      addToeOuterContactPoint(RobotSide.RIGHT, jointMap, contactPointParameters);
      addShoulderPitchBackContactPoint(RobotSide.RIGHT, jointMap, contactPointParameters);
      robotModel.setContactPointParameters(contactPointParameters);

      FullHumanoidRobotModel fullRobotModel = robotModel.createFullRobotModel();
      HumanoidFloatingRootJointRobot robot = robotModel.createHumanoidFloatingRootJointRobot(false);

      double simulationDT = 2.0e-4;
      YoGraphicsListRegistry yoGraphicsListRegistry = setupController(robotModel, fullRobotModel, robot, simulationDT);
      SimulationConstructionSet scs = setupSCS(robotModel, robot, simulationDT, yoGraphicsListRegistry);

      setRobotToRollingB4Configuration(robot, jointMap);
      scs.startOnAThread();

      ThreadTools.sleepForever();
   }

   @Test
   public void testRollingB5()
   {
      ValkyrieRobotModel robotModel = new ValkyrieRobotModel(RobotTarget.SCS, ValkyrieRobotVersion.FINGERLESS);
      ValkyrieJointMap jointMap = robotModel.getJointMap();
      ValkyrieMultiContactPointParameters contactPointParameters = new ValkyrieMultiContactPointParameters(jointMap, robotModel.getRobotPhysicalProperties());
      addFootInnerEdgeContactPoints(RobotSide.LEFT, jointMap, contactPointParameters);
      addKneeOuterContactPoint(RobotSide.RIGHT, jointMap, contactPointParameters);
      addToeOuterContactPoint(RobotSide.RIGHT, jointMap, contactPointParameters);
      addShoulderPitchOuterContactPoint(RobotSide.RIGHT, jointMap, contactPointParameters);
      robotModel.setContactPointParameters(contactPointParameters);

      FullHumanoidRobotModel fullRobotModel = robotModel.createFullRobotModel();
      HumanoidFloatingRootJointRobot robot = robotModel.createHumanoidFloatingRootJointRobot(false);

      double simulationDT = 2.0e-4;
      YoGraphicsListRegistry yoGraphicsListRegistry = setupController(robotModel, fullRobotModel, robot, simulationDT);
      SimulationConstructionSet scs = setupSCS(robotModel, robot, simulationDT, yoGraphicsListRegistry);

      setRobotToRollingB5Configuration(robot, jointMap);
      scs.startOnAThread();

      ThreadTools.sleepForever();
   }

   @Test
   public void testRollingB6()
   {
      ValkyrieRobotModel robotModel = new ValkyrieRobotModel(RobotTarget.SCS, ValkyrieRobotVersion.FINGERLESS);
      ValkyrieJointMap jointMap = robotModel.getJointMap();
      ValkyrieMultiContactPointParameters contactPointParameters = new ValkyrieMultiContactPointParameters(jointMap, robotModel.getRobotPhysicalProperties());
      addHandFist1ContactPoint(RobotSide.LEFT, jointMap, contactPointParameters);
      addFootInnerEdgeContactPoints(RobotSide.LEFT, jointMap, contactPointParameters);
      addKneeOuterContactPoint(RobotSide.RIGHT, jointMap, contactPointParameters);
      addToeOuterContactPoint(RobotSide.RIGHT, jointMap, contactPointParameters);
      addShoulderPitchOuterContactPoint(RobotSide.RIGHT, jointMap, contactPointParameters);
      robotModel.setContactPointParameters(contactPointParameters);

      FullHumanoidRobotModel fullRobotModel = robotModel.createFullRobotModel();
      HumanoidFloatingRootJointRobot robot = robotModel.createHumanoidFloatingRootJointRobot(false);

      double simulationDT = 2.0e-4;
      YoGraphicsListRegistry yoGraphicsListRegistry = setupController(robotModel, fullRobotModel, robot, simulationDT);
      SimulationConstructionSet scs = setupSCS(robotModel, robot, simulationDT, yoGraphicsListRegistry);

      setRobotToRollingB6Configuration(robot, jointMap);
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
      List<? extends ContactablePlaneBody> contactablePlaneBodies = createContactablePlaneBodies((ValkyrieMultiContactPointParameters) robotModel.getContactPointParameters(),
                                                                                                 fullRobotModel);
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

   private List<? extends ContactablePlaneBody> createContactablePlaneBodies(ValkyrieMultiContactPointParameters contactPointParameters,
                                                                             FullHumanoidRobotModel fullRobotModel)
   {
      Map<String, ? extends RigidBodyBasics> bodyNameToBodyMap = fullRobotModel.getElevator().subtreeStream()
                                                                               .collect(Collectors.toMap(RigidBodyBasics::getName, Function.identity()));

      List<ContactablePlaneBody> contactablePlaneBodies = new ArrayList<>();

      for (int i = 0; i < contactPointParameters.getAdditionalContactNames().size(); i++)
      {
         String contactName = contactPointParameters.getAdditionalContactNames().get(i);
         RigidBodyBasics body = bodyNameToBodyMap.get(contactPointParameters.getAdditionalContactRigidBodyNames().get(i));
         RigidBodyTransform contactPoint = contactPointParameters.getAdditionalContactTransforms().get(i);
         contactablePlaneBodies.add(new SimpleContactPointPlaneBody(contactName, body, contactPoint));
      }

      for (String contactName : contactPointParameters.getPlaneContactNames())
      {
         RigidBodyBasics body = bodyNameToBodyMap.get(contactPointParameters.getPlaneContactBodyName(contactName));
         RigidBodyTransform planeContactFramePose = contactPointParameters.getPlaneContactFramePose(contactName);
         List<? extends Point2DReadOnly> planeContactPoints = contactPointParameters.getPlaneContactPoints(contactName);

         ReferenceFrame planeContactFrame = ReferenceFrameTools.constructFrameWithUnchangingTransformToParent(contactName + "Frame",
                                                                                                              body.getParentJoint().getFrameAfterJoint(),
                                                                                                              planeContactFramePose);
         contactablePlaneBodies.add(new ListOfPointsContactablePlaneBody(body, planeContactFrame, planeContactPoints));
      }

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

   private static void setRobotToSit1Configuration(HumanoidFloatingRootJointRobot robot, DRCRobotJointMap jointMap)
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(robotSide, LegJointName.HIP_YAW)).setQ(robotSide.negateIfRightSide(0.0));
         robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(robotSide, LegJointName.HIP_PITCH)).setQ(-1.9);
         robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(robotSide, LegJointName.KNEE_PITCH)).setQ(1.525);
         robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(robotSide, LegJointName.ANKLE_PITCH)).setQ(0.775);

         robot.getOneDegreeOfFreedomJoint(jointMap.getArmJointName(robotSide, ArmJointName.SHOULDER_PITCH)).setQ(1.1);
         robot.getOneDegreeOfFreedomJoint(jointMap.getArmJointName(robotSide, ArmJointName.SHOULDER_ROLL)).setQ(robotSide.negateIfLeftSide(1.0));
         robot.getOneDegreeOfFreedomJoint(jointMap.getArmJointName(robotSide, ArmJointName.SHOULDER_YAW)).setQ(1.1);
         robot.getOneDegreeOfFreedomJoint(jointMap.getArmJointName(robotSide, ArmJointName.ELBOW_PITCH)).setQ(robotSide.negateIfLeftSide(0.9));
      }

      robot.getOneDegreeOfFreedomJoint(jointMap.getSpineJointName(SpineJointName.SPINE_PITCH)).setQ(-0.1);

      RigidBodyTransform rootJointPose = new RigidBodyTransform();
      rootJointPose.setRotation(new Quaternion(0.0, -0.2, 0.0, 1.0));
      rootJointPose.setTranslation(0.0, 0.0, 0.33);
      robot.getRootJoint().setRotationAndTranslation(rootJointPose);
   }

   private static void setRobotToSit2Configuration(HumanoidFloatingRootJointRobot robot, DRCRobotJointMap jointMap)
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(robotSide, LegJointName.HIP_YAW)).setQ(robotSide.negateIfRightSide(0.0));
         robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(robotSide, LegJointName.HIP_PITCH)).setQ(-1.2);
         robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(robotSide, LegJointName.KNEE_PITCH)).setQ(1.525);
         robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(robotSide, LegJointName.ANKLE_PITCH)).setQ(0.775);

         robot.getOneDegreeOfFreedomJoint(jointMap.getArmJointName(robotSide, ArmJointName.SHOULDER_PITCH)).setQ(1.4);
         robot.getOneDegreeOfFreedomJoint(jointMap.getArmJointName(robotSide, ArmJointName.SHOULDER_ROLL)).setQ(robotSide.negateIfRightSide(0.2));
         robot.getOneDegreeOfFreedomJoint(jointMap.getArmJointName(robotSide, ArmJointName.SHOULDER_YAW)).setQ(1.5);
         robot.getOneDegreeOfFreedomJoint(jointMap.getArmJointName(robotSide, ArmJointName.ELBOW_PITCH)).setQ(robotSide.negateIfLeftSide(1.8));
      }

      robot.getOneDegreeOfFreedomJoint(jointMap.getSpineJointName(SpineJointName.SPINE_PITCH)).setQ(-0.1);

      RigidBodyTransform rootJointPose = new RigidBodyTransform();
      rootJointPose.setRotation(new Quaternion(0.0, -0.6, 0.0, 1.0));
      rootJointPose.setTranslation(0.0, 0.0, 0.23);
      robot.getRootJoint().setRotationAndTranslation(rootJointPose);
   }

   private static void setRobotToSit3Configuration(HumanoidFloatingRootJointRobot robot, DRCRobotJointMap jointMap)
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(robotSide, LegJointName.HIP_YAW)).setQ(robotSide.negateIfRightSide(0.0));
         robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(robotSide, LegJointName.HIP_PITCH)).setQ(-0.875);
         robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(robotSide, LegJointName.KNEE_PITCH)).setQ(1.425);
         robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(robotSide, LegJointName.ANKLE_PITCH)).setQ(0.8);

         robot.getOneDegreeOfFreedomJoint(jointMap.getArmJointName(robotSide, ArmJointName.SHOULDER_PITCH)).setQ(-0.5);
         robot.getOneDegreeOfFreedomJoint(jointMap.getArmJointName(robotSide, ArmJointName.SHOULDER_ROLL)).setQ(robotSide.negateIfRightSide(1.266));
         robot.getOneDegreeOfFreedomJoint(jointMap.getArmJointName(robotSide, ArmJointName.SHOULDER_YAW)).setQ(-3.1);
         robot.getOneDegreeOfFreedomJoint(jointMap.getArmJointName(robotSide, ArmJointName.ELBOW_PITCH)).setQ(robotSide.negateIfLeftSide(2.0));
      }

      robot.getOneDegreeOfFreedomJoint(jointMap.getSpineJointName(SpineJointName.SPINE_PITCH)).setQ(0.4);

      RigidBodyTransform rootJointPose = new RigidBodyTransform();
      rootJointPose.setRotation(new Quaternion(0.0, -0.8, 0.0, 1.0));
      rootJointPose.setTranslation(0.0, 0.0, 0.17);
      robot.getRootJoint().setRotationAndTranslation(rootJointPose);
   }

   private static void setRobotToRollingA1Configuration(HumanoidFloatingRootJointRobot robot, DRCRobotJointMap jointMap)
   {
      RobotSide leftSide = RobotSide.LEFT;
      robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(leftSide, LegJointName.HIP_YAW)).setQ(0.8);
      robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(leftSide, LegJointName.HIP_ROLL)).setQ(0.2);
      robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(leftSide, LegJointName.HIP_PITCH)).setQ(-0.33);
      robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(leftSide, LegJointName.KNEE_PITCH)).setQ(1.6);
      robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(leftSide, LegJointName.ANKLE_PITCH)).setQ(0.3);
      robot.getOneDegreeOfFreedomJoint(jointMap.getArmJointName(leftSide, ArmJointName.SHOULDER_PITCH)).setQ(1.4);
      robot.getOneDegreeOfFreedomJoint(jointMap.getArmJointName(leftSide, ArmJointName.SHOULDER_ROLL)).setQ(0.26);
      robot.getOneDegreeOfFreedomJoint(jointMap.getArmJointName(leftSide, ArmJointName.SHOULDER_YAW)).setQ(1.5);
      robot.getOneDegreeOfFreedomJoint(jointMap.getArmJointName(leftSide, ArmJointName.ELBOW_PITCH)).setQ(-1.8);

      RobotSide rightSide = RobotSide.RIGHT;
      robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(rightSide, LegJointName.HIP_YAW)).setQ(-0.8);
      robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(rightSide, LegJointName.HIP_ROLL)).setQ(-0.1);
      robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(rightSide, LegJointName.HIP_PITCH)).setQ(-0.6);
      robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(rightSide, LegJointName.KNEE_PITCH)).setQ(1.4);
      robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(rightSide, LegJointName.ANKLE_PITCH)).setQ(0.0);
      robot.getOneDegreeOfFreedomJoint(jointMap.getArmJointName(rightSide, ArmJointName.SHOULDER_PITCH)).setQ(1.7);
      robot.getOneDegreeOfFreedomJoint(jointMap.getArmJointName(rightSide, ArmJointName.SHOULDER_ROLL)).setQ(1.3);
      robot.getOneDegreeOfFreedomJoint(jointMap.getArmJointName(rightSide, ArmJointName.SHOULDER_YAW)).setQ(0.0);
      robot.getOneDegreeOfFreedomJoint(jointMap.getArmJointName(rightSide, ArmJointName.ELBOW_PITCH)).setQ(1.95);

      robot.getOneDegreeOfFreedomJoint(jointMap.getSpineJointName(SpineJointName.SPINE_PITCH)).setQ(0.34);
      robot.getOneDegreeOfFreedomJoint(jointMap.getSpineJointName(SpineJointName.SPINE_YAW)).setQ(0.85);

      RigidBodyTransform rootJointPose = new RigidBodyTransform();
      rootJointPose.setRotation(new Quaternion(0.5, -0.5, 0.0, 0.7));
      rootJointPose.setTranslation(0.0, 0.0, 0.26);
      robot.getRootJoint().setRotationAndTranslation(rootJointPose);
   }

   private static void setRobotToRollingA2Configuration(HumanoidFloatingRootJointRobot robot, DRCRobotJointMap jointMap)
   {
      RobotSide leftSide = RobotSide.LEFT;
      robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(leftSide, LegJointName.HIP_YAW)).setQ(0.8);
      robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(leftSide, LegJointName.HIP_ROLL)).setQ(0.2);
      robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(leftSide, LegJointName.HIP_PITCH)).setQ(-0.33);
      robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(leftSide, LegJointName.KNEE_PITCH)).setQ(1.6);
      robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(leftSide, LegJointName.ANKLE_PITCH)).setQ(0.3);
      robot.getOneDegreeOfFreedomJoint(jointMap.getArmJointName(leftSide, ArmJointName.SHOULDER_PITCH)).setQ(-0.15);
      robot.getOneDegreeOfFreedomJoint(jointMap.getArmJointName(leftSide, ArmJointName.SHOULDER_ROLL)).setQ(1.266);
      robot.getOneDegreeOfFreedomJoint(jointMap.getArmJointName(leftSide, ArmJointName.SHOULDER_YAW)).setQ(-3.1);
      robot.getOneDegreeOfFreedomJoint(jointMap.getArmJointName(leftSide, ArmJointName.ELBOW_PITCH)).setQ(-2.0);

      RobotSide rightSide = RobotSide.RIGHT;
      robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(rightSide, LegJointName.HIP_YAW)).setQ(-0.8);
      robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(rightSide, LegJointName.HIP_ROLL)).setQ(-0.1);
      robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(rightSide, LegJointName.HIP_PITCH)).setQ(-0.6);
      robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(rightSide, LegJointName.KNEE_PITCH)).setQ(1.4);
      robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(rightSide, LegJointName.ANKLE_PITCH)).setQ(0.0);
      robot.getOneDegreeOfFreedomJoint(jointMap.getArmJointName(rightSide, ArmJointName.SHOULDER_PITCH)).setQ(1.7);
      robot.getOneDegreeOfFreedomJoint(jointMap.getArmJointName(rightSide, ArmJointName.SHOULDER_ROLL)).setQ(1.3);
      robot.getOneDegreeOfFreedomJoint(jointMap.getArmJointName(rightSide, ArmJointName.SHOULDER_YAW)).setQ(0.0);
      robot.getOneDegreeOfFreedomJoint(jointMap.getArmJointName(rightSide, ArmJointName.ELBOW_PITCH)).setQ(1.95);

      robot.getOneDegreeOfFreedomJoint(jointMap.getSpineJointName(SpineJointName.SPINE_PITCH)).setQ(0.34);
      robot.getOneDegreeOfFreedomJoint(jointMap.getSpineJointName(SpineJointName.SPINE_YAW)).setQ(0.85);

      RigidBodyTransform rootJointPose = new RigidBodyTransform();
      rootJointPose.setRotation(new Quaternion(0.5, -0.5, 0.0, 0.7));
      rootJointPose.setTranslation(0.0, 0.0, 0.26);
      robot.getRootJoint().setRotationAndTranslation(rootJointPose);
   }

   private static void setRobotToRollingA3Configuration(HumanoidFloatingRootJointRobot robot, DRCRobotJointMap jointMap)
   {
      RobotSide leftSide = RobotSide.LEFT;
      robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(leftSide, LegJointName.HIP_YAW)).setQ(0.8);
      robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(leftSide, LegJointName.HIP_ROLL)).setQ(0.2);
      robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(leftSide, LegJointName.HIP_PITCH)).setQ(-0.450);
      robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(leftSide, LegJointName.KNEE_PITCH)).setQ(1.4);
      robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(leftSide, LegJointName.ANKLE_PITCH)).setQ(0.425);
      robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(leftSide, LegJointName.ANKLE_ROLL)).setQ(-0.2);
      robot.getOneDegreeOfFreedomJoint(jointMap.getArmJointName(leftSide, ArmJointName.SHOULDER_PITCH)).setQ(1.7);
      robot.getOneDegreeOfFreedomJoint(jointMap.getArmJointName(leftSide, ArmJointName.SHOULDER_ROLL)).setQ(-1.5);
      robot.getOneDegreeOfFreedomJoint(jointMap.getArmJointName(leftSide, ArmJointName.SHOULDER_YAW)).setQ(0);
      robot.getOneDegreeOfFreedomJoint(jointMap.getArmJointName(leftSide, ArmJointName.ELBOW_PITCH)).setQ(-1.2);

      RobotSide rightSide = RobotSide.RIGHT;
      robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(rightSide, LegJointName.HIP_YAW)).setQ(-0.55);
      robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(rightSide, LegJointName.HIP_ROLL)).setQ(-0.13);
      robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(rightSide, LegJointName.HIP_PITCH)).setQ(-0.6);
      robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(rightSide, LegJointName.KNEE_PITCH)).setQ(1.4);
      robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(rightSide, LegJointName.ANKLE_PITCH)).setQ(0.0);
      robot.getOneDegreeOfFreedomJoint(jointMap.getArmJointName(rightSide, ArmJointName.SHOULDER_PITCH)).setQ(1.6);
      robot.getOneDegreeOfFreedomJoint(jointMap.getArmJointName(rightSide, ArmJointName.SHOULDER_ROLL)).setQ(1.3);
      robot.getOneDegreeOfFreedomJoint(jointMap.getArmJointName(rightSide, ArmJointName.SHOULDER_YAW)).setQ(0.0);
      robot.getOneDegreeOfFreedomJoint(jointMap.getArmJointName(rightSide, ArmJointName.ELBOW_PITCH)).setQ(2.0);

      robot.getOneDegreeOfFreedomJoint(jointMap.getSpineJointName(SpineJointName.SPINE_PITCH)).setQ(0.36);
      robot.getOneDegreeOfFreedomJoint(jointMap.getSpineJointName(SpineJointName.SPINE_YAW)).setQ(0.60);

      RigidBodyTransform rootJointPose = new RigidBodyTransform();
      rootJointPose.setRotation(new Quaternion(0.0, -0.585, 1.3));
      rootJointPose.setTranslation(0.0, 0.0, 0.31);
      robot.getRootJoint().setRotationAndTranslation(rootJointPose);
   }

   private static void setRobotToRollingA4Configuration(HumanoidFloatingRootJointRobot robot, DRCRobotJointMap jointMap)
   {
      RobotSide leftSide = RobotSide.LEFT;
      robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(leftSide, LegJointName.HIP_YAW)).setQ(0.975);
      robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(leftSide, LegJointName.HIP_ROLL)).setQ(0.5);
      robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(leftSide, LegJointName.HIP_PITCH)).setQ(-0.450);
      robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(leftSide, LegJointName.KNEE_PITCH)).setQ(1.4);
      robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(leftSide, LegJointName.ANKLE_PITCH)).setQ(0.425);
      robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(leftSide, LegJointName.ANKLE_ROLL)).setQ(-0.3);
      robot.getOneDegreeOfFreedomJoint(jointMap.getArmJointName(leftSide, ArmJointName.SHOULDER_PITCH)).setQ(1.7);
      robot.getOneDegreeOfFreedomJoint(jointMap.getArmJointName(leftSide, ArmJointName.SHOULDER_ROLL)).setQ(-1.5);
      robot.getOneDegreeOfFreedomJoint(jointMap.getArmJointName(leftSide, ArmJointName.SHOULDER_YAW)).setQ(0);
      robot.getOneDegreeOfFreedomJoint(jointMap.getArmJointName(leftSide, ArmJointName.ELBOW_PITCH)).setQ(-1.2);

      RobotSide rightSide = RobotSide.RIGHT;
      robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(rightSide, LegJointName.HIP_YAW)).setQ(-0.41);
      robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(rightSide, LegJointName.HIP_ROLL)).setQ(0.25);
      robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(rightSide, LegJointName.HIP_PITCH)).setQ(-0.6);
      robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(rightSide, LegJointName.KNEE_PITCH)).setQ(1.4);
      robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(rightSide, LegJointName.ANKLE_PITCH)).setQ(0.0);
      robot.getOneDegreeOfFreedomJoint(jointMap.getArmJointName(rightSide, ArmJointName.SHOULDER_PITCH)).setQ(1.2);
      robot.getOneDegreeOfFreedomJoint(jointMap.getArmJointName(rightSide, ArmJointName.SHOULDER_ROLL)).setQ(1.1);
      robot.getOneDegreeOfFreedomJoint(jointMap.getArmJointName(rightSide, ArmJointName.SHOULDER_YAW)).setQ(0.0);
      robot.getOneDegreeOfFreedomJoint(jointMap.getArmJointName(rightSide, ArmJointName.ELBOW_PITCH)).setQ(2.0);

      robot.getOneDegreeOfFreedomJoint(jointMap.getSpineJointName(SpineJointName.SPINE_YAW)).setQ(0.30);
      robot.getOneDegreeOfFreedomJoint(jointMap.getSpineJointName(SpineJointName.SPINE_PITCH)).setQ(0.36);
      robot.getOneDegreeOfFreedomJoint(jointMap.getSpineJointName(SpineJointName.SPINE_ROLL)).setQ(-0.2);

      RigidBodyTransform rootJointPose = new RigidBodyTransform();
      rootJointPose.setRotation(new Quaternion(0.0, -0.385, 1.2));
      rootJointPose.setTranslation(0.0, 0.0, 0.34);
      robot.getRootJoint().setRotationAndTranslation(rootJointPose);
   }

   private static void setRobotToRollingA5Configuration(HumanoidFloatingRootJointRobot robot, DRCRobotJointMap jointMap)
   {
      RobotSide leftSide = RobotSide.LEFT;
      robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(leftSide, LegJointName.HIP_YAW)).setQ(1.075);
      robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(leftSide, LegJointName.HIP_ROLL)).setQ(0.5);
      robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(leftSide, LegJointName.HIP_PITCH)).setQ(-0.250);
      robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(leftSide, LegJointName.KNEE_PITCH)).setQ(1.4);
      robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(leftSide, LegJointName.ANKLE_PITCH)).setQ(0.225);
      robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(leftSide, LegJointName.ANKLE_ROLL)).setQ(-0.3);
      robot.getOneDegreeOfFreedomJoint(jointMap.getArmJointName(leftSide, ArmJointName.SHOULDER_PITCH)).setQ(1.7);
      robot.getOneDegreeOfFreedomJoint(jointMap.getArmJointName(leftSide, ArmJointName.SHOULDER_ROLL)).setQ(-1.5);
      robot.getOneDegreeOfFreedomJoint(jointMap.getArmJointName(leftSide, ArmJointName.SHOULDER_YAW)).setQ(0);
      robot.getOneDegreeOfFreedomJoint(jointMap.getArmJointName(leftSide, ArmJointName.ELBOW_PITCH)).setQ(-1.2);

      RobotSide rightSide = RobotSide.RIGHT;
      robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(rightSide, LegJointName.HIP_YAW)).setQ(-0.01);
      robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(rightSide, LegJointName.HIP_ROLL)).setQ(0.3);
      robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(rightSide, LegJointName.HIP_PITCH)).setQ(-0.6);
      robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(rightSide, LegJointName.KNEE_PITCH)).setQ(1.4);
      robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(rightSide, LegJointName.ANKLE_PITCH)).setQ(0.0);
      robot.getOneDegreeOfFreedomJoint(jointMap.getArmJointName(rightSide, ArmJointName.SHOULDER_PITCH)).setQ(1.2);
      robot.getOneDegreeOfFreedomJoint(jointMap.getArmJointName(rightSide, ArmJointName.SHOULDER_ROLL)).setQ(0.9);
      robot.getOneDegreeOfFreedomJoint(jointMap.getArmJointName(rightSide, ArmJointName.SHOULDER_YAW)).setQ(0.3);
      robot.getOneDegreeOfFreedomJoint(jointMap.getArmJointName(rightSide, ArmJointName.ELBOW_PITCH)).setQ(2.0);

      robot.getOneDegreeOfFreedomJoint(jointMap.getSpineJointName(SpineJointName.SPINE_YAW)).setQ(0.10);
      robot.getOneDegreeOfFreedomJoint(jointMap.getSpineJointName(SpineJointName.SPINE_PITCH)).setQ(0.36);
      robot.getOneDegreeOfFreedomJoint(jointMap.getSpineJointName(SpineJointName.SPINE_ROLL)).setQ(-0.2);

      RigidBodyTransform rootJointPose = new RigidBodyTransform();
      rootJointPose.setRotation(new Quaternion(0.0, 0.0, 1.2));
      rootJointPose.setTranslation(0.0, 0.0, 0.34);
      robot.getRootJoint().setRotationAndTranslation(rootJointPose);
   }

   private static void setRobotToRollingA6Configuration(HumanoidFloatingRootJointRobot robot, DRCRobotJointMap jointMap)
   {
      RobotSide leftSide = RobotSide.LEFT;
      robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(leftSide, LegJointName.HIP_YAW)).setQ(0.875);
      robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(leftSide, LegJointName.HIP_ROLL)).setQ(0.5);
      robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(leftSide, LegJointName.HIP_PITCH)).setQ(-0.550);
      robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(leftSide, LegJointName.KNEE_PITCH)).setQ(1.4);
      robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(leftSide, LegJointName.ANKLE_PITCH)).setQ(0.225);
      robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(leftSide, LegJointName.ANKLE_ROLL)).setQ(-0.3);
      robot.getOneDegreeOfFreedomJoint(jointMap.getArmJointName(leftSide, ArmJointName.SHOULDER_PITCH)).setQ(1.7);
      robot.getOneDegreeOfFreedomJoint(jointMap.getArmJointName(leftSide, ArmJointName.SHOULDER_ROLL)).setQ(-1.5);
      robot.getOneDegreeOfFreedomJoint(jointMap.getArmJointName(leftSide, ArmJointName.SHOULDER_YAW)).setQ(0);
      robot.getOneDegreeOfFreedomJoint(jointMap.getArmJointName(leftSide, ArmJointName.ELBOW_PITCH)).setQ(-1.2);

      RobotSide rightSide = RobotSide.RIGHT;
      robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(rightSide, LegJointName.HIP_YAW)).setQ(-0.01);
      robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(rightSide, LegJointName.HIP_ROLL)).setQ(0.3);
      robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(rightSide, LegJointName.HIP_PITCH)).setQ(-0.6);
      robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(rightSide, LegJointName.KNEE_PITCH)).setQ(1.4);
      robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(rightSide, LegJointName.ANKLE_PITCH)).setQ(0.0);
      robot.getOneDegreeOfFreedomJoint(jointMap.getArmJointName(rightSide, ArmJointName.SHOULDER_PITCH)).setQ(1.2);
      robot.getOneDegreeOfFreedomJoint(jointMap.getArmJointName(rightSide, ArmJointName.SHOULDER_ROLL)).setQ(0.9);
      robot.getOneDegreeOfFreedomJoint(jointMap.getArmJointName(rightSide, ArmJointName.SHOULDER_YAW)).setQ(0.3);
      robot.getOneDegreeOfFreedomJoint(jointMap.getArmJointName(rightSide, ArmJointName.ELBOW_PITCH)).setQ(2.0);

      robot.getOneDegreeOfFreedomJoint(jointMap.getSpineJointName(SpineJointName.SPINE_YAW)).setQ(0.10);
      robot.getOneDegreeOfFreedomJoint(jointMap.getSpineJointName(SpineJointName.SPINE_PITCH)).setQ(0.36);
      robot.getOneDegreeOfFreedomJoint(jointMap.getSpineJointName(SpineJointName.SPINE_ROLL)).setQ(-0.2);

      RigidBodyTransform rootJointPose = new RigidBodyTransform();
      rootJointPose.setRotation(new Quaternion(0.0, 0.0, 1.2));
      rootJointPose.setTranslation(0.0, 0.0, 0.34);
      robot.getRootJoint().setRotationAndTranslation(rootJointPose);
   }

   private static void setRobotToRollingA7Configuration(HumanoidFloatingRootJointRobot robot, DRCRobotJointMap jointMap)
   {
      RobotSide leftSide = RobotSide.LEFT;
      robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(leftSide, LegJointName.HIP_YAW)).setQ(-0.5);
      robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(leftSide, LegJointName.HIP_ROLL)).setQ(0.35);
      robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(leftSide, LegJointName.HIP_PITCH)).setQ(-0.950);
      robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(leftSide, LegJointName.KNEE_PITCH)).setQ(0.8);
      robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(leftSide, LegJointName.ANKLE_PITCH)).setQ(0.225);
      robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(leftSide, LegJointName.ANKLE_ROLL)).setQ(-0.3);
      robot.getOneDegreeOfFreedomJoint(jointMap.getArmJointName(leftSide, ArmJointName.SHOULDER_PITCH)).setQ(1.7);
      robot.getOneDegreeOfFreedomJoint(jointMap.getArmJointName(leftSide, ArmJointName.SHOULDER_ROLL)).setQ(-1.5);
      robot.getOneDegreeOfFreedomJoint(jointMap.getArmJointName(leftSide, ArmJointName.SHOULDER_YAW)).setQ(0);
      robot.getOneDegreeOfFreedomJoint(jointMap.getArmJointName(leftSide, ArmJointName.ELBOW_PITCH)).setQ(-1.2);

      RobotSide rightSide = RobotSide.RIGHT;
      robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(rightSide, LegJointName.HIP_YAW)).setQ(-0.01);
      robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(rightSide, LegJointName.HIP_ROLL)).setQ(0.3);
      robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(rightSide, LegJointName.HIP_PITCH)).setQ(-0.6);
      robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(rightSide, LegJointName.KNEE_PITCH)).setQ(1.4);
      robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(rightSide, LegJointName.ANKLE_PITCH)).setQ(0.0);
      robot.getOneDegreeOfFreedomJoint(jointMap.getArmJointName(rightSide, ArmJointName.SHOULDER_PITCH)).setQ(1.2);
      robot.getOneDegreeOfFreedomJoint(jointMap.getArmJointName(rightSide, ArmJointName.SHOULDER_ROLL)).setQ(0.9);
      robot.getOneDegreeOfFreedomJoint(jointMap.getArmJointName(rightSide, ArmJointName.SHOULDER_YAW)).setQ(0.3);
      robot.getOneDegreeOfFreedomJoint(jointMap.getArmJointName(rightSide, ArmJointName.ELBOW_PITCH)).setQ(2.0);

      robot.getOneDegreeOfFreedomJoint(jointMap.getSpineJointName(SpineJointName.SPINE_YAW)).setQ(0.10);
      robot.getOneDegreeOfFreedomJoint(jointMap.getSpineJointName(SpineJointName.SPINE_PITCH)).setQ(0.36);
      robot.getOneDegreeOfFreedomJoint(jointMap.getSpineJointName(SpineJointName.SPINE_ROLL)).setQ(-0.2);

      RigidBodyTransform rootJointPose = new RigidBodyTransform();
      rootJointPose.setRotation(new Quaternion(0.0, 0.0, 1.2));
      rootJointPose.setTranslation(0.0, 0.0, 0.34);
      robot.getRootJoint().setRotationAndTranslation(rootJointPose);
   }

   private static void setRobotToLieDown1Configuration(HumanoidFloatingRootJointRobot robot, DRCRobotJointMap jointMap)
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(robotSide, LegJointName.HIP_YAW)).setQ(0.0);
         robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(robotSide, LegJointName.HIP_ROLL)).setQ(0.0);
         robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(robotSide, LegJointName.HIP_PITCH)).setQ(-0.325);
         robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(robotSide, LegJointName.KNEE_PITCH)).setQ(1.1);
         robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(robotSide, LegJointName.ANKLE_PITCH)).setQ(0.8);
         robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(robotSide, LegJointName.ANKLE_ROLL)).setQ(0.0);
         robot.getOneDegreeOfFreedomJoint(jointMap.getArmJointName(robotSide, ArmJointName.SHOULDER_PITCH)).setQ(0.0);
         robot.getOneDegreeOfFreedomJoint(jointMap.getArmJointName(robotSide, ArmJointName.SHOULDER_ROLL)).setQ(robotSide.negateIfLeftSide(1.5));
         robot.getOneDegreeOfFreedomJoint(jointMap.getArmJointName(robotSide, ArmJointName.SHOULDER_YAW)).setQ(0);
         robot.getOneDegreeOfFreedomJoint(jointMap.getArmJointName(robotSide, ArmJointName.ELBOW_PITCH)).setQ(robotSide.negateIfLeftSide(2.0));
      }

      robot.getOneDegreeOfFreedomJoint(jointMap.getSpineJointName(SpineJointName.SPINE_YAW)).setQ(0.0);
      robot.getOneDegreeOfFreedomJoint(jointMap.getSpineJointName(SpineJointName.SPINE_PITCH)).setQ(0.0);
      robot.getOneDegreeOfFreedomJoint(jointMap.getSpineJointName(SpineJointName.SPINE_ROLL)).setQ(0.0);

      RigidBodyTransform rootJointPose = new RigidBodyTransform();
      rootJointPose.setRotation(new Quaternion(0.0, -Math.PI / 2.0, Math.PI / 2.0));
      rootJointPose.setTranslation(0.0, 0.0, 0.245);
      robot.getRootJoint().setRotationAndTranslation(rootJointPose);
   }

   private static void setRobotToLieDown2Configuration(HumanoidFloatingRootJointRobot robot, DRCRobotJointMap jointMap)
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(robotSide, LegJointName.HIP_YAW)).setQ(0.0);
         robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(robotSide, LegJointName.HIP_ROLL)).setQ(0.0);
         robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(robotSide, LegJointName.HIP_PITCH)).setQ(-0.325);
         robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(robotSide, LegJointName.KNEE_PITCH)).setQ(1.1);
         robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(robotSide, LegJointName.ANKLE_PITCH)).setQ(0.8);
         robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(robotSide, LegJointName.ANKLE_ROLL)).setQ(0.0);
         robot.getOneDegreeOfFreedomJoint(jointMap.getArmJointName(robotSide, ArmJointName.SHOULDER_PITCH)).setQ(0.0);
         robot.getOneDegreeOfFreedomJoint(jointMap.getArmJointName(robotSide, ArmJointName.SHOULDER_ROLL)).setQ(robotSide.negateIfLeftSide(1.5));
         robot.getOneDegreeOfFreedomJoint(jointMap.getArmJointName(robotSide, ArmJointName.SHOULDER_YAW)).setQ(0);
         robot.getOneDegreeOfFreedomJoint(jointMap.getArmJointName(robotSide, ArmJointName.ELBOW_PITCH)).setQ(robotSide.negateIfLeftSide(2.0));
      }

      RobotSide leftSide = RobotSide.LEFT;
      robot.getOneDegreeOfFreedomJoint(jointMap.getArmJointName(leftSide, ArmJointName.SHOULDER_PITCH)).setQ(0.5 * Math.PI);
      robot.getOneDegreeOfFreedomJoint(jointMap.getArmJointName(leftSide, ArmJointName.SHOULDER_ROLL)).setQ(0.4);
      robot.getOneDegreeOfFreedomJoint(jointMap.getArmJointName(leftSide, ArmJointName.SHOULDER_YAW)).setQ(0.5 * Math.PI);
      robot.getOneDegreeOfFreedomJoint(jointMap.getArmJointName(leftSide, ArmJointName.ELBOW_PITCH)).setQ(-1.75);

      RobotSide rightSide = RobotSide.RIGHT;
      robot.getOneDegreeOfFreedomJoint(jointMap.getArmJointName(rightSide, ArmJointName.SHOULDER_PITCH)).setQ(0.0);
      robot.getOneDegreeOfFreedomJoint(jointMap.getArmJointName(rightSide, ArmJointName.SHOULDER_ROLL)).setQ(1.5);
      robot.getOneDegreeOfFreedomJoint(jointMap.getArmJointName(rightSide, ArmJointName.SHOULDER_YAW)).setQ(0.0);
      robot.getOneDegreeOfFreedomJoint(jointMap.getArmJointName(rightSide, ArmJointName.ELBOW_PITCH)).setQ(2.0);

      robot.getOneDegreeOfFreedomJoint(jointMap.getSpineJointName(SpineJointName.SPINE_YAW)).setQ(0.0);
      robot.getOneDegreeOfFreedomJoint(jointMap.getSpineJointName(SpineJointName.SPINE_PITCH)).setQ(0.0);
      robot.getOneDegreeOfFreedomJoint(jointMap.getSpineJointName(SpineJointName.SPINE_ROLL)).setQ(0.0);

      RigidBodyTransform rootJointPose = new RigidBodyTransform();
      rootJointPose.setRotation(new Quaternion(0.0, -Math.PI / 2.0, Math.PI / 2.0));
      rootJointPose.setTranslation(0.0, 0.0, 0.245);
      robot.getRootJoint().setRotationAndTranslation(rootJointPose);
   }

   private static void setRobotToRollingB1Configuration(HumanoidFloatingRootJointRobot robot, DRCRobotJointMap jointMap)
   {
      RobotSide leftSide = RobotSide.LEFT;
      robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(leftSide, LegJointName.HIP_YAW)).setQ(0.5);
      robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(leftSide, LegJointName.HIP_ROLL)).setQ(0.1);
      robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(leftSide, LegJointName.HIP_PITCH)).setQ(-0.250);
      robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(leftSide, LegJointName.KNEE_PITCH)).setQ(1.2);
      robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(leftSide, LegJointName.ANKLE_PITCH)).setQ(0.625);
      robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(leftSide, LegJointName.ANKLE_ROLL)).setQ(0.0);
      robot.getOneDegreeOfFreedomJoint(jointMap.getArmJointName(leftSide, ArmJointName.SHOULDER_PITCH)).setQ(0.5 * Math.PI);
      robot.getOneDegreeOfFreedomJoint(jointMap.getArmJointName(leftSide, ArmJointName.SHOULDER_ROLL)).setQ(-0.5);
      robot.getOneDegreeOfFreedomJoint(jointMap.getArmJointName(leftSide, ArmJointName.SHOULDER_YAW)).setQ(0.5 * Math.PI);
      robot.getOneDegreeOfFreedomJoint(jointMap.getArmJointName(leftSide, ArmJointName.ELBOW_PITCH)).setQ(-1.45);

      RobotSide rightSide = RobotSide.RIGHT;
      robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(rightSide, LegJointName.HIP_YAW)).setQ(-0.2);
      robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(rightSide, LegJointName.HIP_ROLL)).setQ(0.0);
      robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(rightSide, LegJointName.HIP_PITCH)).setQ(-0.375);
      robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(rightSide, LegJointName.KNEE_PITCH)).setQ(1.1);
      robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(rightSide, LegJointName.ANKLE_PITCH)).setQ(0.82);
      robot.getOneDegreeOfFreedomJoint(jointMap.getArmJointName(rightSide, ArmJointName.SHOULDER_PITCH)).setQ(0.0);
      robot.getOneDegreeOfFreedomJoint(jointMap.getArmJointName(rightSide, ArmJointName.SHOULDER_ROLL)).setQ(1.5);
      robot.getOneDegreeOfFreedomJoint(jointMap.getArmJointName(rightSide, ArmJointName.SHOULDER_YAW)).setQ(0.0);
      robot.getOneDegreeOfFreedomJoint(jointMap.getArmJointName(rightSide, ArmJointName.ELBOW_PITCH)).setQ(2.0);

      robot.getOneDegreeOfFreedomJoint(jointMap.getSpineJointName(SpineJointName.SPINE_YAW)).setQ(0.0);
      robot.getOneDegreeOfFreedomJoint(jointMap.getSpineJointName(SpineJointName.SPINE_PITCH)).setQ(0.0);
      robot.getOneDegreeOfFreedomJoint(jointMap.getSpineJointName(SpineJointName.SPINE_ROLL)).setQ(0.0);

      RigidBodyTransform rootJointPose = new RigidBodyTransform();
      rootJointPose.setRotation(new Quaternion(0.0, -1.07, Math.PI / 2.0));
      rootJointPose.setTranslation(0.0, 0.0, 0.275);
      robot.getRootJoint().setRotationAndTranslation(rootJointPose);
   }

   private static void setRobotToRollingB2Configuration(HumanoidFloatingRootJointRobot robot, DRCRobotJointMap jointMap)
   {
      RobotSide leftSide = RobotSide.LEFT;
      robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(leftSide, LegJointName.HIP_YAW)).setQ(0.5);
      robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(leftSide, LegJointName.HIP_ROLL)).setQ(0.1);
      robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(leftSide, LegJointName.HIP_PITCH)).setQ(-0.220);
      robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(leftSide, LegJointName.KNEE_PITCH)).setQ(1.2);
      robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(leftSide, LegJointName.ANKLE_PITCH)).setQ(0.625);
      robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(leftSide, LegJointName.ANKLE_ROLL)).setQ(0.0);
      robot.getOneDegreeOfFreedomJoint(jointMap.getArmJointName(leftSide, ArmJointName.SHOULDER_PITCH)).setQ(0.5 * Math.PI);
      robot.getOneDegreeOfFreedomJoint(jointMap.getArmJointName(leftSide, ArmJointName.SHOULDER_ROLL)).setQ(-0.75);
      robot.getOneDegreeOfFreedomJoint(jointMap.getArmJointName(leftSide, ArmJointName.SHOULDER_YAW)).setQ(0.5 * Math.PI);
      robot.getOneDegreeOfFreedomJoint(jointMap.getArmJointName(leftSide, ArmJointName.ELBOW_PITCH)).setQ(-1.35);

      RobotSide rightSide = RobotSide.RIGHT;
      robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(rightSide, LegJointName.HIP_YAW)).setQ(-0.2);
      robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(rightSide, LegJointName.HIP_ROLL)).setQ(0.0);
      robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(rightSide, LegJointName.HIP_PITCH)).setQ(-0.375);
      robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(rightSide, LegJointName.KNEE_PITCH)).setQ(1.1);
      robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(rightSide, LegJointName.ANKLE_PITCH)).setQ(0.82);
      robot.getOneDegreeOfFreedomJoint(jointMap.getArmJointName(rightSide, ArmJointName.SHOULDER_PITCH)).setQ(0.0);
      robot.getOneDegreeOfFreedomJoint(jointMap.getArmJointName(rightSide, ArmJointName.SHOULDER_ROLL)).setQ(1.5);
      robot.getOneDegreeOfFreedomJoint(jointMap.getArmJointName(rightSide, ArmJointName.SHOULDER_YAW)).setQ(0.0);
      robot.getOneDegreeOfFreedomJoint(jointMap.getArmJointName(rightSide, ArmJointName.ELBOW_PITCH)).setQ(2.0);

      robot.getOneDegreeOfFreedomJoint(jointMap.getSpineJointName(SpineJointName.SPINE_YAW)).setQ(0.0);
      robot.getOneDegreeOfFreedomJoint(jointMap.getSpineJointName(SpineJointName.SPINE_PITCH)).setQ(0.0);
      robot.getOneDegreeOfFreedomJoint(jointMap.getSpineJointName(SpineJointName.SPINE_ROLL)).setQ(0.0);

      RigidBodyTransform rootJointPose = new RigidBodyTransform();
      rootJointPose.setRotation(new Quaternion(0.0, -0.9, Math.PI / 2.0));
      rootJointPose.setTranslation(0.0, 0.0, 0.26);
      robot.getRootJoint().setRotationAndTranslation(rootJointPose);
   }

   private static void setRobotToRollingB3Configuration(HumanoidFloatingRootJointRobot robot, DRCRobotJointMap jointMap)
   {
      RobotSide leftSide = RobotSide.LEFT;
      robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(leftSide, LegJointName.HIP_YAW)).setQ(0.7);
      robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(leftSide, LegJointName.HIP_ROLL)).setQ(0.3);
      robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(leftSide, LegJointName.HIP_PITCH)).setQ(-0.05);
      robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(leftSide, LegJointName.KNEE_PITCH)).setQ(1.2);
      robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(leftSide, LegJointName.ANKLE_PITCH)).setQ(0.5);
      robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(leftSide, LegJointName.ANKLE_ROLL)).setQ(0.0);
      robot.getOneDegreeOfFreedomJoint(jointMap.getArmJointName(leftSide, ArmJointName.SHOULDER_PITCH)).setQ(0.5 * Math.PI);
      robot.getOneDegreeOfFreedomJoint(jointMap.getArmJointName(leftSide, ArmJointName.SHOULDER_ROLL)).setQ(-1.25);
      robot.getOneDegreeOfFreedomJoint(jointMap.getArmJointName(leftSide, ArmJointName.SHOULDER_YAW)).setQ(0.5 * Math.PI);
      robot.getOneDegreeOfFreedomJoint(jointMap.getArmJointName(leftSide, ArmJointName.ELBOW_PITCH)).setQ(-1.35);

      RobotSide rightSide = RobotSide.RIGHT;
      robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(rightSide, LegJointName.HIP_YAW)).setQ(-0.65);
      robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(rightSide, LegJointName.HIP_ROLL)).setQ(-0.12);
      robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(rightSide, LegJointName.HIP_PITCH)).setQ(-0.375);
      robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(rightSide, LegJointName.KNEE_PITCH)).setQ(1.1);
      robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(rightSide, LegJointName.ANKLE_PITCH)).setQ(0.82);
      robot.getOneDegreeOfFreedomJoint(jointMap.getArmJointName(rightSide, ArmJointName.SHOULDER_PITCH)).setQ(0.0);
      robot.getOneDegreeOfFreedomJoint(jointMap.getArmJointName(rightSide, ArmJointName.SHOULDER_ROLL)).setQ(1.5);
      robot.getOneDegreeOfFreedomJoint(jointMap.getArmJointName(rightSide, ArmJointName.SHOULDER_YAW)).setQ(0.0);
      robot.getOneDegreeOfFreedomJoint(jointMap.getArmJointName(rightSide, ArmJointName.ELBOW_PITCH)).setQ(2.0);

      robot.getOneDegreeOfFreedomJoint(jointMap.getSpineJointName(SpineJointName.SPINE_YAW)).setQ(0.0);
      robot.getOneDegreeOfFreedomJoint(jointMap.getSpineJointName(SpineJointName.SPINE_PITCH)).setQ(0.0);
      robot.getOneDegreeOfFreedomJoint(jointMap.getSpineJointName(SpineJointName.SPINE_ROLL)).setQ(0.0);

      RigidBodyTransform rootJointPose = new RigidBodyTransform();
      rootJointPose.setRotation(new Quaternion(0.0, -0.6, Math.PI / 2.0));
      rootJointPose.setTranslation(0.0, 0.0, 0.30);
      robot.getRootJoint().setRotationAndTranslation(rootJointPose);
   }

   private static void setRobotToRollingB4Configuration(HumanoidFloatingRootJointRobot robot, DRCRobotJointMap jointMap)
   {
      RobotSide leftSide = RobotSide.LEFT;
      robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(leftSide, LegJointName.HIP_YAW)).setQ(1.0);
      robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(leftSide, LegJointName.HIP_ROLL)).setQ(0.4);
      robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(leftSide, LegJointName.HIP_PITCH)).setQ(0.025);
      robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(leftSide, LegJointName.KNEE_PITCH)).setQ(1.2);
      robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(leftSide, LegJointName.ANKLE_PITCH)).setQ(0.45);
      robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(leftSide, LegJointName.ANKLE_ROLL)).setQ(0.0);
      robot.getOneDegreeOfFreedomJoint(jointMap.getArmJointName(leftSide, ArmJointName.SHOULDER_PITCH)).setQ(0.5 * Math.PI);
      robot.getOneDegreeOfFreedomJoint(jointMap.getArmJointName(leftSide, ArmJointName.SHOULDER_ROLL)).setQ(-1.25);
      robot.getOneDegreeOfFreedomJoint(jointMap.getArmJointName(leftSide, ArmJointName.SHOULDER_YAW)).setQ(0.5 * Math.PI);
      robot.getOneDegreeOfFreedomJoint(jointMap.getArmJointName(leftSide, ArmJointName.ELBOW_PITCH)).setQ(-1.35);

      RobotSide rightSide = RobotSide.RIGHT;
      robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(rightSide, LegJointName.HIP_YAW)).setQ(-0.35);
      robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(rightSide, LegJointName.HIP_ROLL)).setQ(-0.13);
      robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(rightSide, LegJointName.HIP_PITCH)).setQ(-0.375);
      robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(rightSide, LegJointName.KNEE_PITCH)).setQ(1.1);
      robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(rightSide, LegJointName.ANKLE_PITCH)).setQ(0.82);
      robot.getOneDegreeOfFreedomJoint(jointMap.getArmJointName(rightSide, ArmJointName.SHOULDER_PITCH)).setQ(0.0);
      robot.getOneDegreeOfFreedomJoint(jointMap.getArmJointName(rightSide, ArmJointName.SHOULDER_ROLL)).setQ(1.519);
      robot.getOneDegreeOfFreedomJoint(jointMap.getArmJointName(rightSide, ArmJointName.SHOULDER_YAW)).setQ(-0.3);
      robot.getOneDegreeOfFreedomJoint(jointMap.getArmJointName(rightSide, ArmJointName.ELBOW_PITCH)).setQ(2.0);

      robot.getOneDegreeOfFreedomJoint(jointMap.getSpineJointName(SpineJointName.SPINE_YAW)).setQ(0.0);
      robot.getOneDegreeOfFreedomJoint(jointMap.getSpineJointName(SpineJointName.SPINE_PITCH)).setQ(0.0);
      robot.getOneDegreeOfFreedomJoint(jointMap.getSpineJointName(SpineJointName.SPINE_ROLL)).setQ(0.0);

      RigidBodyTransform rootJointPose = new RigidBodyTransform();
      rootJointPose.setRotation(new Quaternion(0.0, -0.3, Math.PI / 2.0));
      rootJointPose.setTranslation(0.0, 0.0, 0.32);
      robot.getRootJoint().setRotationAndTranslation(rootJointPose);
   }

   private static void setRobotToRollingB5Configuration(HumanoidFloatingRootJointRobot robot, DRCRobotJointMap jointMap)
   {
      RobotSide leftSide = RobotSide.LEFT;
      robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(leftSide, LegJointName.HIP_YAW)).setQ(1.1);
      robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(leftSide, LegJointName.HIP_ROLL)).setQ(0.4);
      robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(leftSide, LegJointName.HIP_PITCH)).setQ(0.2);
      robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(leftSide, LegJointName.KNEE_PITCH)).setQ(1.2);
      robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(leftSide, LegJointName.ANKLE_PITCH)).setQ(0.4);
      robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(leftSide, LegJointName.ANKLE_ROLL)).setQ(0.0);
      robot.getOneDegreeOfFreedomJoint(jointMap.getArmJointName(leftSide, ArmJointName.SHOULDER_PITCH)).setQ(-0.5 * Math.PI);
      robot.getOneDegreeOfFreedomJoint(jointMap.getArmJointName(leftSide, ArmJointName.SHOULDER_ROLL)).setQ(-0.5 * Math.PI);
      robot.getOneDegreeOfFreedomJoint(jointMap.getArmJointName(leftSide, ArmJointName.SHOULDER_YAW)).setQ(0.5 * Math.PI);
      robot.getOneDegreeOfFreedomJoint(jointMap.getArmJointName(leftSide, ArmJointName.ELBOW_PITCH)).setQ(-0.7);

      RobotSide rightSide = RobotSide.RIGHT;
      robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(rightSide, LegJointName.HIP_YAW)).setQ(-0.075);
      robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(rightSide, LegJointName.HIP_ROLL)).setQ(-0.15);
      robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(rightSide, LegJointName.HIP_PITCH)).setQ(-0.375);
      robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(rightSide, LegJointName.KNEE_PITCH)).setQ(1.1);
      robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(rightSide, LegJointName.ANKLE_PITCH)).setQ(0.82);
      robot.getOneDegreeOfFreedomJoint(jointMap.getArmJointName(rightSide, ArmJointName.SHOULDER_PITCH)).setQ(0.0);
      robot.getOneDegreeOfFreedomJoint(jointMap.getArmJointName(rightSide, ArmJointName.SHOULDER_ROLL)).setQ(1.519);
      robot.getOneDegreeOfFreedomJoint(jointMap.getArmJointName(rightSide, ArmJointName.SHOULDER_YAW)).setQ(0.0);
      robot.getOneDegreeOfFreedomJoint(jointMap.getArmJointName(rightSide, ArmJointName.ELBOW_PITCH)).setQ(2.0);

      robot.getOneDegreeOfFreedomJoint(jointMap.getSpineJointName(SpineJointName.SPINE_YAW)).setQ(0.0);
      robot.getOneDegreeOfFreedomJoint(jointMap.getSpineJointName(SpineJointName.SPINE_PITCH)).setQ(0.0);
      robot.getOneDegreeOfFreedomJoint(jointMap.getSpineJointName(SpineJointName.SPINE_ROLL)).setQ(0.0);

      RigidBodyTransform rootJointPose = new RigidBodyTransform();
      rootJointPose.setRotation(new Quaternion(0.0, 0.0, Math.PI / 2.0));
      rootJointPose.setTranslation(0.0, 0.0, 0.335);
      robot.getRootJoint().setRotationAndTranslation(rootJointPose);
   }

   private static void setRobotToRollingB6Configuration(HumanoidFloatingRootJointRobot robot, DRCRobotJointMap jointMap)
   {
      RobotSide leftSide = RobotSide.LEFT;
      robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(leftSide, LegJointName.HIP_YAW)).setQ(1.1);
      robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(leftSide, LegJointName.HIP_ROLL)).setQ(0.4);
      robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(leftSide, LegJointName.HIP_PITCH)).setQ(0.175);
      robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(leftSide, LegJointName.KNEE_PITCH)).setQ(1.2);
      robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(leftSide, LegJointName.ANKLE_PITCH)).setQ(0.4);
      robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(leftSide, LegJointName.ANKLE_ROLL)).setQ(0.0);
      robot.getOneDegreeOfFreedomJoint(jointMap.getArmJointName(leftSide, ArmJointName.SHOULDER_PITCH)).setQ(-0.5 * Math.PI);
      robot.getOneDegreeOfFreedomJoint(jointMap.getArmJointName(leftSide, ArmJointName.SHOULDER_ROLL)).setQ(-0.5 * Math.PI);
      robot.getOneDegreeOfFreedomJoint(jointMap.getArmJointName(leftSide, ArmJointName.SHOULDER_YAW)).setQ(0.5 * Math.PI);
      robot.getOneDegreeOfFreedomJoint(jointMap.getArmJointName(leftSide, ArmJointName.ELBOW_PITCH)).setQ(-1.3);

      RobotSide rightSide = RobotSide.RIGHT;
      robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(rightSide, LegJointName.HIP_YAW)).setQ(-0.075);
      robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(rightSide, LegJointName.HIP_ROLL)).setQ(-0.13);
      robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(rightSide, LegJointName.HIP_PITCH)).setQ(-0.375);
      robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(rightSide, LegJointName.KNEE_PITCH)).setQ(1.1);
      robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(rightSide, LegJointName.ANKLE_PITCH)).setQ(0.82);
      robot.getOneDegreeOfFreedomJoint(jointMap.getArmJointName(rightSide, ArmJointName.SHOULDER_PITCH)).setQ(0.0);
      robot.getOneDegreeOfFreedomJoint(jointMap.getArmJointName(rightSide, ArmJointName.SHOULDER_ROLL)).setQ(1.519);
      robot.getOneDegreeOfFreedomJoint(jointMap.getArmJointName(rightSide, ArmJointName.SHOULDER_YAW)).setQ(0.4);
      robot.getOneDegreeOfFreedomJoint(jointMap.getArmJointName(rightSide, ArmJointName.ELBOW_PITCH)).setQ(2.0);

      robot.getOneDegreeOfFreedomJoint(jointMap.getSpineJointName(SpineJointName.SPINE_YAW)).setQ(-0.4);
      robot.getOneDegreeOfFreedomJoint(jointMap.getSpineJointName(SpineJointName.SPINE_PITCH)).setQ(0.0);
      robot.getOneDegreeOfFreedomJoint(jointMap.getSpineJointName(SpineJointName.SPINE_ROLL)).setQ(0.075);

      RigidBodyTransform rootJointPose = new RigidBodyTransform();
      rootJointPose.setRotation(new Quaternion(0.0, 0.0, Math.PI / 2.0));
      rootJointPose.setTranslation(0.0, 0.0, 0.32);
      robot.getRootJoint().setRotationAndTranslation(rootJointPose);
   }

   private static void addButt1ContactPoint(ValkyrieJointMap jointMap, ValkyrieMultiContactPointParameters contactPointParameters)
   {
      String parentJointName = jointMap.getPelvisName();
      String bodyName = parentJointName;
      String contactName = "ButtCP";
      contactPointParameters.addSingleContactPoint(parentJointName, bodyName, contactName, new Point3D(-0.06, 0.0, -0.335));
   }

   private static void addButt2ContactPoint(ValkyrieJointMap jointMap, ValkyrieMultiContactPointParameters contactPointParameters)
   {
      String parentJointName = jointMap.getPelvisName();
      String bodyName = parentJointName;
      String contactName = "ButtCP";
      contactPointParameters.addSingleContactPoint(parentJointName, bodyName, contactName, new Point3D(-0.105, 0.0, -0.29));
   }

   private static void addKneeUpperContactPoints(ValkyrieJointMap jointMap, ValkyrieMultiContactPointParameters contactPointParameters)
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         String parentJointName = jointMap.getLegJointName(robotSide, LegJointName.HIP_PITCH);
         String bodyName = parentJointName + "Link";
         String contactName = robotSide.getCamelCaseName() + "KneeCP";
         contactPointParameters.addSingleContactPoint(parentJointName, bodyName, contactName, new Point3D(0.06, robotSide.negateIfRightSide(0.07), -0.46));
      }
   }

   private static void addKneeOuterContactPoint(RobotSide robotSide, ValkyrieJointMap jointMap, ValkyrieMultiContactPointParameters contactPointParameters)
   {
      String parentJointName = jointMap.getLegJointName(robotSide, LegJointName.KNEE_PITCH);
      String bodyName = parentJointName + "Link";
      String contactName = robotSide.getCamelCaseName() + "KneeCP";
      contactPointParameters.addSingleContactPoint(parentJointName, bodyName, contactName, new Point3D(0.0, robotSide.negateIfRightSide(0.115), 0.0));
   }

   private static void addKneeLowerContactPoints(ValkyrieJointMap jointMap, ValkyrieMultiContactPointParameters contactPointParameters)
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         String parentJointName = jointMap.getLegJointName(robotSide, LegJointName.KNEE_PITCH);
         String bodyName = parentJointName + "Link";
         String contactName = robotSide.getCamelCaseName() + "KneeCP";
         contactPointParameters.addSingleContactPoint(parentJointName, bodyName, contactName, new Point3D(0.075, robotSide.negateIfRightSide(0.07), -0.01));
      }
   }

   private static void addToeFrontContactPoints(ValkyrieJointMap jointMap, ValkyrieMultiContactPointParameters contactPointParameters)
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         String parentJointName = jointMap.getJointBeforeFootName(robotSide);
         String bodyName = jointMap.getFootName(robotSide);
         String contactName = robotSide.getCamelCaseName() + "ToeCP";
         contactPointParameters.addSingleContactPoint(parentJointName, bodyName, contactName, new Point3D(0.19, 0.0, -0.05));
      }
   }

   private static void addToeTopContactPoints(ValkyrieJointMap jointMap, ValkyrieMultiContactPointParameters contactPointParameters)
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         String parentJointName = jointMap.getJointBeforeFootName(robotSide);
         String bodyName = jointMap.getFootName(robotSide);
         String contactName = robotSide.getCamelCaseName() + "ToeCP";
         contactPointParameters.addSingleContactPoint(parentJointName, bodyName, contactName, new Point3D(0.18, 0.0, -0.035));
      }
   }

   private static void addToeBottomContactPoints(ValkyrieJointMap jointMap, ValkyrieMultiContactPointParameters contactPointParameters)
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         String parentJointName = jointMap.getJointBeforeFootName(robotSide);
         String bodyName = jointMap.getFootName(robotSide);
         String contactName = robotSide.getCamelCaseName() + "ToeCP";
         contactPointParameters.addSingleContactPoint(parentJointName, bodyName, contactName, new Point3D(0.18, 0.0, -0.085));
      }
   }

   private static void addToeInnerBottomContactPoint(RobotSide robotSide, ValkyrieJointMap jointMap, ValkyrieMultiContactPointParameters contactPointParameters)
   {
      String parentJointName = jointMap.getJointBeforeFootName(robotSide);
      String bodyName = jointMap.getFootName(robotSide);
      String contactName = robotSide.getCamelCaseName() + "ToeCP";
      contactPointParameters.addSingleContactPoint(parentJointName, bodyName, contactName, new Point3D(0.18, robotSide.negateIfLeftSide(0.07), -0.085));
   }

   private static void addToeOuterContactPoint(RobotSide robotSide, ValkyrieJointMap jointMap, ValkyrieMultiContactPointParameters contactPointParameters)
   {
      String parentJointName = jointMap.getJointBeforeFootName(robotSide);
      String bodyName = jointMap.getFootName(robotSide);
      String contactName = robotSide.getCamelCaseName() + "ToeCP";
      contactPointParameters.addSingleContactPoint(parentJointName, bodyName, contactName, new Point3D(0.165, robotSide.negateIfRightSide(0.075), -0.06));
   }

   private static void addHeelBottomContactPoints(ValkyrieJointMap jointMap, ValkyrieMultiContactPointParameters contactPointParameters)
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         addHeelBottomContactPoint(robotSide, jointMap, contactPointParameters);
      }
   }

   private static void addHeelBottomContactPoint(RobotSide robotSide, ValkyrieJointMap jointMap, ValkyrieMultiContactPointParameters contactPointParameters)
   {
      String parentJointName = jointMap.getJointBeforeFootName(robotSide);
      String bodyName = jointMap.getFootName(robotSide);
      String contactName = robotSide.getCamelCaseName() + "HeelCP";
      contactPointParameters.addSingleContactPoint(parentJointName, bodyName, contactName, new Point3D(-0.085, 0.0, -0.085));
   }

   private static void addHeelInnerBottomContactPoint(RobotSide robotSide, ValkyrieJointMap jointMap,
                                                      ValkyrieMultiContactPointParameters contactPointParameters)
   {
      String parentJointName = jointMap.getJointBeforeFootName(robotSide);
      String bodyName = jointMap.getFootName(robotSide);
      String contactName = robotSide.getCamelCaseName() + "HeelCP";
      contactPointParameters.addSingleContactPoint(parentJointName, bodyName, contactName, new Point3D(-0.085, robotSide.negateIfLeftSide(0.07), -0.085));
   }

   private static void addFootInnerEdgeContactPoints(RobotSide robotSide, ValkyrieJointMap jointMap, ValkyrieMultiContactPointParameters contactPointParameters)
   {
      String parentJointName = jointMap.getJointBeforeFootName(robotSide);
      String bodyName = jointMap.getFootName(robotSide);
      String contactName = robotSide.getCamelCaseName() + "FootCP";

      RigidBodyTransform contactFramePose = new RigidBodyTransform(new YawPitchRoll(0.0, 0.0, 0.0), new Point3D(0.0, 0.0, -0.085));
      List<Point2D> contactPoints = new ArrayList<>();
      contactPoints.add(new Point2D(0.18, robotSide.negateIfLeftSide(0.07)));
      contactPoints.add(new Point2D(-0.085, robotSide.negateIfLeftSide(0.07)));
      contactPointParameters.addPlaneContact(parentJointName, bodyName, contactName, contactFramePose, contactPoints);
   }

   private static void addFootOuterEdgeContactPoints(RobotSide robotSide, ValkyrieJointMap jointMap, ValkyrieMultiContactPointParameters contactPointParameters)
   {
      String parentJointName = jointMap.getJointBeforeFootName(robotSide);
      String bodyName = jointMap.getFootName(robotSide);
      String contactName = robotSide.getCamelCaseName() + "FootCP";

      RigidBodyTransform contactFramePose = new RigidBodyTransform(new YawPitchRoll(0.0, 0.0, 0.0), new Point3D(0.0, 0.0, -0.085));
      List<Point2D> contactPoints = new ArrayList<>();
      contactPoints.add(new Point2D(0.18, robotSide.negateIfRightSide(0.07)));
      contactPoints.add(new Point2D(-0.085, robotSide.negateIfRightSide(0.07)));
      contactPointParameters.addPlaneContact(parentJointName, bodyName, contactName, contactFramePose, contactPoints);
   }

   private static void addShoulderPitchBackContactPoint(RobotSide robotSide, ValkyrieJointMap jointMap,
                                                        ValkyrieMultiContactPointParameters contactPointParameters)
   {
      String parentJointName = jointMap.getArmJointName(robotSide, ArmJointName.SHOULDER_PITCH);
      String bodyName = parentJointName + "Link";
      String contactName = robotSide.getCamelCaseName() + "ShoulderCP";
      contactPointParameters.addSingleContactPoint(parentJointName, bodyName, contactName, new Point3D(-0.1, robotSide.negateIfRightSide(0.3), 0.0));
   }

   private static void addShoulderPitchOuterContactPoint(RobotSide robotSide, ValkyrieJointMap jointMap,
                                                        ValkyrieMultiContactPointParameters contactPointParameters)
   {
      String parentJointName = jointMap.getArmJointName(robotSide, ArmJointName.SHOULDER_PITCH);
      String bodyName = parentJointName + "Link";
      String contactName = robotSide.getCamelCaseName() + "ShoulderCP";
      contactPointParameters.addSingleContactPoint(parentJointName, bodyName, contactName, new Point3D(0.0, robotSide.negateIfRightSide(0.31), 0.0));
   }

   private static void addElbowContactPoints(ValkyrieJointMap jointMap, ValkyrieMultiContactPointParameters contactPointParameters)
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         addElbowContactPoint(robotSide, jointMap, contactPointParameters);
      }
   }

   private static void addElbowContactPoint(RobotSide robotSide, ValkyrieJointMap jointMap, ValkyrieMultiContactPointParameters contactPointParameters)
   {
      String parentJointName = jointMap.getArmJointName(robotSide, ArmJointName.ELBOW_PITCH);
      String bodyName = parentJointName + "Link";
      String contactName = robotSide.getCamelCaseName() + "ElbowCP";
      contactPointParameters.addSingleContactPoint(parentJointName, bodyName, contactName, new Point3D(-0.11, robotSide.negateIfRightSide(0.05), 0.0));
   }

   private static void addElbowOuterContactPoint(RobotSide robotSide, ValkyrieJointMap jointMap, ValkyrieMultiContactPointParameters contactPointParameters)
   {
      String parentJointName = jointMap.getArmJointName(robotSide, ArmJointName.ELBOW_PITCH);
      String bodyName = parentJointName + "Link";
      String contactName = robotSide.getCamelCaseName() + "ElbowCP";
      contactPointParameters.addSingleContactPoint(parentJointName, bodyName, contactName, new Point3D(0.0, 0.0, 0.07));
   }

   private static void addHandFist1ContactPoints(ValkyrieJointMap jointMap, ValkyrieMultiContactPointParameters contactPointParameters)
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         addHandFist1ContactPoint(robotSide, jointMap, contactPointParameters);
      }
   }

   private static void addHandFist1ContactPoint(RobotSide robotSide, ValkyrieJointMap jointMap, ValkyrieMultiContactPointParameters contactPointParameters)
   {
      String parentJointName = jointMap.getJointBeforeHandName(robotSide);
      String bodyName = jointMap.getHandName(robotSide);
      String contactName = robotSide.getCamelCaseName() + "FistCP";
      contactPointParameters.addSingleContactPoint(parentJointName, bodyName, contactName, new Point3D(-0.025, robotSide.negateIfRightSide(0.10), -0.01));
   }

   private static void addBackpackFourContactPoints(ValkyrieJointMap jointMap, ValkyrieMultiContactPointParameters contactPointParameters)
   {
      String parentJointName = jointMap.getNameOfJointBeforeChest();
      String bodyName = jointMap.getChestName();
      String contactName = "BackpackCP";

      RigidBodyTransform contactFramePose = new RigidBodyTransform(new YawPitchRoll(0.0, 0.5 * Math.PI, 0.0), new Point3D(-0.29, 0.0, 0.25));
      List<Point2D> contactPoints = new ArrayList<>();
      contactPoints.add(new Point2D(0.15, 0.1));
      contactPoints.add(new Point2D(0.15, -0.1));
      contactPoints.add(new Point2D(-0.15, 0.1));
      contactPoints.add(new Point2D(-0.15, -0.1));
      contactPointParameters.addPlaneContact(parentJointName, bodyName, contactName, contactFramePose, contactPoints);
   }

   private static void addBackpackSideEdgeContactPoints(RobotSide robotSide, ValkyrieJointMap jointMap,
                                                        ValkyrieMultiContactPointParameters contactPointParameters)
   {
      String parentJointName = jointMap.getNameOfJointBeforeChest();
      String bodyName = jointMap.getChestName();
      String contactName = "BackpackCP";

      RigidBodyTransform contactFramePose = new RigidBodyTransform(new YawPitchRoll(0.0, 0.5 * Math.PI, 0.0), new Point3D(-0.29, 0.0, 0.25));
      List<Point2D> contactPoints = new ArrayList<>();
      contactPoints.add(new Point2D(0.15, robotSide.negateIfRightSide(0.1)));
      contactPoints.add(new Point2D(-0.15, robotSide.negateIfRightSide(0.1)));
      contactPointParameters.addPlaneContact(parentJointName, bodyName, contactName, contactFramePose, contactPoints);
   }
}
