package us.ihmc.valkyrie.torquespeedcurve;

import static org.junit.jupiter.api.Assertions.assertTrue;

import java.awt.Dimension;
import java.awt.Graphics;
import java.awt.RenderingHints;
import java.awt.geom.AffineTransform;
import java.awt.image.AffineTransformOp;
import java.awt.image.BufferedImage;
import java.io.File;
import java.io.IOException;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.function.Function;
import java.util.stream.Collectors;

import javax.imageio.ImageIO;
import javax.swing.JFrame;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.MethodOrderer;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.TestInfo;
import org.junit.jupiter.api.TestMethodOrder;

import us.ihmc.avatar.SimulatedLowLevelOutputWriter;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.initialSetup.DRCSCSInitialSetup;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ListOfPointsContactablePlaneBody;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.SimpleContactPointPlaneBody;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;
import us.ihmc.graphicsDescription.plotting.artifact.TextArtifact;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.jMonkeyEngineToolkit.camera.CameraConfiguration;
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
import us.ihmc.simulationconstructionset.OneDegreeOfFreedomJoint;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.ViewportConfiguration;
import us.ihmc.simulationconstructionset.gui.EventDispatchThreadHelper;
import us.ihmc.simulationconstructionset.gui.SimulationOverheadPlotter;
import us.ihmc.simulationconstructionset.gui.ViewportAdapterAndCameraControllerHolder;
import us.ihmc.simulationconstructionset.gui.ViewportWindow;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.valkyrie.ValkyrieRobotModel;
import us.ihmc.valkyrie.configuration.ValkyrieRobotVersion;
import us.ihmc.valkyrie.parameters.ValkyrieJointMap;
import us.ihmc.wholeBodyController.DRCRobotJointMap;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoFrameConvexPolygon2D;

@TestMethodOrder(MethodOrderer.Alphanumeric.class)
public class ValkyrieMultiContactStaticPoseEndToEndTest
{
   private static final double halfPi = Math.PI / 2.0;
   private static final double simulationDT = 2.0e-4;

   private static final File outputFolder = Paths.get("D:", "DataAndVideos", "ValkyrieStaticConfigurations").toFile();

   private ValkyrieRobotModel robotModel;
   private ValkyrieJointMap jointMap;
   private ValkyrieMultiContactPointParameters contactPointParameters;
   private HumanoidFloatingRootJointRobot robot;
   private SimulationConstructionSet scs;
   private ViewportWindow viewportWindow;
   private BlockingSimulationRunner blockingSimulationRunner;
   private SimulationOverheadPlotter plotter;
   private MultiContactStaticController controller;
   private JFrame plotterWindow;

   @BeforeEach
   private void setup()
   {
      robotModel = new ValkyrieRobotModel(RobotTarget.SCS, ValkyrieRobotVersion.FINGERLESS);
      jointMap = robotModel.getJointMap();
      contactPointParameters = new ValkyrieMultiContactPointParameters(jointMap, robotModel.getRobotPhysicalProperties());
   }

   @AfterEach
   private void clean()
   {
      robotModel = null;
      jointMap = null;
      contactPointParameters = null;

      controller = null;
      plotterWindow.dispose();
      plotterWindow = null;
      plotter = null;
      blockingSimulationRunner = null;
      viewportWindow = null;
      scs.closeAndDispose();
      scs = null;
      robot = null;
   }

   @Test
   public void testCrawl1(TestInfo info) throws Exception
   {
      addKneeUpperContactPoints(jointMap, contactPointParameters);
      addToeFrontContactPoints(jointMap, contactPointParameters);
      addElbowContactPoints(jointMap, contactPointParameters);
      setupSimulationAndController();

      setLegJointQs(robot, jointMap, 0.5, 0.0, -1.0, 1.2, 0.2, 0.0);
      setArmJointQs(robot, jointMap, -2.0, -1.2, 0.0, -1.2);
      setRootJointPose(robot, 0.0, 0.0, 0.39, 0.0, halfPi, 0.0);
      startSim(info);
   }

   @Test
   public void testCrawl2(TestInfo info) throws Exception
   {
      addKneeLowerContactPoints(jointMap, contactPointParameters);
      addToeFrontContactPoints(jointMap, contactPointParameters);
      addHandFist1ContactPoints(jointMap, contactPointParameters);
      setupSimulationAndController();

      setLegJointQs(robot, jointMap, 0.0, 0.0, -1.25, 1.75, -0.8, 0.0);
      setArmJointQs(robot, jointMap, -1.5, -0.65, 1.5, -1.4);
      setSpineJointQs(robot, jointMap, 0.0, 0.1, 0.0);
      setRootJointPose(robot, 0.0, 0.0, 0.56, 0.0, 0.4, 0.0, 0.5);
      startSim(info);
   }

   @Test
   public void testKneel1(TestInfo info) throws Exception
   {
      addKneeLowerContactPoints(jointMap, contactPointParameters);
      addToeTopContactPoints(jointMap, contactPointParameters);
      setupSimulationAndController();

      setLegJointQs(robot, jointMap, 0.0, 0.0, -0.3, 1.9, 0.8, 0.0);
      setArmJointQs(robot, jointMap, 0.7, -1.2, 0.0, -1.7);
      setSpineJointQs(robot, jointMap, 0.0, 0.0, 0.0);
      setRootJointPose(robot, 0.0, 0.0, 0.74, 0.0, 0.0, 0.0);
      startSim(info);
   }

   @Test
   public void testKneel2(TestInfo info) throws Exception
   {
      addKneeLowerContactPoints(jointMap, contactPointParameters);
      addToeFrontContactPoints(jointMap, contactPointParameters);
      setupSimulationAndController();

      setLegJointQs(robot, jointMap, 0.0, 0.0, 0.75, 2.05, -0.8, 0.0);
      setArmJointQs(robot, jointMap, 0.7, -1.2, 0.0, -1.7);
      setSpineJointQs(robot, jointMap, 0.0, 0.4, 0.0);
      setRootJointPose(robot, 0.0, 0.0, 0.64, 0.0, -0.4, 0.0, 0.8);
      startSim(info);
   }

   @Test
   public void testSit1(TestInfo info) throws Exception
   {
      addButt1ContactPoint(jointMap, contactPointParameters);
      addHeelBottomContactPoints(jointMap, contactPointParameters);
      addHandFist1ContactPoints(jointMap, contactPointParameters);
      setupSimulationAndController();

      setLegJointQs(robot, jointMap, 0.0, 0.0, -1.9, 1.525, 0.775, 0.0);
      setArmJointQs(robot, jointMap, 1.1, -1.0, 1.1, -0.9);
      setSpineJointQs(robot, jointMap, 0.0, -0.1, 0.0);
      setRootJointPose(robot, 0.0, 0.0, 0.33, 0.0, -0.2, 0.0, 1.0);
      startSim(info);
   }

   @Test
   public void testSit2(TestInfo info) throws Exception
   {
      addButt2ContactPoint(jointMap, contactPointParameters);
      addHeelBottomContactPoints(jointMap, contactPointParameters);
      addHandFist1ContactPoints(jointMap, contactPointParameters);
      setupSimulationAndController();

      setLegJointQs(robot, jointMap, 0.0, 0.0, -1.2, 1.525, 0.775, 0.0);
      setArmJointQs(robot, jointMap, 1.4, 0.2, 1.5, -1.8);
      setSpineJointQs(robot, jointMap, 0.0, -0.1, 0.0);
      setRootJointPose(robot, 0.0, 0.0, 0.23, 0.0, -0.6, 0.0, 1.0);
      startSim(info);
   }

   @Test
   public void testSit3(TestInfo info) throws Exception
   {
      addButt2ContactPoint(jointMap, contactPointParameters);
      addHeelBottomContactPoints(jointMap, contactPointParameters);
      addHandFist1ContactPoints(jointMap, contactPointParameters);
      setupSimulationAndController();

      setLegJointQs(robot, jointMap, 0.0, 0.0, -0.875, 1.425, 0.8, 0.0);
      setArmJointQs(robot, jointMap, -0.5, 1.266, -3.1, -2.0);
      setSpineJointQs(robot, jointMap, 0.0, 0.4, 0.0);
      setRootJointPose(robot, 0.0, 0.0, 0.17, 0.0, -0.8, 0.0, 1.0);
      startSim(info);
   }

   @Test
   public void testRollingA1(TestInfo info) throws Exception
   {
      addKneeOuterContactPoint(RobotSide.RIGHT, jointMap, contactPointParameters);
      addToeOuterContactPoint(RobotSide.RIGHT, jointMap, contactPointParameters);
      addElbowContactPoint(RobotSide.RIGHT, jointMap, contactPointParameters);
      addHandFist1ContactPoint(RobotSide.LEFT, jointMap, contactPointParameters);
      addHeelBottomContactPoint(RobotSide.LEFT, jointMap, contactPointParameters);
      setupSimulationAndController();

      setLegJointQs(RobotSide.LEFT, robot, jointMap, 0.8, 0.2, -0.33, 1.6, 0.3, 0.0);
      setArmJointQs(RobotSide.LEFT, robot, jointMap, 1.4, 0.26, 1.5, -1.8);
      setLegJointQs(RobotSide.RIGHT, robot, jointMap, -0.8, -0.1, -0.6, 1.4, 0.0, 0.0);
      setArmJointQs(RobotSide.RIGHT, robot, jointMap, 1.7, 1.3, 0.0, 1.95);
      setSpineJointQs(robot, jointMap, 0.85, 0.34, 0.0);
      setRootJointPose(robot, 0.0, 0.0, 0.26, halfPi, -0.785, halfPi);
      startSim(info);
   }

   @Test
   public void testRollingA2(TestInfo info) throws Exception
   {
      addKneeOuterContactPoint(RobotSide.RIGHT, jointMap, contactPointParameters);
      addToeOuterContactPoint(RobotSide.RIGHT, jointMap, contactPointParameters);
      addElbowContactPoint(RobotSide.RIGHT, jointMap, contactPointParameters);
      addHandFist1ContactPoint(RobotSide.LEFT, jointMap, contactPointParameters);
      addHeelBottomContactPoint(RobotSide.LEFT, jointMap, contactPointParameters);
      setupSimulationAndController();

      setLegJointQs(RobotSide.LEFT, robot, jointMap, 0.8, 0.2, -0.33, 1.6, 0.3, 0.0);
      setArmJointQs(RobotSide.LEFT, robot, jointMap, -0.15, 1.266, -3.1, -2.0);
      setLegJointQs(RobotSide.RIGHT, robot, jointMap, -0.8, -0.1, -0.6, 1.4, 0.0, 0.0);
      setArmJointQs(RobotSide.RIGHT, robot, jointMap, 1.7, 1.3, 0.0, 1.95);
      setSpineJointQs(robot, jointMap, 0.85, 0.34, 0.0);
      setRootJointPose(robot, 0.0, 0.0, 0.26, halfPi, -0.785, halfPi);
      startSim(info);
   }

   @Test
   public void testRollingA3(TestInfo info) throws Exception
   {
      addKneeOuterContactPoint(RobotSide.RIGHT, jointMap, contactPointParameters);
      addToeOuterContactPoint(RobotSide.RIGHT, jointMap, contactPointParameters);
      addElbowContactPoint(RobotSide.RIGHT, jointMap, contactPointParameters);
      addHandFist1ContactPoint(RobotSide.LEFT, jointMap, contactPointParameters);
      addHeelBottomContactPoint(RobotSide.LEFT, jointMap, contactPointParameters);
      setupSimulationAndController();

      setLegJointQs(RobotSide.LEFT, robot, jointMap, 0.8, 0.2, -0.45, 1.4, 0.425, -0.2);
      setArmJointQs(RobotSide.LEFT, robot, jointMap, 1.7, -1.5, 0.0, -1.2);
      setLegJointQs(RobotSide.RIGHT, robot, jointMap, -0.55, 0.13, -0.6, 1.4, 0.0, 0.0);
      setArmJointQs(RobotSide.RIGHT, robot, jointMap, 1.6, 1.3, 0.0, 2.0);
      setSpineJointQs(robot, jointMap, 0.60, 0.36, 0.0);
      setRootJointPose(robot, 0.0, 0.0, 0.31, halfPi, -0.585, 1.3);
      startSim(info);
   }

   @Test
   public void testRollingA4(TestInfo info) throws Exception
   {
      addKneeOuterContactPoint(RobotSide.RIGHT, jointMap, contactPointParameters);
      addToeOuterContactPoint(RobotSide.RIGHT, jointMap, contactPointParameters);
      addElbowContactPoint(RobotSide.RIGHT, jointMap, contactPointParameters);
      addHeelBottomContactPoint(RobotSide.LEFT, jointMap, contactPointParameters);
      setupSimulationAndController();

      setLegJointQs(RobotSide.LEFT, robot, jointMap, 0.975, 0.5, -0.45, 1.4, 0.425, -0.3);
      setArmJointQs(RobotSide.LEFT, robot, jointMap, 1.7, -1.5, 0.0, -1.2);
      setLegJointQs(RobotSide.RIGHT, robot, jointMap, -0.41, 0.25, -0.6, 1.4, 0.0, 0.0);
      setArmJointQs(RobotSide.RIGHT, robot, jointMap, 1.2, 1.1, 0.0, 2.0);
      setSpineJointQs(robot, jointMap, 0.30, 0.36, -0.2);
      setRootJointPose(robot, 0.0, 0.0, 0.34, halfPi, -0.385, 1.2);
      startSim(info);
   }

   @Test
   public void testRollingA5(TestInfo info) throws Exception
   {
      addKneeOuterContactPoint(RobotSide.RIGHT, jointMap, contactPointParameters);
      addToeOuterContactPoint(RobotSide.RIGHT, jointMap, contactPointParameters);
      addElbowContactPoint(RobotSide.RIGHT, jointMap, contactPointParameters);
      addHeelInnerBottomContactPoint(RobotSide.LEFT, jointMap, contactPointParameters);
      setupSimulationAndController();

      setLegJointQs(RobotSide.LEFT, robot, jointMap, 1.075, 0.5, -0.25, 1.4, 0.225, -0.3);
      setArmJointQs(RobotSide.LEFT, robot, jointMap, 1.7, -1.5, 0.0, -1.2);
      setLegJointQs(RobotSide.RIGHT, robot, jointMap, -0.01, 0.3, -0.6, 1.4, 0.0, 0.0);
      setArmJointQs(RobotSide.RIGHT, robot, jointMap, 1.2, 0.9, 0.3, 2.0);
      setSpineJointQs(robot, jointMap, 0.10, 0.36, -0.2);
      setRootJointPose(robot, 0.0, 0.0, 0.34, halfPi, 0.0, 1.2);
      startSim(info);
   }

   @Test
   public void testRollingA6(TestInfo info) throws Exception
   {
      addKneeOuterContactPoint(RobotSide.RIGHT, jointMap, contactPointParameters);
      addToeOuterContactPoint(RobotSide.RIGHT, jointMap, contactPointParameters);
      addElbowContactPoint(RobotSide.RIGHT, jointMap, contactPointParameters);
      setupSimulationAndController();

      setLegJointQs(RobotSide.LEFT, robot, jointMap, 0.875, 0.5, -0.55, 1.4, 0.225, -0.3);
      setArmJointQs(RobotSide.LEFT, robot, jointMap, 1.7, -1.5, 0.0, -1.2);
      setLegJointQs(RobotSide.RIGHT, robot, jointMap, -0.01, 0.3, -0.6, 1.4, 0.0, 0.0);
      setArmJointQs(RobotSide.RIGHT, robot, jointMap, 1.2, 0.9, 0.3, 2.0);
      setSpineJointQs(robot, jointMap, 0.10, 0.36, -0.2);
      setRootJointPose(robot, 0.0, 0.0, 0.34, halfPi, 0.0, 1.2);
      startSim(info);
   }

   @Test
   public void testRollingA7(TestInfo info) throws Exception
   {
      addKneeOuterContactPoint(RobotSide.RIGHT, jointMap, contactPointParameters);
      addToeOuterContactPoint(RobotSide.RIGHT, jointMap, contactPointParameters);
      addElbowContactPoint(RobotSide.RIGHT, jointMap, contactPointParameters);
      addToeInnerBottomContactPoint(RobotSide.LEFT, jointMap, contactPointParameters);
      setupSimulationAndController();

      setLegJointQs(RobotSide.LEFT, robot, jointMap, -0.414, 0.35, -0.95, 0.4, 0.225, -0.3);
      setArmJointQs(RobotSide.LEFT, robot, jointMap, 1.7, -1.5, 0.0, -1.2);
      setLegJointQs(RobotSide.RIGHT, robot, jointMap, -0.01, 0.3, -0.6, 1.4, 0.0, 0.0);
      setArmJointQs(RobotSide.RIGHT, robot, jointMap, 1.2, 0.9, 0.3, 2.0);
      setSpineJointQs(robot, jointMap, 0.10, 0.36, -0.2);
      setRootJointPose(robot, 0.0, 0.0, 0.34, halfPi, 0.0, 1.2);
      startSim(info);
   }

   @Test
   public void testLieDown1(TestInfo info) throws Exception
   {
      addHeelBottomContactPoints(jointMap, contactPointParameters);
      addBackpackFourContactPoints(jointMap, contactPointParameters);
      setupSimulationAndController();

      setLegJointQs(robot, jointMap, 0.0, 0.0, -0.325, 1.1, 0.8, 0.0);
      setArmJointQs(robot, jointMap, 0.0, -1.5, 0.0, -2.0);
      setSpineJointQs(robot, jointMap, 0.0, 0.0, 0.0);
      setRootJointPose(robot, 0.0, 0.0, 0.245, 0.707, 0.0, 0.707, 0.0);
      startSim(info);
   }

   @Test
   public void testLieDown2(TestInfo info) throws Exception
   {
      addHeelBottomContactPoints(jointMap, contactPointParameters);
      addBackpackFourContactPoints(jointMap, contactPointParameters);
      addHandFist1ContactPoint(RobotSide.LEFT, jointMap, contactPointParameters);
      setupSimulationAndController();

      setLegJointQs(robot, jointMap, 0.0, 0.0, -0.325, 1.1, 0.8, 0.0);
      setArmJointQs(RobotSide.LEFT, robot, jointMap, halfPi, 0.4, halfPi, -1.75);
      setArmJointQs(RobotSide.RIGHT, robot, jointMap, 0.0, 1.5, 0.0, 2.0);
      setSpineJointQs(robot, jointMap, 0.0, 0.0, 0.0);
      setRootJointPose(robot, 0.0, 0.0, 0.245, 0.707, 0.0, 0.707, 0.0);
      startSim(info);
   }

   @Test
   public void testRollingB1(TestInfo info) throws Exception
   {
      addHandFist1ContactPoint(RobotSide.LEFT, jointMap, contactPointParameters);
      addHeelBottomContactPoint(RobotSide.LEFT, jointMap, contactPointParameters);
      addFootOuterEdgeContactPoints(RobotSide.RIGHT, jointMap, contactPointParameters);
      addBackpackSideEdgeContactPoints(RobotSide.RIGHT, jointMap, contactPointParameters);
      setupSimulationAndController();

      setLegJointQs(RobotSide.LEFT, robot, jointMap, 0.5, 0.1, -0.25, 1.2, 0.625, 0.0);
      setArmJointQs(RobotSide.LEFT, robot, jointMap, halfPi, -0.5, halfPi, -1.45);
      setLegJointQs(RobotSide.RIGHT, robot, jointMap, -0.2, 0.0, -0.375, 1.1, 0.82, 0.0);
      setArmJointQs(RobotSide.RIGHT, robot, jointMap, 0.0, 1.5, 0.0, 2.0);
      setSpineJointQs(robot, jointMap, 0.0, 0.0, 0.0);
      setRootJointPose(robot, 0.0, 0.0, 0.275, halfPi, -1.07, halfPi);
      startSim(info);
   }

   @Test
   public void testRollingB2(TestInfo info) throws Exception
   {
      addHandFist1ContactPoint(RobotSide.LEFT, jointMap, contactPointParameters);
      addHeelBottomContactPoint(RobotSide.LEFT, jointMap, contactPointParameters);
      addFootOuterEdgeContactPoints(RobotSide.RIGHT, jointMap, contactPointParameters);
      addBackpackSideEdgeContactPoints(RobotSide.RIGHT, jointMap, contactPointParameters);
      addShoulderPitchBackContactPoint(RobotSide.RIGHT, jointMap, contactPointParameters);
      setupSimulationAndController();

      setLegJointQs(RobotSide.LEFT, robot, jointMap, 0.5, 0.1, -0.22, 1.2, 0.625, 0.0);
      setArmJointQs(RobotSide.LEFT, robot, jointMap, halfPi, -0.75, halfPi, -1.35);
      setLegJointQs(RobotSide.RIGHT, robot, jointMap, -0.2, 0.0, -0.375, 1.1, 0.82, 0.0);
      setArmJointQs(RobotSide.RIGHT, robot, jointMap, 0.0, 1.5, 0.0, 2.0);
      setSpineJointQs(robot, jointMap, 0.0, 0.0, 0.0);
      setRootJointPose(robot, 0.0, 0.0, 0.26, halfPi, -0.9, halfPi);
      startSim(info);
   }

   @Test
   public void testRollingB3(TestInfo info) throws Exception
   {
      addHandFist1ContactPoint(RobotSide.LEFT, jointMap, contactPointParameters);
      addFootInnerEdgeContactPoints(RobotSide.LEFT, jointMap, contactPointParameters);
      addKneeOuterContactPoint(RobotSide.RIGHT, jointMap, contactPointParameters);
      addToeOuterContactPoint(RobotSide.RIGHT, jointMap, contactPointParameters);
      addShoulderPitchBackContactPoint(RobotSide.RIGHT, jointMap, contactPointParameters);
      setupSimulationAndController();

      setLegJointQs(RobotSide.LEFT, robot, jointMap, 0.7, 0.3, -0.05, 1.2, 0.5, 0.0);
      setArmJointQs(RobotSide.LEFT, robot, jointMap, halfPi, -1.25, halfPi, -1.35);
      setLegJointQs(RobotSide.RIGHT, robot, jointMap, -0.65, -0.12, -0.375, 1.1, 0.82, 0.0);
      setArmJointQs(RobotSide.RIGHT, robot, jointMap, 0.0, 1.5, 0.0, 2.0);
      setSpineJointQs(robot, jointMap, 0.0, 0.0, 0.0);
      setRootJointPose(robot, 0.0, 0.0, 0.3, halfPi, -0.6, halfPi);
      startSim(info);
   }

   @Test
   public void testRollingB4(TestInfo info) throws Exception
   {
      addFootInnerEdgeContactPoints(RobotSide.LEFT, jointMap, contactPointParameters);
      addKneeOuterContactPoint(RobotSide.RIGHT, jointMap, contactPointParameters);
      addToeOuterContactPoint(RobotSide.RIGHT, jointMap, contactPointParameters);
      addShoulderPitchBackContactPoint(RobotSide.RIGHT, jointMap, contactPointParameters);
      setupSimulationAndController();

      setLegJointQs(RobotSide.LEFT, robot, jointMap, 1.0, 0.4, 0.025, 1.2, 0.45, 0.0);
      setArmJointQs(RobotSide.LEFT, robot, jointMap, halfPi, -1.25, halfPi, -1.35);
      setLegJointQs(RobotSide.RIGHT, robot, jointMap, -0.35, -0.13, -0.375, 1.1, 0.82, 0.0);
      setArmJointQs(RobotSide.RIGHT, robot, jointMap, 0.0, 1.519, -0.3, 2.0);
      setSpineJointQs(robot, jointMap, 0.0, 0.0, 0.0);
      setRootJointPose(robot, 0.0, 0.0, 0.32, halfPi, -0.3, halfPi);
      startSim(info);
   }

   @Test
   public void testRollingB5(TestInfo info) throws Exception
   {
      addFootInnerEdgeContactPoints(RobotSide.LEFT, jointMap, contactPointParameters);
      addKneeOuterContactPoint(RobotSide.RIGHT, jointMap, contactPointParameters);
      addToeOuterContactPoint(RobotSide.RIGHT, jointMap, contactPointParameters);
      addShoulderPitchOuterContactPoint(RobotSide.RIGHT, jointMap, contactPointParameters);
      setupSimulationAndController();

      setLegJointQs(RobotSide.LEFT, robot, jointMap, 1.1, 0.4, 0.2, 1.2, 0.4, 0.0);
      setArmJointQs(RobotSide.LEFT, robot, jointMap, -halfPi, -1.519, halfPi, -0.7);
      setLegJointQs(RobotSide.RIGHT, robot, jointMap, -0.075, -0.15, -0.375, 1.1, 0.82, 0.0);
      setArmJointQs(RobotSide.RIGHT, robot, jointMap, 0.0, 1.519, 0.0, 2.0);
      setSpineJointQs(robot, jointMap, 0.0, 0.0, 0.0);
      setRootJointPose(robot, 0.0, 0.0, 0.335, halfPi, 0.0, halfPi);
      startSim(info);
   }

   @Test
   public void testRollingB6(TestInfo info) throws Exception
   {
      addHandFist1ContactPoint(RobotSide.LEFT, jointMap, contactPointParameters);
      addFootInnerEdgeContactPoints(RobotSide.LEFT, jointMap, contactPointParameters);
      addKneeOuterContactPoint(RobotSide.RIGHT, jointMap, contactPointParameters);
      addToeOuterContactPoint(RobotSide.RIGHT, jointMap, contactPointParameters);
      addShoulderPitchOuterContactPoint(RobotSide.RIGHT, jointMap, contactPointParameters);
      setupSimulationAndController();

      setLegJointQs(RobotSide.LEFT, robot, jointMap, 1.1, 0.4, 0.175, 1.2, 0.4, 0.0);
      setArmJointQs(RobotSide.LEFT, robot, jointMap, -halfPi, -1.519, halfPi, -1.3);
      setLegJointQs(RobotSide.RIGHT, robot, jointMap, -0.075, -0.13, -0.375, 1.1, 0.82, 0.0);
      setArmJointQs(RobotSide.RIGHT, robot, jointMap, 0.0, 1.519, 0.4, 2.0);
      setSpineJointQs(robot, jointMap, -0.4, 0.0, 0.075);
      setRootJointPose(robot, 0.0, 0.0, 0.32, halfPi, 0.0, halfPi);
      startSim(info);
   }

   @Test
   public void testRollingB7(TestInfo info) throws Exception
   {
      addHandFist1ContactPoint(RobotSide.LEFT, jointMap, contactPointParameters);
      addToeInnerBottomContactPoint(RobotSide.LEFT, jointMap, contactPointParameters);
      addKneeOuterContactPoint(RobotSide.RIGHT, jointMap, contactPointParameters);
      addToeOuterContactPoint(RobotSide.RIGHT, jointMap, contactPointParameters);
      addShoulderPitchFrontContactPoint(RobotSide.RIGHT, jointMap, contactPointParameters);
      setupSimulationAndController();

      setLegJointQs(RobotSide.LEFT, robot, jointMap, -0.4, -0.2, -0.825, 0.5, 0.4, 0.0);
      setArmJointQs(RobotSide.LEFT, robot, jointMap, -halfPi, -1.519, halfPi, -1.3);
      setLegJointQs(RobotSide.RIGHT, robot, jointMap, -0.075, -0.13, -0.375, 1.1, 0.82, 0.0);
      setArmJointQs(RobotSide.RIGHT, robot, jointMap, 0.0, 1.519, 0.4, 2.0);
      setSpineJointQs(robot, jointMap, -0.4, 0.0, 0.075);
      setRootJointPose(robot, 0.0, 0.0, 0.32, halfPi, 0.0, halfPi);
      startSim(info);
   }

   @Test
   public void testRollingB8(TestInfo info) throws Exception
   {
      addHandFist1ContactPoint(RobotSide.LEFT, jointMap, contactPointParameters);
      addToeInnerBottomContactPoint(RobotSide.LEFT, jointMap, contactPointParameters);
      addKneeOuterContactPoint(RobotSide.RIGHT, jointMap, contactPointParameters);
      addToeOuterContactPoint(RobotSide.RIGHT, jointMap, contactPointParameters);
      addShoulderPitchFrontContactPoint(RobotSide.RIGHT, jointMap, contactPointParameters);
      setupSimulationAndController();

      setLegJointQs(RobotSide.LEFT, robot, jointMap, -0.4, 0.0, -1.425, 1.3, -0.8, 0.0);
      setArmJointQs(RobotSide.LEFT, robot, jointMap, -halfPi, -1.519, 1.3, -1.3);
      setLegJointQs(RobotSide.RIGHT, robot, jointMap, 0.05, -0.13, -0.475, 0.8, 0.82, 0.0);
      setArmJointQs(RobotSide.RIGHT, robot, jointMap, 0.0, 1.45, 0.45, 2.0);
      setSpineJointQs(robot, jointMap, 0.0, 0.0, 0.1);
      setRootJointPose(robot, 0.0, 0.0, 0.35, halfPi, 0.3, halfPi);
      startSim(info);
   }

   @Test
   public void testRollingB9(TestInfo info) throws Exception
   {
      addHandFist1ContactPoint(RobotSide.LEFT, jointMap, contactPointParameters);
      addKneeLowerContactPoint(RobotSide.LEFT, jointMap, contactPointParameters);
      addToeInnerBottomContactPoint(RobotSide.LEFT, jointMap, contactPointParameters);
      addShoulderPitchFrontContactPoint(RobotSide.RIGHT, jointMap, contactPointParameters);
      addKneeUpperContactPoint(RobotSide.RIGHT, jointMap, contactPointParameters);
      addToeOuterContactPoint(RobotSide.RIGHT, jointMap, contactPointParameters);
      setupSimulationAndController();

      setLegJointQs(RobotSide.LEFT, robot, jointMap, -0.4, 0.0, -1.625, 2.0, -0.8, 0.0);
      setArmJointQs(RobotSide.LEFT, robot, jointMap, -halfPi, -1.519, 1.35, -1.5);
      setLegJointQs(RobotSide.RIGHT, robot, jointMap, -0.65, -0.2, -0.425, 0.775, -0.8, 0.0);
      setArmJointQs(RobotSide.RIGHT, robot, jointMap, 0.0, 1.45, 0.45, 2.0);
      setSpineJointQs(robot, jointMap, 0.25, 0.0, 0.15);
      setRootJointPose(robot, 0.0, 0.0, 0.36, halfPi, 0.7, halfPi);
      startSim(info);
   }

   @Test
   public void testRollingB10(TestInfo info) throws Exception
   {
      addHandFist1ContactPoint(RobotSide.LEFT, jointMap, contactPointParameters);
      addKneeLowerContactPoint(RobotSide.LEFT, jointMap, contactPointParameters);
      addToeInnerBottomContactPoint(RobotSide.LEFT, jointMap, contactPointParameters);
      addShoulderPitchFrontContactPoint(RobotSide.RIGHT, jointMap, contactPointParameters);
      addKneeUpperContactPoint(RobotSide.RIGHT, jointMap, contactPointParameters);
      addToeInnerBottomContactPoint(RobotSide.RIGHT, jointMap, contactPointParameters);
      setupSimulationAndController();

      setLegJointQs(RobotSide.LEFT, robot, jointMap, 0.1, 0.0, -1.625, 2.057, -0.8, 0.0);
      setLegJointQs(RobotSide.RIGHT, robot, jointMap, -0.8, -0.2, -0.825, 1.175, -0.8, 0.0);
      setArmJointQs(RobotSide.LEFT, robot, jointMap, -halfPi, -1.519, 1.35, -1.5);
      setArmJointQs(RobotSide.RIGHT, robot, jointMap, 0.0, 1.45, 0.45, 2.0);
      setSpineJointQs(robot, jointMap, 0.65, 0.0, 0.15);
      setRootJointPose(robot, 0.0, 0.0, 0.36, halfPi, 1.1, halfPi);
      startSim(info);
   }

   @Test
   public void testRollingB11(TestInfo info) throws Exception
   {
      addHandFist1ContactPoint(RobotSide.LEFT, jointMap, contactPointParameters);
      addKneeLowerContactPoint(RobotSide.LEFT, jointMap, contactPointParameters);
      addToeInnerBottomContactPoint(RobotSide.LEFT, jointMap, contactPointParameters);
      addShoulderPitchFrontContactPoint(RobotSide.RIGHT, jointMap, contactPointParameters);
      addKneeUpperContactPoint(RobotSide.RIGHT, jointMap, contactPointParameters);
      addToeInnerBottomContactPoint(RobotSide.RIGHT, jointMap, contactPointParameters);
      setupSimulationAndController();

      setLegJointQs(RobotSide.LEFT, robot, jointMap, 0.7, 0.0, -1.625, 2.057, -0.8, 0.0);
      setLegJointQs(RobotSide.RIGHT, robot, jointMap, -0.1, -0.2, -0.825, 1.175, -0.8, 0.0);
      setArmJointQs(RobotSide.LEFT, robot, jointMap, -halfPi, -1.519, 1.35, -1.5);
      setArmJointQs(RobotSide.RIGHT, robot, jointMap, 0.0, 1.45, 0.45, 2.0);
      setSpineJointQs(robot, jointMap, 1.15, 0.0, 0.15);
      setRootJointPose(robot, 0.0, 0.0, 0.36, 0.0, halfPi, 0.0);
      startSim(info);
   }

   @Test
   public void testRollingB12(TestInfo info) throws Exception
   {
      addHandFist1ContactPoint(RobotSide.LEFT, jointMap, contactPointParameters);
      addKneeLowerContactPoint(RobotSide.LEFT, jointMap, contactPointParameters);
      addToeInnerBottomContactPoint(RobotSide.LEFT, jointMap, contactPointParameters);
      addShoulderPitchFrontContactPoint(RobotSide.RIGHT, jointMap, contactPointParameters);
      addKneeUpperContactPoint(RobotSide.RIGHT, jointMap, contactPointParameters);
      addToeInnerBottomContactPoint(RobotSide.RIGHT, jointMap, contactPointParameters);
      setupSimulationAndController();

      setLegJointQs(RobotSide.LEFT, robot, jointMap, 0.7, 0.0, -1.625, 2.057, -0.8, 0.0);
      setLegJointQs(RobotSide.RIGHT, robot, jointMap, -0.5, -0.2, -1.325, 1.725, -0.8, 0.0);
      setArmJointQs(RobotSide.LEFT, robot, jointMap, -halfPi, -1.519, 1.35, -1.5);
      setArmJointQs(RobotSide.RIGHT, robot, jointMap, 0.0, 1.45, 0.45, 2.0);
      setSpineJointQs(robot, jointMap, 1.15, 0.0, 0.15);
      setRootJointPose(robot, 0.0, 0.0, 0.36, 0.0, halfPi, 0.0);
      startSim(info);
   }

   @Test
   public void testRollingB13(TestInfo info) throws Exception
   {
      addHandFist1ContactPoint(RobotSide.LEFT, jointMap, contactPointParameters);
      addKneeLowerContactPoint(RobotSide.LEFT, jointMap, contactPointParameters);
      addToeInnerBottomContactPoint(RobotSide.LEFT, jointMap, contactPointParameters);
      addElbowOuterContactPoint(RobotSide.RIGHT, jointMap, contactPointParameters);
      addKneeUpperContactPoint(RobotSide.RIGHT, jointMap, contactPointParameters);
      addToeInnerBottomContactPoint(RobotSide.RIGHT, jointMap, contactPointParameters);
      setupSimulationAndController();

      setLegJointQs(RobotSide.LEFT, robot, jointMap, 0.7, 0.0, -1.625, 2.057, -0.8, 0.0);
      setLegJointQs(RobotSide.RIGHT, robot, jointMap, -0.5, -0.2, -1.325, 1.725, -0.8, 0.0);
      setArmJointQs(RobotSide.LEFT, robot, jointMap, -halfPi, -1.519, 1.35, -1.5);
      setArmJointQs(RobotSide.RIGHT, robot, jointMap, 0.0, 1.275, 0.65, 2.0);
      setSpineJointQs(robot, jointMap, 0.95, -0.1, 0.05);
      setRootJointPose(robot, 0.0, 0.0, 0.36, 0.0, halfPi, 0.0);
      startSim(info);
   }

   @Test
   public void testRollingB14(TestInfo info) throws Exception
   {
      addHandFist1ContactPoint(RobotSide.LEFT, jointMap, contactPointParameters);
      addKneeLowerContactPoint(RobotSide.LEFT, jointMap, contactPointParameters);
      addToeInnerBottomContactPoint(RobotSide.LEFT, jointMap, contactPointParameters);
      addKneeUpperContactPoint(RobotSide.RIGHT, jointMap, contactPointParameters);
      addToeInnerBottomContactPoint(RobotSide.RIGHT, jointMap, contactPointParameters);
      setupSimulationAndController();

      setLegJointQs(RobotSide.LEFT, robot, jointMap, 0.7, 0.0, -1.625, 2.057, -0.8, 0.0);
      setLegJointQs(RobotSide.RIGHT, robot, jointMap, -0.5, -0.2, -1.325, 1.725, -0.8, 0.0);
      setArmJointQs(RobotSide.LEFT, robot, jointMap, -halfPi, -1.0, 1.35, -1.5);
      setArmJointQs(RobotSide.RIGHT, robot, jointMap, 1.3, 1.375, -0.55, 2.0);
      setSpineJointQs(robot, jointMap, 0.65, -0.1, 0.05);
      setRootJointPose(robot, 0.0, 0.0, 0.36, 0.0, halfPi, 0.0);
      startSim(info);
   }

   @Test
   public void testRollingB15(TestInfo info) throws Exception
   {
      addHandFist1ContactPoints(jointMap, contactPointParameters);
      addKneeLowerContactPoint(RobotSide.LEFT, jointMap, contactPointParameters);
      addToeInnerBottomContactPoint(RobotSide.LEFT, jointMap, contactPointParameters);
      addKneeUpperContactPoint(RobotSide.RIGHT, jointMap, contactPointParameters);
      addToeInnerBottomContactPoint(RobotSide.RIGHT, jointMap, contactPointParameters);
      setupSimulationAndController();

      setLegJointQs(RobotSide.LEFT, robot, jointMap, 0.7, 0.0, -1.625, 2.057, -0.8, 0.0);
      setLegJointQs(RobotSide.RIGHT, robot, jointMap, -0.5, -0.2, -1.325, 1.725, -0.8, 0.0);
      setArmJointQs(RobotSide.LEFT, robot, jointMap, -halfPi, -1.0, 1.35, -1.5);
      setArmJointQs(RobotSide.RIGHT, robot, jointMap, 1.5, 1.275, -1.55, 2.0);
      setSpineJointQs(robot, jointMap, 0.65, -0.1, 0.05);
      setRootJointPose(robot, 0.0, 0.0, 0.36, 0.0, halfPi, 0.0);
      startSim(info);
   }

   @Test
   public void testJSCSequenceA1(TestInfo info) throws Exception
   {
      addHandFist1ContactPoints(jointMap, contactPointParameters);
      addKneeLowerContactPoints(jointMap, contactPointParameters);
      addToeFrontContactPoints(jointMap, contactPointParameters);
      addBreastContactPoints(jointMap, contactPointParameters);
      setupSimulationAndController();

      setLegJointQs(robot, jointMap, 0.0, 0.0, -0.25, 0.5, -0.25, 0.0);
      setArmJointQs(robot, jointMap, 0.7, -1.5, 0.0, -2.0);
      setSpineJointQs(robot, jointMap, 0.0, 0.0, 0.0);
      setRootJointPose(robot, 0.0, 0.0, 0.185, 0.0, halfPi, 0.0);
      startSim(info);
   }

   @Test
   public void testJSCSequenceA2a(TestInfo info) throws Exception
   {
      addHandFist1ContactPoints(jointMap, contactPointParameters);
      addKneeLowerContactPoints(jointMap, contactPointParameters);
      addToeFrontContactPoints(jointMap, contactPointParameters);
      setupSimulationAndController();

      setLegJointQs(robot, jointMap, 0.0, 0.0, -0.5, 0.75, -0.25, 0.0);
      setArmJointQs(robot, jointMap, 0.45, -1.5, 0.0, -2.0);
      setSpineJointQs(robot, jointMap, 0.0, -0.1, 0.0);
      setRootJointPose(robot, 0.0, 0.0, 0.285, 0.0, halfPi, 0.0);
      startSim(info);
   }

   @Test
   public void testJSCSequenceA2b(TestInfo info) throws Exception
   {
      addHandFist1ContactPoints(jointMap, contactPointParameters);
      addKneeLowerContactPoints(jointMap, contactPointParameters);
      addToeFrontContactPoints(jointMap, contactPointParameters);
      setupSimulationAndController();

      setLegJointQs(robot, jointMap, 0.0, 0.0, -0.5, 0.75, -0.25, 0.0);
      setArmJointQs(robot, jointMap, -1.55, 0.35, 1.5, -2.0);
      setSpineJointQs(robot, jointMap, 0.0, -0.1, 0.0);
      setRootJointPose(robot, 0.0, 0.0, 0.285, 0.0, halfPi, 0.0);
      startSim(info);
   }

   @Test
   public void testJSCSequenceA2c(TestInfo info) throws Exception
   {
      addHandFist1ContactPoints(jointMap, contactPointParameters);
      addKneeLowerContactPoints(jointMap, contactPointParameters);
      addToeFrontContactPoints(jointMap, contactPointParameters);
      setupSimulationAndController();

      setLegJointQs(robot, jointMap, 0.0, 0.0, -0.5, 0.75, -0.25, 0.0);
      setArmJointQs(robot, jointMap, -0.25, 1.266, 0.0, -2.0);
      setSpineJointQs(robot, jointMap, 0.0, -0.1, 0.0);
      setRootJointPose(robot, 0.0, 0.0, 0.285, 0.0, halfPi, 0.0);
      startSim(info);
   }

   @Test
   public void testJSCSequenceA3(TestInfo info) throws Exception
   {
      addHandFist1ContactPoints(jointMap, contactPointParameters);
      addKneeLowerContactPoints(jointMap, contactPointParameters);
      addToeFrontContactPoints(jointMap, contactPointParameters);
      setupSimulationAndController();

      setLegJointQs(robot, jointMap, 0.0, 0.0, -1.3, 1.625, -0.65, 0.0);
      setArmJointQs(robot, jointMap, -2.25, -1.334, 1.5, -0.5);
      setSpineJointQs(robot, jointMap, 0.0, -0.1, 0.0);
      setRootJointPose(robot, 0.0, 0.0, 0.485, 0.0, halfPi, 0.0);
      startSim(info);
   }

   @Test
   public void testJSCSequenceA4(TestInfo info) throws Exception
   {
      addHandFist1ContactPoint(RobotSide.RIGHT, jointMap, contactPointParameters);
      addKneeLowerContactPoints(jointMap, contactPointParameters);
      addToeFrontContactPoints(jointMap, contactPointParameters);
      setupSimulationAndController();

      setLegJointQs(robot, jointMap, 0.0, 0.0, -1.3, 1.625, -0.65, 0.0);
      setArmJointQs(RobotSide.LEFT, robot, jointMap, -1.45, -0.8, 1.5, -2.0);
      setArmJointQs(RobotSide.RIGHT, robot, jointMap, -2.4, 1.45, 1.5, 0.5);
      setSpineJointQs(robot, jointMap, 0.0, 0.0, 0.0);
      setRootJointPose(robot, 0.0, 0.0, 0.485, 0.0, halfPi, 0.0);
      startSim(info);
   }

   @Test
   public void testJSCSequenceA5a(TestInfo info) throws Exception
   {
      addHandFist1ContactPoints(jointMap, contactPointParameters);
      addKneeLowerContactPoints(jointMap, contactPointParameters);
      addToeFrontContactPoints(jointMap, contactPointParameters);
      setupSimulationAndController();

      setLegJointQs(robot, jointMap, 0.0, 0.0, -1.3, 1.625, -0.65, 0.0);
      setArmJointQs(RobotSide.LEFT, robot, jointMap, -1.45, -0.3, 1.5, -1.7);
      setArmJointQs(RobotSide.RIGHT, robot, jointMap, -2.25, 1.334, 1.5, 0.5);
      setSpineJointQs(robot, jointMap, 0.0, -0.1, 0.0);
      setRootJointPose(robot, 0.0, 0.0, 0.485, 0.0, halfPi, 0.0);
      startSim(info);
   }

   @Test
   public void testJSCSequenceA5b(TestInfo info) throws Exception
   {
      addHandFist1ContactPoints(jointMap, contactPointParameters);
      addKneeLowerContactPoints(jointMap, contactPointParameters);
      addToeFrontContactPoints(jointMap, contactPointParameters);
      setupSimulationAndController();

      setLegJointQs(robot, jointMap, 0.0, 0.0, -1.3, 1.625, -0.65, 0.0);
      setArmJointQs(RobotSide.LEFT, robot, jointMap, -0.25, -1.4, 0.2, -1.7);
      setArmJointQs(RobotSide.RIGHT, robot, jointMap, -2.25, 1.334, 1.5, 0.5);
      setSpineJointQs(robot, jointMap, 0.0, -0.1, 0.0);
      setRootJointPose(robot, 0.0, 0.0, 0.485, 0.0, halfPi, 0.0);
      startSim(info);
   }

   @Test
   public void testJSCSequenceA5c(TestInfo info) throws Exception
   {
      addElbowContactPoint(RobotSide.LEFT, jointMap, contactPointParameters);
      addHandFist1ContactPoint(RobotSide.RIGHT, jointMap, contactPointParameters);
      addKneeLowerContactPoints(jointMap, contactPointParameters);
      addToeFrontContactPoints(jointMap, contactPointParameters);
      setupSimulationAndController();

      setLegJointQs(robot, jointMap, 0.0, 0.0, -1.3, 1.625, -0.65, 0.0);
      setArmJointQs(RobotSide.LEFT, robot, jointMap, -2.05, -1.4, 1.5, -1.7);
      setArmJointQs(RobotSide.RIGHT, robot, jointMap, -2.75, 1.334, 1.5, 0.5);
      setSpineJointQs(robot, jointMap, 0.0, 0.2, 0.0);
      setRootJointPose(robot, 0.0, 0.0, 0.485, 0.0, halfPi, 0.0);
      startSim(info);
   }

   @Test
   public void testJSCSequenceA6a(TestInfo info) throws Exception
   {
      addHandFist1ContactPoint(RobotSide.LEFT, jointMap, contactPointParameters);
      addKneeLowerContactPoints(jointMap, contactPointParameters);
      addToeInnerBottomContactPoints(jointMap, contactPointParameters);
      setupSimulationAndController();

      setLegJointQs(robot, jointMap, 0.6, 0.0, -1.5, 1.925, -0.65, 0.0);
      setArmJointQs(RobotSide.LEFT, robot, jointMap, -1.5, -0.6, 1.1, -2.0);
      setArmJointQs(RobotSide.RIGHT, robot, jointMap, -2.0, 0.7, 0.7, 1.7);
      setSpineJointQs(robot, jointMap, 0.0, -0.1, 0.0);
      setRootJointPose(robot, 0.0, 0.0, 0.4, 0.0, halfPi, 0.0);
      startSim(info);
   }

   @Test
   public void testJSCSequenceA6b(TestInfo info) throws Exception
   {
      addElbowContactPoint(RobotSide.LEFT, jointMap, contactPointParameters);
      addKneeLowerContactPoints(jointMap, contactPointParameters);
      addToeInnerBottomContactPoints(jointMap, contactPointParameters);
      setupSimulationAndController();

      setLegJointQs(robot, jointMap, 0.6, 0.0, -1.5, 1.925, -0.65, 0.0);
      setArmJointQs(RobotSide.LEFT, robot, jointMap, -1.5, -1.5, 1.5, -1.7);
      setArmJointQs(RobotSide.RIGHT, robot, jointMap, -2.0, 0.7, 0.7, 1.7);
      setSpineJointQs(robot, jointMap, 0.0, -0.1, 0.0);
      setRootJointPose(robot, 0.0, 0.0, 0.4, 0.0, halfPi, 0.0);
      startSim(info);
   }

   @Test
   public void testJSCSequenceA6B(TestInfo info) throws Exception
   {
      addHandFist1ContactPoints(jointMap, contactPointParameters);
      addKneeLowerContactPoints(jointMap, contactPointParameters);
      addToeInnerBottomContactPoints(jointMap, contactPointParameters);
      setupSimulationAndController();

      setLegJointQs(robot, jointMap, 0.6, 0.0, -1.5, 1.925, -0.65, 0.0);
      setArmJointQs(robot, jointMap, -1.5, -0.6, 1.1, -2.0);
      setSpineJointQs(robot, jointMap, 0.0, -0.1, 0.0);
      setRootJointPose(robot, 0.0, 0.0, 0.4, 0.0, halfPi, 0.0);
      startSim(info);
   }

   @Test
   public void testJSCSequenceA7a(TestInfo info) throws Exception
   {
      addHandFist1ContactPoints(jointMap, contactPointParameters);
      addKneeLowerContactPoints(jointMap, contactPointParameters);
      addToeInnerBottomContactPoints(jointMap, contactPointParameters);
      setupSimulationAndController();

      setLegJointQs(robot, jointMap, 0.6, 0.25, -1.8, 2.05, -0.8, 0.0);
      setArmJointQs(robot, jointMap, -1.8, -0.775, 1.1, -1.7);
      setSpineJointQs(robot, jointMap, 0.0, -0.1, 0.0);
      setRootJointPose(robot, 0.0, 0.0, 0.4, 0.0, halfPi, 0.0);
      startSim(info);
   }

   @Test
   public void testJSCSequenceA7b(TestInfo info) throws Exception
   {
      addHandFist1ContactPoints(jointMap, contactPointParameters);
      addKneeLowerContactPoints(jointMap, contactPointParameters);
      addToeInnerBottomContactPoints(jointMap, contactPointParameters);
      setupSimulationAndController();

      setLegJointQs(robot, jointMap, 0.45, 0.55, -1.25, 2.05, -0.8, 0.0);
      setArmJointQs(robot, jointMap, -1.55, -1.519, 1.1, -0.2);
      setSpineJointQs(robot, jointMap, 0.0, 0.075, 0.0);
      setRootJointPose(robot, 0.0, 0.0, 0.5, 0.0, 1.0, 0.0);
      startSim(info);
   }

   @Test
   public void testJSCSequenceA7c(TestInfo info) throws Exception
   {
      addHandFist1ContactPoints(jointMap, contactPointParameters);
      addKneeLowerContactPoints(jointMap, contactPointParameters);
      addToeInnerBottomContactPoints(jointMap, contactPointParameters);
      setupSimulationAndController();

      setLegJointQs(robot, jointMap, 0.45, 0.55, -0.9, 2.05, -0.8, 0.3);
      setArmJointQs(robot, jointMap, -1.45, -1.519, 1.1, -0.2);
      setSpineJointQs(robot, jointMap, 0.0, 0.4, 0.0);
      setRootJointPose(robot, 0.0, 0.0, 0.53, 0.0, 0.7, 0.0);
      startSim(info);
   }

   @Test
   public void testJSCSequenceA7d(TestInfo info) throws Exception
   {
      addHandFist1ContactPoints(jointMap, contactPointParameters);
      addKneeLowerContactPoints(jointMap, contactPointParameters);
      addToeInnerBottomContactPoints(jointMap, contactPointParameters);
      setupSimulationAndController();

      setLegJointQs(robot, jointMap, 0.0, 0.0, -0.875, 2.05, -0.8, 0.0);
      setArmJointQs(robot, jointMap, -1.35, -1.519, 1.2, -0.2);
      setSpineJointQs(robot, jointMap, 0.0, 0.666, 0.0);
      setRootJointPose(robot, 0.0, 0.0, 0.67, 0.0, 0.75, 0.0);
      startSim(info);
   }

   @Test
   public void testJSCSequenceA10(TestInfo info) throws Exception
   {
      addKneeMidEdgeContactPoints(jointMap, contactPointParameters);
      addToeBottomEdgeContactPoints(jointMap, contactPointParameters);
      setupSimulationAndController();

      setLegJointQs(robot, jointMap, 0.0, 0.0, -0.15, 2.05, -0.8, 0.0);
      setArmJointQs(robot, jointMap, 0.0, -1.3, 0.4, -0.7);
      setSpineJointQs(robot, jointMap, 0.0, 0.0, 0.0);
      setRootJointPose(robot, 0.0, 0.0, 0.74, 0.0, 0.0, 0.0);
      startSim(info);
   }

   @Test
   public void testJSCSequenceA11(TestInfo info) throws Exception
   {
      addKneeMidEdgeContactPoints(RobotSide.LEFT, jointMap, contactPointParameters);
      addToeBottomEdgeContactPoints(RobotSide.LEFT, jointMap, contactPointParameters);
      setupSimulationAndController();

      setLegJointQs(RobotSide.LEFT, robot, jointMap, 0.0, 0.5, -0.15, 2.05, -0.8, 0.0);
      setLegJointQs(RobotSide.RIGHT, robot, jointMap, 0.0, 0.4, -0.15, 2.05, -0.8, 0.0);
      setArmJointQs(RobotSide.LEFT, robot, jointMap, 0.0, -0.6, 0.4, -0.7);
      setArmJointQs(RobotSide.RIGHT, robot, jointMap, 0.0, 1.5, 0.4, 1.3);
      setSpineJointQs(robot, jointMap, 0.0, 0.0, -0.23);
      setRootJointPose(robot, 0.0, 0.0, 0.755, 0.0, 0.0, -0.5);
      startSim(info);
   }

   @Test
   public void testJSCSequenceA12(TestInfo info) throws Exception
   {
      addKneeMidEdgeContactPoints(RobotSide.LEFT, jointMap, contactPointParameters);
      addToeBottomEdgeContactPoints(RobotSide.LEFT, jointMap, contactPointParameters);
      setupSimulationAndController();

      setLegJointQs(RobotSide.LEFT, robot, jointMap, 0.0, 0.5, -0.15, 2.05, -0.8, 0.0);
      setLegJointQs(RobotSide.RIGHT, robot, jointMap, 0.0, 0.4, -1.4, 1.35, 0.05, 0.1);
      setArmJointQs(RobotSide.LEFT, robot, jointMap, 0.0, -0.6, 0.4, -0.7);
      setArmJointQs(RobotSide.RIGHT, robot, jointMap, 0.0, 1.5, 0.4, 1.3);
      setSpineJointQs(robot, jointMap, 0.0, 0.0, -0.23);
      setRootJointPose(robot, 0.0, 0.0, 0.755, 0.0, 0.0, -0.5);
      startSim(info);
   }

   @Test
   public void testJSCSequenceA13a(TestInfo info) throws Exception
   {
      addKneeMidEdgeContactPoints(RobotSide.LEFT, jointMap, contactPointParameters);
      addToeBottomEdgeContactPoints(RobotSide.LEFT, jointMap, contactPointParameters);
      addFootFullSupportContactPoints(RobotSide.RIGHT, jointMap, contactPointParameters);
      setupSimulationAndController();

      setLegJointQs(RobotSide.LEFT, robot, jointMap, 0.0, 0.5, -0.15, 2.05, -0.8, 0.0);
      setLegJointQs(RobotSide.RIGHT, robot, jointMap, 0.0, 0.4, -1.35, 1.35, 0.0, 0.1);
      setArmJointQs(RobotSide.LEFT, robot, jointMap, 0.0, -0.6, 0.4, -0.7);
      setArmJointQs(RobotSide.RIGHT, robot, jointMap, 0.0, 1.5, 0.4, 1.3);
      setSpineJointQs(robot, jointMap, 0.0, 0.0, -0.23);
      setRootJointPose(robot, 0.0, 0.0, 0.755, 0.0, 0.0, -0.5);
      startSim(info);
   }

   @Test
   public void testJSCSequenceA13b(TestInfo info) throws Exception
   {
      addKneeMidEdgeContactPoints(RobotSide.LEFT, jointMap, contactPointParameters);
      addToeBottomEdgeContactPoints(RobotSide.LEFT, jointMap, contactPointParameters);
      addFootFullSupportContactPoints(RobotSide.RIGHT, jointMap, contactPointParameters);
      setupSimulationAndController();

      setLegJointQs(RobotSide.LEFT, robot, jointMap, 0.0, 0.0, -0.15, 2.05, -0.8, 0.0);
      setLegJointQs(RobotSide.RIGHT, robot, jointMap, 0.0, 0.0, -1.55, 1.35, 0.2, 0.0);
      setArmJointQs(robot, jointMap, 0.0, -1.2, 0.4, -0.7);
      setSpineJointQs(robot, jointMap, 0.0, 0.0, 0.0);
      setRootJointPose(robot, 0.0, 0.0, 0.735, 0.0, 0.0, 0.0);
      startSim(info);
   }

   @Test
   public void testJSCSequenceA15a(TestInfo info) throws Exception
   {
      addToeBottomEdgeContactPoints(RobotSide.LEFT, jointMap, contactPointParameters);
      addFootFullSupportContactPoints(RobotSide.RIGHT, jointMap, contactPointParameters);
      setupSimulationAndController();

      setLegJointQs(RobotSide.LEFT, robot, jointMap, 0.0, 0.0, 0.05, 1.6, -0.5, 0.0);
      setLegJointQs(RobotSide.RIGHT, robot, jointMap, 0.0, 0.0, -1.31, 1.35, -0.04, 0.0);
      setArmJointQs(robot, jointMap, 0.0, -1.2, 0.4, -0.7);
      setSpineJointQs(robot, jointMap, 0.0, 0.0, 0.0);
      setRootJointPose(robot, 0.0, 0.0, 0.85, 0.0, 0.0, 0.0);
      startSim(info);
   }

   @Test
   public void testJSCSequenceA15b(TestInfo info) throws Exception
   {
      addToeBottomEdgeContactPoints(RobotSide.LEFT, jointMap, contactPointParameters);
      addFootFullSupportContactPoints(RobotSide.RIGHT, jointMap, contactPointParameters);
      setupSimulationAndController();
      
      setLegJointQs(RobotSide.LEFT, robot, jointMap, 0.0, 0.0, -0.25, 1.59, -0.5, 0.0);
      setLegJointQs(RobotSide.RIGHT, robot, jointMap, 0.0, 0.0, -1.62, 1.35, -0.03, 0.0);
      setArmJointQs(robot, jointMap, 0.0, -1.2, 0.4, -0.7);
      setSpineJointQs(robot, jointMap, 0.0, 0.0, 0.0);
      setRootJointPose(robot, 0.0, 0.0, 0.835, 0.0, 0.3, 0.0);
      startSim(info);
   }

   @Test
   public void testJSCSequenceA16(TestInfo info) throws Exception
   {
      addToeBottomEdgeContactPoints(RobotSide.LEFT, jointMap, contactPointParameters);
      addFootFullSupportContactPoints(RobotSide.RIGHT, jointMap, contactPointParameters);
      setupSimulationAndController();
      
      setLegJointQs(RobotSide.LEFT, robot, jointMap, 0.0, 0.06, 0.365, 0.3, 0.5, 0.0);
      setLegJointQs(RobotSide.RIGHT, robot, jointMap, 0.0, 0.1, -1.1, 1.2, -0.4, 0.0);
      setArmJointQs(RobotSide.LEFT, robot, jointMap, 0.0, -1.4, 0.4, -0.7);
      setArmJointQs(RobotSide.RIGHT, robot, jointMap, 0.0, 1.2, 0.4, 0.7);
      setSpineJointQs(robot, jointMap, 0.0, 0.0, 0.2);
      setRootJointPose(robot, 0.0, 0.0, 0.995, 0.0, 0.3, 0.0);
      startSim(info);
   }

   private void startSim(TestInfo info) throws Exception
   {
      assertJointLimits(robot);
      setupMultiView();
      blockingSimulationRunner = new BlockingSimulationRunner(scs, 1000.0);
      scs.startOnAThread();
      blockingSimulationRunner.simulateAndBlockAndCatchExceptions(0.1);
      configurePlotter();
      blockingSimulationRunner.simulateAndBlockAndCatchExceptions(1.4);
      scs.disableGUIComponents();
      String configurationName = info.getTestMethod().get().getName().replace("test", "");
      saveScreenshot(outputFolder, configurationName);
      scs.writeState(new File(outputFolder, configurationName + ".state"));
      scs.enableGUIComponents();
      ThreadTools.sleepForever();
   }

   private void configurePlotter()
   {
      EventDispatchThreadHelper.invokeAndWait(() ->
      {
         YoFrameConvexPolygon2D supportPolygon = controller.getSupportPolygon();
         double xRange = supportPolygon.getMaxX() - supportPolygon.getMinX();
         double yRange = supportPolygon.getMaxY() - supportPolygon.getMinY();

         plotter.getPlotter().setViewRange(1.5 * Math.max(xRange, yRange));
         plotter.getPlotter().setFocusPointX(0.5 * (supportPolygon.getMaxX() + supportPolygon.getMinX()));
         plotter.getPlotter().setFocusPointY(0.5 * (supportPolygon.getMaxY() + supportPolygon.getMinY()));

         FullHumanoidRobotModel fullRobotModel = controller.getFullRobotModel();
         FramePoint3D headPosition = new FramePoint3D(fullRobotModel.getHead().getBodyFixedFrame());
         headPosition.changeFrame(ReferenceFrame.getWorldFrame());
         TextArtifact headText = new TextArtifact("head", "Head", headPosition.getX(), headPosition.getY());
         headText.setFontSize(16);
         plotter.getPlotter().addArtifact(headText);
         FramePoint3D pelvisPosition = new FramePoint3D(fullRobotModel.getPelvis().getBodyFixedFrame());
         pelvisPosition.changeFrame(ReferenceFrame.getWorldFrame());
         TextArtifact pelvisText = new TextArtifact("pelvis", "Pelvis", pelvisPosition.getX(), pelvisPosition.getY());
         pelvisText.setFontSize(16);
         plotter.getPlotter().addArtifact(pelvisText);
      });
   }

   private void saveScreenshot(File outputFolder, String fileName) throws IOException
   {
      BufferedImage view3Dimage = getSCSView3D(0, 2.0);

      int smallWidth = view3Dimage.getWidth();
      int smallHeight = view3Dimage.getHeight();
      int width = 2 * smallWidth + 5;
      int height = 2 * smallHeight + 5;
      BufferedImage allViews = new BufferedImage(width, height, BufferedImage.TYPE_INT_RGB);
      Graphics graphics = allViews.getGraphics();
      graphics.drawImage(view3Dimage, 0, 0, null);
      graphics.drawImage(getSCSView3D(1, 2.0), smallWidth + 5, 0, null);
      graphics.drawImage(getSCSView3D(2, 2.0), 0, smallHeight + 5, null);
      graphics.drawImage(getSCSView3D(3, 2.0), smallWidth + 5, smallHeight + 5, null);

      BufferedImage plotterImage = new BufferedImage(plotter.getJPanel().getWidth(), plotter.getJPanel().getHeight(), BufferedImage.TYPE_INT_RGB);
      plotter.getJPanel().paint(plotterImage.getGraphics());
      graphics.drawImage(plotterImage, smallWidth - plotterImage.getWidth() / 2, height - plotterImage.getHeight(), null);
      ImageIO.write(allViews, "png", new File(outputFolder, fileName + ".png"));
   }

   private BufferedImage getSCSView3D(int viewIndex, double scale)
   {
      ArrayList<ViewportAdapterAndCameraControllerHolder> cameraAdapters = viewportWindow.getViewportPanel().getCameraAdapters();
      BufferedImage exportSnapshotAsBufferedImage = scs.exportSnapshotAsBufferedImage(cameraAdapters.get(viewIndex).getViewportAdapter().getCaptureDevice());
      return scaleImage(exportSnapshotAsBufferedImage, scale);
   }

   private static BufferedImage scaleImage(BufferedImage imageToScale, double scale)
   {
      AffineTransform scaleTransform = new AffineTransform();
      scaleTransform.scale(scale, scale);
      AffineTransformOp scaleOp = new AffineTransformOp(scaleTransform, AffineTransformOp.TYPE_BILINEAR);
      scaleOp.getRenderingHints().add(new RenderingHints(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON));
      return scaleOp.filter(imageToScale, null);
   }

   private void setupSimulationAndController()
   {
      robotModel.setContactPointParameters(contactPointParameters);
      robot = robotModel.createHumanoidFloatingRootJointRobot(false);
      FullHumanoidRobotModel fullRobotModel = robotModel.createFullRobotModel();
      // Setup Controller
      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();
      List<? extends ContactablePlaneBody> contactablePlaneBodies = createContactablePlaneBodies(fullRobotModel);
      controller = new MultiContactStaticController(robotModel.getControllerDT(),
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

      // Setup SCS
      FlatGroundEnvironment flatGroundEnvironment = new FlatGroundEnvironment();
      scs = new SimulationConstructionSet(robot, SimulationTestingParameters.createFromSystemProperties());

      createPlotter(yoGraphicsListRegistry);

      scs.setGroundVisible(false);
      scs.addStaticLinkGraphics(flatGroundEnvironment.getTerrainObject3D().getLinkGraphics());
      scs.addYoGraphicsListRegistry(yoGraphicsListRegistry);

      DRCSCSInitialSetup initialSetup = new DRCSCSInitialSetup(flatGroundEnvironment, simulationDT);
      initialSetup.setDrawGroundProfile(false);
      initialSetup.setRecordFrequency((int) (robotModel.getControllerDT() / simulationDT));
      initialSetup.initializeSimulation(scs);
      initialSetup.initializeRobot(robot, robotModel, yoGraphicsListRegistry);
   }

   private void createPlotter(YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      plotter = new SimulationOverheadPlotter();
      scs.attachPlaybackListener(plotter);

      plotterWindow = new JFrame("Plotter");
      plotterWindow.setSize(new Dimension(1000, 1000));
      plotterWindow.add(plotter.getJPanel());
      yoGraphicsListRegistry.addArtifactListsToPlotter(plotter.getPlotter());
      plotterWindow.setVisible(true);
   }

   private void setupMultiView()
   {
      double rootX = robot.getRootJoint().getQx().getValue();
      double rootY = robot.getRootJoint().getQy().getValue();
      double rootZ = robot.getRootJoint().getQz().getValue();

      CameraConfiguration rightCamera = new CameraConfiguration("RightView");
      rightCamera.setCameraFix(rootX, rootY, rootZ);
      rightCamera.setCameraPosition(rootX + 0.0, rootY - 5.0, rootZ + 0.0);
      rightCamera.setCameraTracking(false, false, false, false);
      rightCamera.setCameraDolly(false, false, false, false);
      scs.setupCamera(rightCamera);
      CameraConfiguration leftCamera = new CameraConfiguration("LeftView");
      leftCamera.setCameraFix(rootX, rootY, rootZ);
      leftCamera.setCameraPosition(rootX + 0.0, rootY + 5.0, rootZ + 0.0);
      leftCamera.setCameraTracking(false, false, false, false);
      leftCamera.setCameraDolly(false, false, false, false);
      scs.setupCamera(leftCamera);
      CameraConfiguration headCamera = new CameraConfiguration("HeadView");
      headCamera.setCameraFix(rootX, rootY, rootZ);
      headCamera.setCameraPosition(rootX + 5.0, rootY + 0.0, rootZ + 1.40);
      headCamera.setCameraTracking(false, false, false, false);
      headCamera.setCameraDolly(false, false, false, false);
      scs.setupCamera(headCamera);
      CameraConfiguration feetCamera = new CameraConfiguration("FeetView");
      feetCamera.setCameraFix(rootX, rootY, rootZ);
      feetCamera.setCameraPosition(rootX - 5.0, rootY + 0.0, rootZ + 1.40);
      feetCamera.setCameraTracking(false, false, false, false);
      feetCamera.setCameraDolly(false, false, false, false);
      scs.setupCamera(feetCamera);

      ViewportConfiguration multiView = new ViewportConfiguration("multi-view");
      multiView.addCameraView("RightView", 0, 0, 1, 1);
      multiView.addCameraView("LeftView", 1, 0, 1, 1);
      multiView.addCameraView("HeadView", 0, 1, 1, 1);
      multiView.addCameraView("FeetView", 1, 1, 1, 1);
      scs.setupViewport(multiView);
      viewportWindow = scs.createNewViewportWindow("multi-view", 1, true);
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

   private List<? extends ContactablePlaneBody> createContactablePlaneBodies(FullHumanoidRobotModel fullRobotModel)
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

   private static void setLegJointQs(HumanoidFloatingRootJointRobot robot, DRCRobotJointMap jointMap, double hipYaw, double hipRoll, double hipPitch,
                                     double knee, double anklePitch, double ankleRoll)
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         setLegJointQs(robotSide,
                       robot,
                       jointMap,
                       robotSide.negateIfRightSide(hipYaw),
                       robotSide.negateIfRightSide(hipRoll),
                       hipPitch,
                       knee,
                       anklePitch,
                       robotSide.negateIfRightSide(ankleRoll));
      }
   }

   private static void setLegJointQs(RobotSide robotSide, HumanoidFloatingRootJointRobot robot, DRCRobotJointMap jointMap, double hipYaw, double hipRoll,
                                     double hipPitch, double knee, double anklePitch, double ankleRoll)
   {
      robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(robotSide, LegJointName.HIP_YAW)).setQ(hipYaw);
      robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(robotSide, LegJointName.HIP_ROLL)).setQ(hipRoll);
      robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(robotSide, LegJointName.HIP_PITCH)).setQ(hipPitch);
      robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(robotSide, LegJointName.KNEE_PITCH)).setQ(knee);
      robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(robotSide, LegJointName.ANKLE_PITCH)).setQ(anklePitch);
      robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(robotSide, LegJointName.ANKLE_ROLL)).setQ(ankleRoll);
   }

   private static void setArmJointQs(HumanoidFloatingRootJointRobot robot, DRCRobotJointMap jointMap, double shoulderPitch, double shoulderRoll,
                                     double shoulderYaw, double elbowPitch)
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         setArmJointQs(robotSide,
                       robot,
                       jointMap,
                       shoulderPitch,
                       robotSide.negateIfRightSide(shoulderRoll),
                       shoulderYaw,
                       robotSide.negateIfRightSide(elbowPitch));
      }
   }

   private static void setArmJointQs(RobotSide robotSide, HumanoidFloatingRootJointRobot robot, DRCRobotJointMap jointMap, double shoulderPitch,
                                     double shoulderRoll, double shoulderYaw, double elbowPitch)
   {
      robot.getOneDegreeOfFreedomJoint(jointMap.getArmJointName(robotSide, ArmJointName.SHOULDER_PITCH)).setQ(shoulderPitch);
      robot.getOneDegreeOfFreedomJoint(jointMap.getArmJointName(robotSide, ArmJointName.SHOULDER_ROLL)).setQ(shoulderRoll);
      robot.getOneDegreeOfFreedomJoint(jointMap.getArmJointName(robotSide, ArmJointName.SHOULDER_YAW)).setQ(shoulderYaw);
      robot.getOneDegreeOfFreedomJoint(jointMap.getArmJointName(robotSide, ArmJointName.ELBOW_PITCH)).setQ(elbowPitch);
   }

   private static void setSpineJointQs(HumanoidFloatingRootJointRobot robot, DRCRobotJointMap jointMap, double qYaw, double qPitch, double qRoll)
   {
      robot.getOneDegreeOfFreedomJoint(jointMap.getSpineJointName(SpineJointName.SPINE_YAW)).setQ(qYaw);
      robot.getOneDegreeOfFreedomJoint(jointMap.getSpineJointName(SpineJointName.SPINE_PITCH)).setQ(qPitch);
      robot.getOneDegreeOfFreedomJoint(jointMap.getSpineJointName(SpineJointName.SPINE_ROLL)).setQ(qRoll);
   }

   private static void setRootJointPose(HumanoidFloatingRootJointRobot robot, double x, double y, double z, double yaw, double pitch, double roll)
   {
      RigidBodyTransform rootJointPose = new RigidBodyTransform();
      rootJointPose.setRotation(new Quaternion(yaw, pitch, roll));
      rootJointPose.setTranslation(x, y, z);
      robot.getRootJoint().setRotationAndTranslation(rootJointPose);
   }

   private static void setRootJointPose(HumanoidFloatingRootJointRobot robot, double x, double y, double z, double qx, double qy, double qz, double qs)
   {
      robot.getRootJoint().setQuaternion(new Quaternion(qx, qy, qz, qs));
      robot.getRootJoint().setPosition(x, y, z);
   }

   private static void assertJointLimits(HumanoidFloatingRootJointRobot robot)
   {
      ArrayList<OneDegreeOfFreedomJoint> joints = new ArrayList<>();
      robot.getAllOneDegreeOfFreedomJoints(joints);

      for (OneDegreeOfFreedomJoint joint : joints)
      {
         assertTrue(joint.getQ() >= joint.getJointLowerLimit(),
                    "Joint " + joint.getName() + " is out of range: " + joint.getQ() + " < lower-limit (" + joint.getJointLowerLimit() + ")");
         assertTrue(joint.getQ() <= joint.getJointUpperLimit(),
                    "Joint " + joint.getName() + " is out of range: " + joint.getQ() + " > upper-limit (" + joint.getJointUpperLimit() + ")");
      }
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
         addKneeUpperContactPoint(robotSide, jointMap, contactPointParameters);
   }

   private static void addKneeUpperContactPoint(RobotSide robotSide, ValkyrieJointMap jointMap, ValkyrieMultiContactPointParameters contactPointParameters)
   {
      String parentJointName = jointMap.getLegJointName(robotSide, LegJointName.HIP_PITCH);
      String bodyName = parentJointName + "Link";
      String contactName = robotSide.getCamelCaseName() + "KneeCP";
      contactPointParameters.addSingleContactPoint(parentJointName, bodyName, contactName, new Point3D(0.06, robotSide.negateIfRightSide(0.07), -0.46));
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
         addKneeLowerContactPoint(robotSide, jointMap, contactPointParameters);
   }

   private static void addKneeLowerContactPoint(RobotSide robotSide, ValkyrieJointMap jointMap, ValkyrieMultiContactPointParameters contactPointParameters)
   {
      String parentJointName = jointMap.getLegJointName(robotSide, LegJointName.KNEE_PITCH);
      String bodyName = parentJointName + "Link";
      String contactName = robotSide.getCamelCaseName() + "KneeCP";
      contactPointParameters.addSingleContactPoint(parentJointName, bodyName, contactName, new Point3D(0.075, 0.0, -0.01));
   }

   private static void addKneeMidContactPoints(ValkyrieJointMap jointMap, ValkyrieMultiContactPointParameters contactPointParameters)
   {
      for (RobotSide robotSide : RobotSide.values)
         addKneeMidContactPoint(robotSide, jointMap, contactPointParameters);
   }

   private static void addKneeMidContactPoint(RobotSide robotSide, ValkyrieJointMap jointMap, ValkyrieMultiContactPointParameters contactPointParameters)
   {
      String parentJointName = jointMap.getLegJointName(robotSide, LegJointName.KNEE_PITCH);
      String bodyName = parentJointName + "Link";
      String contactName = robotSide.getCamelCaseName() + "KneeCP";
      contactPointParameters.addSingleContactPoint(parentJointName, bodyName, contactName, new Point3D(0.05, 0.0, 0.04));
   }

   private static void addKneeMidEdgeContactPoints(ValkyrieJointMap jointMap, ValkyrieMultiContactPointParameters contactPointParameters)
   {
      for (RobotSide robotSide : RobotSide.values)
         addKneeMidEdgeContactPoints(robotSide, jointMap, contactPointParameters);
   }

   private static void addKneeMidEdgeContactPoints(RobotSide robotSide, ValkyrieJointMap jointMap, ValkyrieMultiContactPointParameters contactPointParameters)
   {
      String parentJointName = jointMap.getLegJointName(robotSide, LegJointName.KNEE_PITCH);
      String bodyName = parentJointName + "Link";
      String contactName = robotSide.getCamelCaseName() + "KneeCP";

      RigidBodyTransform contactFramePose = new RigidBodyTransform(new YawPitchRoll(0.0, -halfPi, 0.0), new Point3D(0.05, 0.0, 0.04));
      List<Point2D> contactPoints = new ArrayList<>();
      contactPoints.add(new Point2D(0.0, 0.08));
      contactPoints.add(new Point2D(0.0, -0.02));
      contactPointParameters.addPlaneContact(parentJointName, bodyName, contactName, contactFramePose, contactPoints);
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

   private static void addToeInnerBottomContactPoints(ValkyrieJointMap jointMap, ValkyrieMultiContactPointParameters contactPointParameters)
   {
      for (RobotSide robotSide : RobotSide.values)
         addToeInnerBottomContactPoint(robotSide, jointMap, contactPointParameters);
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

   private static void addToeBottomEdgeContactPoints(ValkyrieJointMap jointMap, ValkyrieMultiContactPointParameters contactPointParameters)
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         addToeBottomEdgeContactPoints(robotSide, jointMap, contactPointParameters);
      }
   }

   private static void addToeBottomEdgeContactPoints(RobotSide robotSide, ValkyrieJointMap jointMap, ValkyrieMultiContactPointParameters contactPointParameters)
   {
      String parentJointName = jointMap.getJointBeforeFootName(robotSide);
      String bodyName = jointMap.getFootName(robotSide);
      String contactName = robotSide.getCamelCaseName() + "ToeCP";

      RigidBodyTransform contactFramePose = new RigidBodyTransform(new YawPitchRoll(0.0, 0.0, 0.0), new Point3D(0.0, 0.0, -0.085));
      List<Point2D> contactPoints = new ArrayList<>();
      contactPoints.add(new Point2D(0.18, 0.07));
      contactPoints.add(new Point2D(0.18, -0.07));
      contactPointParameters.addPlaneContact(parentJointName, bodyName, contactName, contactFramePose, contactPoints);
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

   private static void addFootFullSupportContactPoints(RobotSide robotSide, ValkyrieJointMap jointMap,
                                                       ValkyrieMultiContactPointParameters contactPointParameters)
   {
      String parentJointName = jointMap.getJointBeforeFootName(robotSide);
      String bodyName = jointMap.getFootName(robotSide);
      String contactName = robotSide.getCamelCaseName() + "FootCP";

      RigidBodyTransform contactFramePose = new RigidBodyTransform(new YawPitchRoll(0.0, 0.0, 0.0), new Point3D(0.0, 0.0, -0.085));
      List<Point2D> contactPoints = new ArrayList<>();
      contactPoints.add(new Point2D(0.18, 0.07));
      contactPoints.add(new Point2D(0.18, -0.07));
      contactPoints.add(new Point2D(-0.085, 0.07));
      contactPoints.add(new Point2D(-0.085, -0.07));
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

   private static void addShoulderPitchFrontContactPoint(RobotSide robotSide, ValkyrieJointMap jointMap,
                                                         ValkyrieMultiContactPointParameters contactPointParameters)
   {
      String parentJointName = jointMap.getArmJointName(robotSide, ArmJointName.SHOULDER_PITCH);
      String bodyName = parentJointName + "Link";
      String contactName = robotSide.getCamelCaseName() + "ShoulderCP";
      contactPointParameters.addSingleContactPoint(parentJointName, bodyName, contactName, new Point3D(0.1, robotSide.negateIfRightSide(0.3), 0.0));
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

      RigidBodyTransform contactFramePose = new RigidBodyTransform(new YawPitchRoll(0.0, halfPi, 0.0), new Point3D(-0.29, 0.0, 0.25));
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

      RigidBodyTransform contactFramePose = new RigidBodyTransform(new YawPitchRoll(0.0, halfPi, 0.0), new Point3D(-0.29, 0.0, 0.25));
      List<Point2D> contactPoints = new ArrayList<>();
      contactPoints.add(new Point2D(0.15, robotSide.negateIfRightSide(0.1)));
      contactPoints.add(new Point2D(-0.15, robotSide.negateIfRightSide(0.1)));
      contactPointParameters.addPlaneContact(parentJointName, bodyName, contactName, contactFramePose, contactPoints);
   }

   private static void addBreastContactPoints(ValkyrieJointMap jointMap, ValkyrieMultiContactPointParameters contactPointParameters)
   {
      String parentJointName = jointMap.getNameOfJointBeforeChest();
      String bodyName = jointMap.getChestName();
      String contactName = "BackpackCP";

      RigidBodyTransform contactFramePose = new RigidBodyTransform(new YawPitchRoll(0.0, -halfPi, 0.0), new Point3D(0.15, 0.0, 0.25));
      List<Point2D> contactPoints = new ArrayList<>();
      contactPoints.add(new Point2D(0.0, 0.08));
      contactPoints.add(new Point2D(0.0, -0.08));
      contactPointParameters.addPlaneContact(parentJointName, bodyName, contactName, contactFramePose, contactPoints);
   }
}
