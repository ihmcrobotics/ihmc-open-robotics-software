package us.ihmc.simulationconstructionset;

import static org.junit.Assert.assertArrayEquals;
import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertTrue;

import java.awt.AWTException;
import java.awt.Button;
import java.awt.Component;
import java.awt.Dimension;
import java.awt.Frame;
import java.awt.Point;
import java.awt.image.BufferedImage;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.io.InputStreamReader;
import java.util.ArrayList;

import javax.swing.AbstractAction;

import org.fest.swing.edt.FailOnThreadViolationRepaintManager;
import org.junit.After;
import org.junit.Before;
import org.junit.BeforeClass;
import org.junit.Ignore;
import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.structure.Graphics3DNode;
import us.ihmc.graphicsDescription.structure.Graphics3DNodeType;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphic;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicVector;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsList;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.jMonkeyEngineToolkit.camera.CameraConfiguration;
import us.ihmc.jMonkeyEngineToolkit.camera.CaptureDevice;
import us.ihmc.jMonkeyEngineToolkit.camera.ClassicCameraController;
import us.ihmc.jMonkeyEngineToolkit.jme.JMEGraphics3DAdapter;
import us.ihmc.robotics.dataStructures.listener.RewoundListener;
import us.ihmc.robotics.dataStructures.registry.NameSpace;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.YoVariable;
import us.ihmc.robotics.dataStructures.variable.YoVariableList;
import us.ihmc.simulationconstructionset.commands.ToggleKeyPointModeCommandListener;
import us.ihmc.simulationconstructionset.examples.FallingBrickRobot;
import us.ihmc.simulationconstructionset.graphics.GraphicsDynamicGraphicsObject;
import us.ihmc.simulationconstructionset.gui.DynamicGraphicMenuManager;
import us.ihmc.simulationconstructionset.gui.GraphArrayWindow;
import us.ihmc.simulationconstructionset.gui.StandardGUIActions;
import us.ihmc.simulationconstructionset.gui.StandardSimulationGUI;
import us.ihmc.simulationconstructionset.gui.ViewportWindow;
import us.ihmc.simulationconstructionset.gui.camera.CameraTrackAndDollyYoVariablesHolder;
import us.ihmc.simulationconstructionset.gui.config.GraphGroupList;
import us.ihmc.simulationconstructionset.physics.CollisionArbiter;
import us.ihmc.simulationconstructionset.physics.CollisionHandler;
import us.ihmc.simulationconstructionset.physics.CollisionShapeFactory;
import us.ihmc.simulationconstructionset.physics.ScsCollisionConfigure;
import us.ihmc.simulationconstructionset.physics.ScsCollisionDetector;
import us.ihmc.simulationconstructionset.physics.ScsPhysics;
import us.ihmc.simulationconstructionset.physics.collision.CollisionDetectionResult;
import us.ihmc.simulationconstructionset.physics.collision.DefaultCollisionHandler;
import us.ihmc.simulationconstructionset.physics.collision.DefaultCollisionVisualizer;
import us.ihmc.simulationconstructionset.physics.collision.simple.DoNothingCollisionArbiter;
import us.ihmc.simulationconstructionset.robotcommprotocol.RobotSocketConnection;
import us.ihmc.tools.thread.ThreadTools;

@ContinuousIntegrationPlan(categories = {IntegrationCategory.UI})
public class SimulationConstructionSetUsingDirectCallsTest
{
   // Note: some of the tests assume that:
   // - the simpleRobot has been used to create the SimulationConstructionSet instance
   // - the registry "simpleRegistry" is empty

   private static final long CLOSING_SLEEP_TIME = 2000;
   private static final long THREAD_SLEEP_TIME = 1000;
   private static final String SCS_VERSION = "12.06.22";
   private static final String TEST_DIRECTORY = "testResources/us/ihmc/simulationconstructionset/simulationConstructionSetUsingDirectCallsTest/";

   private static double epsilon = 1e-10;

   private final int recordFrequency = 10;
   private final int index = 15;
   private final int inputPoint = 10;
   private final int outputPoint = 50;
   private final int middleIndex = outputPoint - 1;
   private final int keyPoint = Math.round((inputPoint + outputPoint) / 2);
   private final int nonMiddleIndex = outputPoint + 1;
   private final int ticksIncrease = 11;
   private final int numberOfSimulationTicks = 2000;
   private final int graphGroupNumberOfColumns = 2;
   private final int numberOfTicksBeforeUpdatingGraphs = 2;
   private final int numberOfTicksBeforeUpdatingGraphs2 = 5;
   private final int indexStep = 2;
   private final double simulateDT = 0.001;
   private final double simulateTime = 1.0;
   private final double recordDT = 0.05;
   private final double recordFreq = computeRecordFreq(recordDT, simulateDT);
   private final double realTimeRate = 0.75;
   private final double frameRate = 0.01;
   private final double recomputedSecondsPerFrameRate = recomputeTiming(simulateDT, recordFreq, realTimeRate, frameRate);
   private final double cameraFieldOfView = 1.5;
   private final double secondsOfSimulation = 10;
   private final double simulateDurationInSeconds = 2.5;
   private final double[] cameraDollyOffsetXYZValues = { 1.2, 2.2, 3.2 };
   private final double[] cameraTrackingOffsetXYZValues = { 1.5, 2.5, 3.5 };
   private final double[] cameraDollyXYZVarValues = { 1.5, 1.6, 1.7 };
   private final double[] cameraTrackingXYZVarValues = { 0.5, 0.6, 0.7 };
   private final double[] cameraFixXYZValues = { 1.8, 2.8, 3.8 };
   private final double[] cameraFixXYZValues2 = { 1.1, 2.1, 3.1 };
   private final double[] cameraPositionXYZValues = { 1.3, 2.3, 3.3 };
   private final double[] cameraClipDistancesNearFarValues = { 1.75, 2.75 };
   private final double[] variableGroup1Values = { 1.3, 2.3, 3.3 };
   private final double[] variableGroup2Values = { 1.5, 1.6, 1.7 };
   private final double[] variableGroup3Values = { 5.5, 5.6, 5.7 };
   private final double[] variableGroup4Values = { 8.5, 8.6, 8.7 };
   private final double[] variableGroup5Values = { 9.5, 9.6, 9.7 };

   private final Point location = new Point(25, 50);
   private final FallingBrickRobot simpleRobot = new FallingBrickRobot();
   private final String rootRegistryName = "root";
   private final String simpleRegistryName = "simpleRegistry";
   private final String searchString = "d";
   private final String searchStringStart = "q";
   private final String[] variableGroup1 = new String[] { "simpleVar11", "simpleVar12", "simpleVar13" };
   private final String[] variableGroup2 = new String[] { "simpleVar21", "simpleVar22", "simpleVar23" };
   private final String[] variableGroup3 = new String[] { "simpleVar31", "simpleVar32", "simpleVar33" };
   private final String[] variableGroup4 = new String[] { "simpleVar41", "simpleVar42", "simpleVar43" };
   private final String[] variableGroup5 = new String[] { "simpleVar51", "simpleVar52", "simpleVar53" };
   private final String[] cameraTrackingXYZVarNames = new String[] { "simpleCameraTrackingVarNameX", "simpleCameraTrackingVarNameY", "simpleCameraTrackingVarNameZ" };
   private final String[] cameraDollyXYZVarNames = new String[] { "simpleCameraDollyVarNameX", "simpleCameraDollyVarNameY", "simpleCameraDollyVarNameZ" };
   private final String configurationName = "simpleConfigurationName";
   private final String cameraConfigurationName = "simpleCameraConfigurationName";
   private final String viewportConfigurationName = "simpleViewportConfiguration";
   private final String varGroupName = "simpleVarGroup";
   private final String varGroupName2 = "simpleVarGroup2";
   private final String varGroupName3 = "simpleVarGroup3";
   private final String graphGroupName = "simpleGraphGroup";
   private final String graphGroupName2 = "simpleGraphGroup2";
   private final String graphGroupName3 = "simpleGraphGroup3";
   private final String graphGroupName4 = "simpleGraphGroup4";
   private final String graphGroupName5 = "simpleGraphGroup5";
   private final String entryBoxGroupName = "simpleEntryBoxGroup";
   private final String entryBoxGroupName2 = "simpleEntryBoxGroup2";
   private final String entryBoxGroupName3 = "simpleEntryBoxGroup3";
   private final String[] graphConfigurationNames = { "simpleGraphConfiguration", "simpleGraphConfiguration2" };
   private final String extraPanelConfigurationName = "simpleExtraPanelConfigurationName";
   private final String simpleComponentName =  "simpleComponent";
   private final String runningName = "simpleRunningName";
   private final String yoGraphicsListName = "simpleDynamicGraphicObjectsList";
   private final String[][] graphGroupVars = { cameraTrackingXYZVarNames, cameraDollyXYZVarNames };
   private final String[][][] graphGroupVarsWithConfig = { { cameraTrackingXYZVarNames, { "config_1" } }, { cameraDollyXYZVarNames, { "config_2" } } };
   private final String simpleRobotFirstVariableName = getFirstVariableNameFromRobotRegistry(simpleRobot);
   private final String simpleRobotLastVariableName = getLastVariableNameFromRobotRegistry(simpleRobot);
   private final String simpleRobotRegistryNameSpace = getRegistryNameSpaceFromRobot(simpleRobot);
   private final String[] regularExpressions = new String[] { "gc.*.fs" };
   private final Dimension dimension = new Dimension(250, 350);


   private NameSpace simpleRegistryNameSpace;
   private YoVariableRegistry simpleRegistry;
   private YoVariableRegistry dummyRegistry;
   private Link staticLink;
   private Graphics3DObject staticLinkGraphics;
   private Graphics3DNodeType graphics3DNodeType;
   private ExternalForcePoint simpleExternalForcePoint;
   private YoGraphic yoGraphic;
   private BooleanYoVariable exitActionListenerHasBeenNotified;
   private BooleanYoVariable simulationRewoundListenerHasBeenNotified;
   private BooleanYoVariable simulationDoneListenerHasBeenNotified;
   private BooleanYoVariable setSimulationDoneCriterion;
   private ExtraPanelConfiguration extraPanelConfiguration;
   private CameraConfiguration cameraConfiguration;
   private ViewportConfiguration viewportConfiguration;
   private GraphConfiguration[] graphConfigurations;
   private DoubleYoVariable realTimeRateInSCS;
   private BooleanYoVariable processDataHasBeenCalled;
   private BooleanYoVariable toggleKeyPointModeCommandListenerHasBeenCalled;
   private YoGraphicsListRegistry yoGraphicsListRegistry;
   private DynamicGraphicMenuManager dynamicGraphicMenuManager;
   private ScsPhysics simpleScsPhysics;
   private SimulationConstructionSet scs;

   @BeforeClass
   public static void setUpOnce()
   {
      FailOnThreadViolationRepaintManager.install();
   }

   @Before
   public void createAndStartSCSWithRobot()
   {
      simpleRegistryNameSpace = new NameSpace(rootRegistryName + "." + simpleRegistryName);
      simpleRegistry = new YoVariableRegistry(simpleRegistryName);
      dummyRegistry = new YoVariableRegistry("dummyRegistry");
      staticLink = new Link("simpleLink");
      staticLinkGraphics = staticLink.getLinkGraphics();
      graphics3DNodeType = Graphics3DNodeType.GROUND;
      simpleExternalForcePoint = new ExternalForcePoint("simpleExternalForcePoint", dummyRegistry);
      yoGraphic = new YoGraphicVector("simpleDynamicGraphicObject", simpleExternalForcePoint.getYoPosition(), simpleExternalForcePoint.getYoForce(), 1.0/50.0);
      exitActionListenerHasBeenNotified = new BooleanYoVariable("exitActionListenerHasBeenNotified", dummyRegistry);
      simulationRewoundListenerHasBeenNotified = new BooleanYoVariable("simulationRewoundListenerHasBeenNotified", dummyRegistry);
      simulationDoneListenerHasBeenNotified = new BooleanYoVariable("simulationDoneListenerHasBeenNotified", dummyRegistry);
      setSimulationDoneCriterion = new BooleanYoVariable("setSimulationDoneCriterion", dummyRegistry);
      extraPanelConfiguration = createExtraPanelConfigurationWithPanel(extraPanelConfigurationName);
      cameraConfiguration = createCameraConfiguration(cameraConfigurationName);
      viewportConfiguration = createViewportConfiguration(viewportConfigurationName);
      viewportConfiguration.addCameraView("Back View", 0, 0, 1, 1);

      graphConfigurations = createGraphConfigurations(graphConfigurationNames);
      realTimeRateInSCS = new DoubleYoVariable("realTimeRate", dummyRegistry);
      processDataHasBeenCalled = new BooleanYoVariable("processDataHasBeenCalled", dummyRegistry);
      toggleKeyPointModeCommandListenerHasBeenCalled = new BooleanYoVariable("toggleKeyPointModeCommandListenerHasBeenCalled", dummyRegistry);
      yoGraphicsListRegistry = createYoGraphicsListRegistryWithObject();
      dynamicGraphicMenuManager = new DynamicGraphicMenuManager();

      scs = new SimulationConstructionSet(simpleRobot);
      simpleScsPhysics = createScsPhysics();
      scs.setFrameMaximized();
      scs.startOnAThread();
   }

   @After
   public void closeSCS()
   {
      ThreadTools.sleep(CLOSING_SLEEP_TIME);
      scs.closeAndDispose();
      scs = null;

      simpleRegistryNameSpace = null;
      simpleRegistry = null;
      dummyRegistry = null;
      staticLink = null;
      staticLinkGraphics = null;
      graphics3DNodeType = null;
      simpleExternalForcePoint = null;
      yoGraphic = null;
      exitActionListenerHasBeenNotified = null;
      simulationRewoundListenerHasBeenNotified = null;
      simulationDoneListenerHasBeenNotified = null;
      setSimulationDoneCriterion = null;
      extraPanelConfiguration = null;
      cameraConfiguration = null;
      viewportConfiguration = null;
      graphConfigurations = null;
      realTimeRateInSCS = null;
      processDataHasBeenCalled = null;
      toggleKeyPointModeCommandListenerHasBeenCalled = null;
      yoGraphicsListRegistry = null;
      dynamicGraphicMenuManager = null;
      simpleScsPhysics = null;
   }

	@ContinuousIntegrationTest(estimatedDuration = 2.8)
	@Test(timeout = 30000)
   public void testSimulationConstructionSetMiscellaneous() throws AWTException
   {
      Robot[] robotFromSCS = scs.getRobots();
      assertEquals(simpleRobot, robotFromSCS[0]);

      YoVariableRegistry rootRegistryFromSCS = scs.getRootRegistry();
      String rootRegistryNameFromSCS = rootRegistryFromSCS.getName();
      assertEquals(rootRegistryName, rootRegistryNameFromSCS);

      scs.addYoVariableRegistry(simpleRegistry);
      YoVariableRegistry simpleRegistryFromSCS = scs.getRootRegistry().getRegistry(simpleRegistryNameSpace);
      assertEquals(simpleRegistry, simpleRegistryFromSCS);

      ExitActionListener exitActionListener = createExitActionListener();
      scs.attachExitActionListener(exitActionListener);
      exitActionListenerHasBeenNotified.set(false);
      scs.notifyExitActionListeners();
      assertTrue(exitActionListenerHasBeenNotified.getBooleanValue());

      scs.setGraphsUpdatedDuringPlayback(false);
      boolean isGraphsUpdatedDuringPlaybackFromSCS = scs.areGraphsUpdatedDuringPlayback();
      assertFalse(isGraphsUpdatedDuringPlaybackFromSCS);

      scs.setGraphsUpdatedDuringPlayback(true);
      boolean isGraphsUpdatedDuringPlaybackFromSCS2 = scs.areGraphsUpdatedDuringPlayback();
      assertTrue(isGraphsUpdatedDuringPlaybackFromSCS2);

      scs.setScrollGraphsEnabled(true);
      boolean isScrollGraphsEnabled = scs.isSafeToScroll();
      assertTrue(isScrollGraphsEnabled);

      scs.setScrollGraphsEnabled(false);
      boolean isScrollGraphsEnabled2 = scs.isSafeToScroll();
      assertFalse(isScrollGraphsEnabled2);

      RobotSocketConnection robotSocketConnectionFromSCS = scs.allowTCPConnectionToHost("host");
      assertNotNull(robotSocketConnectionFromSCS);

      NewDataListener newDataListener = createNewDataListener();
      RobotSocketConnection robotSocketConnectionFromSCS2 = scs.allowTCPConnectionToHost("host2", newDataListener);
      assertNotNull(robotSocketConnectionFromSCS2);

      boolean initialKeyPointStatus = scs.isKeyPointModeToggled();
      scs.toggleKeyPointMode();
      boolean finalKeyPointStatus = scs.isKeyPointModeToggled();
      assertBooleansAreOpposite(initialKeyPointStatus, finalKeyPointStatus);

      scs.setRunName(runningName);
      String runningNameFromSCS = scs.getRunningName();
      assertEquals(runningName, runningNameFromSCS);

      String scsVersion = scs.getVersion();
      assertEquals(SCS_VERSION, scsVersion);

      scs.disableSystemExit();
      scs.enableSystemExit();
      boolean systemExitDisabledFromSCS = scs.systemExitDisabled();
      assertFalse(systemExitDisabledFromSCS);

      scs.enableSystemExit();
      scs.disableSystemExit();
      boolean systemExitDisabledFromSCS2 = scs.systemExitDisabled();
      assertTrue(systemExitDisabledFromSCS2);

      scs.initPhysics(simpleScsPhysics);
      ScsPhysics scsPhysicsFromSCS = scs.getPhysics();
      assertEquals(simpleScsPhysics, scsPhysicsFromSCS);
   }

	@ContinuousIntegrationTest(estimatedDuration = 2.6)
	@Test(timeout = 30000)
   public void testSimulationManagement()
   {
      double startTime = scs.getTime();
      simulateForTime(scs, simulateTime);
      double endTime = scs.getTime();
      assertEquals(simulateTime, endTime - startTime, 1e-7);

      scs.stop();
      scs.simulate(numberOfSimulationTicks);
      boolean isSimulatingFromSCS = scs.isSimulating();
      assertTrue(isSimulatingFromSCS);

      scs.stop();
      scs.simulate(numberOfSimulationTicks);
      scs.stop();
      boolean isSimulatingFromSCS2 = scs.isSimulating();
      assertFalse(isSimulatingFromSCS2);

      scs.stop();
      scs.simulate(secondsOfSimulation);
      boolean isSimulatingFromSCS3 = scs.isSimulating();
      assertTrue(isSimulatingFromSCS3);

      scs.stop();
      scs.simulate();
      boolean isSimulatingFromSCS4 = scs.isSimulating();
      assertTrue(isSimulatingFromSCS4);

      scs.stop();
      scs.play();
      boolean isPlayingFromSCS = scs.isPlaying();
      scs.stop();
      assertTrue(isPlayingFromSCS);

      scs.setFastSimulate(true);
      boolean isFastSimulateFromSCS = scs.isFastSimulateEnabled();
      assertTrue(isFastSimulateFromSCS);

      scs.setFastSimulate(false);
      boolean isFastSimulateFromSCS2 = scs.isFastSimulateEnabled();
      assertFalse(isFastSimulateFromSCS2);

      scs.setFastSimulate(true, numberOfTicksBeforeUpdatingGraphs);
      boolean isFastSimulateFromSCS3 = scs.isFastSimulateEnabled();
      int numberOfTicksBeforeUpdatingGraphsFromSCS = scs.getNumberOfTicksBeforeUpdatingGraphs();
      assertTrue(isFastSimulateFromSCS3);
      assertEquals(numberOfTicksBeforeUpdatingGraphs, numberOfTicksBeforeUpdatingGraphsFromSCS);

      scs.setFastSimulate(false, numberOfTicksBeforeUpdatingGraphs2);
      boolean isFastSimulateFromSCS4 = scs.isFastSimulateEnabled();
      int numberOfTicksBeforeUpdatingGraphsFromSCS2 = scs.getNumberOfTicksBeforeUpdatingGraphs();
      assertFalse(isFastSimulateFromSCS4);
      assertEquals(numberOfTicksBeforeUpdatingGraphs2, numberOfTicksBeforeUpdatingGraphsFromSCS2);

      double initialTime = scs.getRobots()[0].getTime();
      double DT = scs.getDT();
      callSCSMethodSimulateOneTimeStep(scs);
      double finalTime = scs.getRobots()[0].getTime();
      assertEquals(initialTime + DT, finalTime, epsilon);

      double expectedFinalTime2 = getExpectedFinalTime(scs);
      callSCSMethodSimulateOneRecordStep(scs);
      double finalTime2 = scs.getRobots()[0].getTime();
      assertEquals(expectedFinalTime2, finalTime2, epsilon);

      double expectedFinalTime3 = getExpectedFinalTime(scs);
      callSCSMethodSimulateOneRecordStepNow(scs);
      double finalTime3 = scs.getRobots()[0].getTime();
      assertEquals(expectedFinalTime3, finalTime3, epsilon);

      scs.setSimulateDuration(simulateDurationInSeconds);
      double simulateDurationInSecondsFromSCS = scs.getSimulateDuration();
      assertEquals(simulateDurationInSeconds, simulateDurationInSecondsFromSCS, epsilon);
   }

   @Ignore // Only run this one locally since it doesn't work on Bamboo on Linux necessarily.

	@ContinuousIntegrationTest(estimatedDuration = 0.1)
	@Test(timeout=300000)
   public void testFrameMethodsThatOnlyWorkOnSomeOperatingSystems()
   {
      scs.setFrameSize(dimension);
      ThreadTools.sleep(THREAD_SLEEP_TIME);
      Dimension dimensionFromSCS = scs.getJFrame().getBounds().getSize();
      assertEquals(dimension.height, dimensionFromSCS.height, epsilon);
      assertEquals(dimension.width, dimensionFromSCS.width, epsilon);

      scs.setFrameLocation(location.x, location.y);
      ThreadTools.sleep(THREAD_SLEEP_TIME);
      Point locationFromSCS = scs.getJFrame().getLocation();
      assertEquals(location.x, locationFromSCS.x, epsilon);
      assertEquals(location.y, locationFromSCS.y, epsilon);

      scs.setFrameMaximized();
      ThreadTools.sleep(THREAD_SLEEP_TIME);
      int frameStateFromSCS = getExtendedStateFromSCS(scs);
      assertEquals(Frame.MAXIMIZED_BOTH, frameStateFromSCS, epsilon);

      scs.setFrameAlwaysOnTop(true);
      ThreadTools.sleep(THREAD_SLEEP_TIME);
      boolean alwaysOnTopFromSCS = scs.getJFrame().isAlwaysOnTop();
      assertEquals(true, alwaysOnTopFromSCS);

      scs.setFrameAlwaysOnTop(false);
      ThreadTools.sleep(THREAD_SLEEP_TIME);
      alwaysOnTopFromSCS = scs.getJFrame().isAlwaysOnTop();
      assertEquals(false, alwaysOnTopFromSCS);

      scs.maximizeMainWindow();
      ThreadTools.sleep(THREAD_SLEEP_TIME);
      int frameStateFromSCS2 = getExtendedStateFromSCS(scs);
      assertEquals(Frame.MAXIMIZED_BOTH, frameStateFromSCS2, epsilon);
   }

	@ContinuousIntegrationTest(estimatedDuration = 12.7)
	@Test(timeout = 64000)
   public void testFrameMethods()
   {
      ThreadTools.sleep(THREAD_SLEEP_TIME);

      scs.createNewGraphWindow();
      ThreadTools.sleep(THREAD_SLEEP_TIME);
      GraphArrayWindow graphArrayWindowFromSCS = scs.getGraphArrayWindow("Unnamed");
      assertNotNull(graphArrayWindowFromSCS);

      scs.createNewGraphWindow("simpleGraphArrayWindow");
      ThreadTools.sleep(THREAD_SLEEP_TIME);
      GraphArrayWindow graphArrayWindowFromSCS2 = scs.getGraphArrayWindow("simpleGraphArrayWindow");
      assertNotNull(graphArrayWindowFromSCS2);

      scs.createNewGraphWindow("simpleGraphArrayWindow2", 0, false);
      ThreadTools.sleep(THREAD_SLEEP_TIME);
      GraphArrayWindow graphArrayWindowFromSCS3 = scs.getGraphArrayWindow("simpleGraphArrayWindow2");
      assertNotNull(graphArrayWindowFromSCS3);

      scs.createNewViewportWindow();
      ThreadTools.sleep(THREAD_SLEEP_TIME);
      ViewportWindow viewportWindowFromSCS = scs.getViewportWindow("Unnamed");
      assertNotNull(viewportWindowFromSCS);

      scs.createNewViewportWindow("simpleViewportWindow");
      ThreadTools.sleep(THREAD_SLEEP_TIME);
      ViewportWindow viewportWindowFromSCS2 = scs.getViewportWindow("simpleViewportWindow");
      assertNotNull(viewportWindowFromSCS2);

      scs.createNewViewportWindow("simpleViewportWindow", 0, false);
      ThreadTools.sleep(THREAD_SLEEP_TIME);
      ViewportWindow viewportWindowFromSCS3 = scs.getViewportWindow("simpleViewportWindow");
      assertNotNull(viewportWindowFromSCS3);

      scs.showViewport();
      scs.hideViewport();
      ThreadTools.sleep(THREAD_SLEEP_TIME);
      boolean isViewportHidden = scs.isViewportHidden();
      assertTrue(isViewportHidden);

      scs.hideViewport();
      scs.showViewport();
      ThreadTools.sleep(THREAD_SLEEP_TIME);
      boolean isViewportHidden2 = scs.isViewportHidden();
      assertFalse(isViewportHidden2);

      Component component = new Button();
      scs.addExtraJpanel(component, simpleComponentName, false);
      ThreadTools.sleep(THREAD_SLEEP_TIME);
      Component componentFromSCS =  scs.getExtraPanel(simpleComponentName);
      assertEquals(component, componentFromSCS);
   }

	@ContinuousIntegrationTest(estimatedDuration = 2.5)
	@Test(timeout = 30000)
   public void testCameraMethods()
   {
      scs.setCameraTracking(true, false, false, false);
      boolean isCameraTracking = scs.getGUI().getCamera().isTracking();
      boolean isCameraTrackingX = scs.getGUI().getCamera().isTrackingX();
      boolean isCameraTrackingY = scs.getGUI().getCamera().isTrackingY();
      boolean isCameraTrackingZ = scs.getGUI().getCamera().isTrackingZ();
      assertTrue(isCameraTracking);
      assertFalse(isCameraTrackingX);
      assertFalse(isCameraTrackingY);
      assertFalse(isCameraTrackingZ);

      scs.setCameraTracking(false, true, false, false);
      boolean isCameraTracking2 = scs.getGUI().getCamera().isTracking();
      boolean isCameraTrackingX2 = scs.getGUI().getCamera().isTrackingX();
      boolean isCameraTrackingY2 = scs.getGUI().getCamera().isTrackingY();
      boolean isCameraTrackingZ2 = scs.getGUI().getCamera().isTrackingZ();
      assertFalse(isCameraTracking2);
      assertTrue(isCameraTrackingX2);
      assertFalse(isCameraTrackingY2);
      assertFalse(isCameraTrackingZ2);

      scs.setCameraTracking(false, false, true, false);
      boolean isCameraTracking3 = scs.getGUI().getCamera().isTracking();
      boolean isCameraTrackingX3 = scs.getGUI().getCamera().isTrackingX();
      boolean isCameraTrackingY3 = scs.getGUI().getCamera().isTrackingY();
      boolean isCameraTrackingZ3 = scs.getGUI().getCamera().isTrackingZ();
      assertFalse(isCameraTracking3);
      assertFalse(isCameraTrackingX3);
      assertTrue(isCameraTrackingY3);
      assertFalse(isCameraTrackingZ3);

      scs.setCameraTracking(false, false, false, true);
      boolean isCameraTracking4 = scs.getGUI().getCamera().isTracking();
      boolean isCameraTrackingX4 = scs.getGUI().getCamera().isTrackingX();
      boolean isCameraTrackingY4 = scs.getGUI().getCamera().isTrackingY();
      boolean isCameraTrackingZ4 = scs.getGUI().getCamera().isTrackingZ();
      assertFalse(isCameraTracking4);
      assertFalse(isCameraTrackingX4);
      assertFalse(isCameraTrackingY4);
      assertTrue(isCameraTrackingZ4);

      scs.setCameraDolly(true, false, false, false);
      boolean isCameraDolly = scs.getGUI().getCamera().isDolly();
      boolean isCameraDollyX = scs.getGUI().getCamera().isDollyX();
      boolean isCameraDollyY = scs.getGUI().getCamera().isDollyY();
      boolean isCameraDollyZ = scs.getGUI().getCamera().isDollyZ();
      assertTrue(isCameraDolly);
      assertFalse(isCameraDollyX);
      assertFalse(isCameraDollyY);
      assertFalse(isCameraDollyZ);

      scs.setCameraDolly(false, true, false, false);
      boolean isCameraDolly2 = scs.getGUI().getCamera().isDolly();
      boolean isCameraDollyX2 = scs.getGUI().getCamera().isDollyX();
      boolean isCameraDollyY2 = scs.getGUI().getCamera().isDollyY();
      boolean isCameraDollyZ2 = scs.getGUI().getCamera().isDollyZ();
      assertFalse(isCameraDolly2);
      assertTrue(isCameraDollyX2);
      assertFalse(isCameraDollyY2);
      assertFalse(isCameraDollyZ2);

      scs.setCameraDolly(false, false, true, false);
      boolean isCameraDolly3 = scs.getGUI().getCamera().isDolly();
      boolean isCameraDollyX3 = scs.getGUI().getCamera().isDollyX();
      boolean isCameraDollyY3 = scs.getGUI().getCamera().isDollyY();
      boolean isCameraDollyZ3 = scs.getGUI().getCamera().isDollyZ();
      assertFalse(isCameraDolly3);
      assertFalse(isCameraDollyX3);
      assertTrue(isCameraDollyY3);
      assertFalse(isCameraDollyZ3);

      scs.setCameraDolly(false, false, false, true);
      boolean isCameraDolly4 = scs.getGUI().getCamera().isDolly();
      boolean isCameraDollyX4 = scs.getGUI().getCamera().isDollyX();
      boolean isCameraDollyY4 = scs.getGUI().getCamera().isDollyY();
      boolean isCameraDollyZ4 = scs.getGUI().getCamera().isDollyZ();
      assertFalse(isCameraDolly4);
      assertFalse(isCameraDollyX4);
      assertFalse(isCameraDollyY4);
      assertTrue(isCameraDollyZ4);

      addDoubleYoVariablesInSCSRegistry(cameraTrackingXYZVarNames, cameraTrackingXYZVarValues, scs);
      scs.setCameraTrackingVars(cameraTrackingXYZVarNames[0], cameraTrackingXYZVarNames[1], cameraTrackingXYZVarNames[2]);
      double[] cameraTrackingXYZVarsFromSCS = getCameraTrackingXYZVars(scs);
      assertArrayEquals(cameraTrackingXYZVarValues, cameraTrackingXYZVarsFromSCS, epsilon);

      addDoubleYoVariablesInSCSRegistry(cameraDollyXYZVarNames, cameraDollyXYZVarValues, scs);
      scs.setCameraDollyVars(cameraDollyXYZVarNames[0], cameraDollyXYZVarNames[1], cameraDollyXYZVarNames[2]);
      double[] cameraDollyXYZVarsFromSCS = getCameraDollyXYZVars(scs);
      assertArrayEquals(cameraDollyXYZVarValues, cameraDollyXYZVarsFromSCS, epsilon);

      scs.setCameraTrackingOffsets(cameraTrackingOffsetXYZValues[0], cameraTrackingOffsetXYZValues[1], cameraTrackingOffsetXYZValues[2]);
      double[] cameraTrackingOffsetXYZValuesFromSCS = getCameraTrackingOffsetXYZValues(scs);
      assertArrayEquals(cameraTrackingOffsetXYZValues, cameraTrackingOffsetXYZValuesFromSCS, epsilon);

      scs.setCameraDollyOffsets(cameraDollyOffsetXYZValues[0], cameraDollyOffsetXYZValues[1], cameraDollyOffsetXYZValues[2]);
      double[] cameraDollyOffsetXYZValuesFromSCS = getCameraDollyOffsetXYZValues(scs);
      assertArrayEquals(cameraDollyOffsetXYZValues, cameraDollyOffsetXYZValuesFromSCS, epsilon);

      scs.setCameraFix(cameraFixXYZValues[0], cameraFixXYZValues[1], cameraFixXYZValues[2]);
      double[] cameraFixXYZValuesFromSCS = getCameraFixXYZValues(scs);
      assertArrayEquals(cameraFixXYZValues, cameraFixXYZValuesFromSCS, epsilon);

      scs.setCameraPosition(cameraPositionXYZValues[0], cameraPositionXYZValues[1], cameraPositionXYZValues[2]);
      double[] cameraPositionXYZValuesFromSCS = getCameraPositionXYZValues(scs);
      assertArrayEquals(cameraPositionXYZValues, cameraPositionXYZValuesFromSCS, epsilon);

      scs.setupCamera(cameraConfiguration);
      String[] cameraConfigurationNamesFromSCS = getCameraConfigurationNames(scs);
      assertArrayOfStringsContainsTheString(cameraConfigurationNamesFromSCS, cameraConfigurationName);

      scs.setupViewport(viewportConfiguration);
      String[] viewportConfigurationNamesFromSCS = getViewportConfigurationNames(scs);
      assertArrayOfStringsContainsTheString(viewportConfigurationNamesFromSCS, viewportConfigurationName);

      scs.selectViewport(viewportConfigurationName);
      boolean isCurrentView = isCurrentView(scs, viewportConfigurationName);
      assertTrue(isCurrentView);

      scs.setupGraphConfigurations(graphConfigurations);
      String[] graphConfigurationNamesFromSCS = getGraphConfigurationListNames(scs);
      assertArrayOfStringsContainsTheStrings(graphConfigurationNamesFromSCS, graphConfigurationNames);

      scs.setClipDistances(cameraClipDistancesNearFarValues[0], cameraClipDistancesNearFarValues[1]);
      double[] cameraClipDistancesNearFarValuesFromSCS = getCameraNearFarValues(scs);
      assertArrayEquals(cameraClipDistancesNearFarValues, cameraClipDistancesNearFarValuesFromSCS, epsilon);

      scs.setFieldOfView(cameraFieldOfView);
      double cameraFieldOfViewFromSCS = getCameraFieldOfView(scs);
      assertEquals(cameraFieldOfView, cameraFieldOfViewFromSCS, epsilon);

      setInputAndOutputPointsWithoutCroppingInSCS(scs, inputPoint, outputPoint);
      scs.setIndex(keyPoint);
      scs.addCameraKey();
      Integer keyPointFromSCS = scs.getCameraKeyPoints().get(0);
      assertEquals(keyPoint, keyPointFromSCS.intValue(), epsilon);

      setInputAndOutputPointsWithoutCroppingInSCS(scs, inputPoint, outputPoint);
      scs.setIndex(keyPoint);
      scs.addCameraKey();
      scs.removeCameraKey();
      ArrayList<Integer> keyPointFromSCS2 = scs.getCameraKeyPoints();
      assertEquals(0, keyPointFromSCS2.size(), epsilon);

      setInputAndOutputPointsWithoutCroppingInSCS(scs, inputPoint, outputPoint);
      scs.setCameraFix(cameraFixXYZValues[0], cameraFixXYZValues[1], cameraFixXYZValues[2]);
      scs.setIndex(inputPoint);
      scs.addCameraKey();
      scs.setCameraFix(cameraFixXYZValues2[0], cameraFixXYZValues2[1], cameraFixXYZValues2[2]);
      scs.setIndex(keyPoint);
      scs.addCameraKey();
      scs.nextCameraKey();
      double[] cameraFixXYZValuesFromSCS2 = getCameraFixXYZValues(scs);
      assertArrayEquals(cameraFixXYZValues2, cameraFixXYZValuesFromSCS2, epsilon);
      scs.previousCameraKey();
      double[] cameraFixXYZValuesFromSCS3 = getCameraFixXYZValues(scs);
      assertArrayEquals(cameraFixXYZValues, cameraFixXYZValuesFromSCS3, epsilon);

      boolean initialCameraKeyModeStatus = getCameraKeyMode(scs);
      scs.toggleCameraKeyMode();
      boolean finalCameraKeyModeStatus = getCameraKeyMode(scs);
      assertBooleansAreOpposite(initialCameraKeyModeStatus, finalCameraKeyModeStatus);
   }

	@ContinuousIntegrationTest(estimatedDuration = 2.6)
	@Test(timeout = 30000)
   public void test3DGraphicsMethods()
   {
      scs.addYoGraphicsListRegistry(yoGraphicsListRegistry);

      Graphics3DNode graphics3DNodeFromSCS = scs.addStaticLinkGraphics(staticLinkGraphics);
      assertNotNull(graphics3DNodeFromSCS);

      Graphics3DNode graphics3DNodeFromSCS2 = scs.addStaticLinkGraphics(staticLinkGraphics, graphics3DNodeType);
      assertNotNull(graphics3DNodeFromSCS2);

      Graphics3DNode graphics3DNodeFromSCS3 = scs.addStaticLink(staticLink);
      assertNotNull(graphics3DNodeFromSCS3);

      GraphicsDynamicGraphicsObject graphicsDynamicGraphicsObjectFromSCS = scs.addYoGraphic(yoGraphic);
      assertNotNull(graphicsDynamicGraphicsObjectFromSCS);

      GraphicsDynamicGraphicsObject graphicsDynamicGraphicsObjectFromSCS2 = scs.addYoGraphic(yoGraphic, true);
      assertNotNull(graphicsDynamicGraphicsObjectFromSCS2);

      GraphicsDynamicGraphicsObject graphicsDynamicGraphicsObjectFromSCS3 = scs.addYoGraphic(yoGraphic, false);
      assertNotNull(graphicsDynamicGraphicsObjectFromSCS3);

      scs.setGroundVisible(false);
      boolean isGroundVisibleFromSCS = stateIfTerrainIsVisible(scs);
      assertTrue(isGroundVisibleFromSCS);

      scs.setGroundVisible(true);
      boolean isGroundVisibleFromSCS2 = stateIfTerrainIsVisible(scs);
      assertTrue(isGroundVisibleFromSCS2);

      ArrayList<YoGraphicsListRegistry> yoGraphicListRegistriesFromSCS = scs.getDynamicGraphicObjectsListRegistries();
      assertArrayOfObjectsContainsTheObject(yoGraphicListRegistriesFromSCS, yoGraphicsListRegistry);

      scs.setDynamicGraphicObjectsListVisible(yoGraphicsListName, true);
      scs.hideAllDynamicGraphicObjects();
      boolean yoGraphicsAreShowing = scs.checkAllDynamicGraphicObjectsListRegistriesAreShowing();
      assertFalse(yoGraphicsAreShowing);

      scs.hideAllDynamicGraphicObjects();
      scs.setDynamicGraphicObjectsListVisible(yoGraphicsListName, true);
      boolean yoGraphicsAreShowing2 = scs.checkAllDynamicGraphicObjectsListRegistriesAreShowing();
      assertTrue(yoGraphicsAreShowing2);

//      scs.setDynamicGraphicMenuManager(dynamicGraphicMenuManager);
//      DynamicGraphicMenuManager dynamicGraphicMenuManagerFromSCS =  scs.getDynamicGraphicMenuManager();
//      assertEquals(dynamicGraphicMenuManager, dynamicGraphicMenuManagerFromSCS);

      scs.disableGUIComponents();
      assertIfGUIComponentsAreDisableOrEnabled(scs, false);

      scs.enableGUIComponents();
      assertIfGUIComponentsAreDisableOrEnabled(scs, true);
   }

	@ContinuousIntegrationTest(estimatedDuration = 2.5)
	@Test(timeout = 30000)
   public void testGetVariableMethods() throws AWTException
   {
      ArrayList<YoVariable<?>> allVariablesFromRobot = simpleRobot.getAllVariables();
      ArrayList<YoVariable<?>> allVariablesFromSCS = scs.getAllVariables();
      assertEquals(allVariablesFromRobot, allVariablesFromSCS);

      int allVariablesArrayFromRobot = simpleRobot.getAllVariablesArray().length;
      int allVariablesArrayFromSCS = scs.getAllVariablesArray().length;
      assertEquals(allVariablesArrayFromRobot, allVariablesArrayFromSCS);

      YoVariable<?> yoVariableFromSCS = scs.getVariable(simpleRobotFirstVariableName);
      String variableNameFromSCS = yoVariableFromSCS.getName();
      assertEquals(simpleRobotFirstVariableName, variableNameFromSCS);

      YoVariable<?> yoVariableFromRobot = simpleRobot.getVariable(simpleRobotFirstVariableName);
      YoVariable<?> yoVariableFromSCS2 = scs.getVariable(simpleRobotRegistryNameSpace, simpleRobotFirstVariableName);
      assertEquals(yoVariableFromRobot, yoVariableFromSCS2);

      ArrayList<YoVariable<?>> yoVariableArrayFromRobot = simpleRobot.getVariables(simpleRobotRegistryNameSpace, simpleRobotFirstVariableName);
      ArrayList<YoVariable<?>> yoVariableArrayFromSCS = scs.getVariables(simpleRobotRegistryNameSpace, simpleRobotFirstVariableName);
      assertEquals(yoVariableArrayFromRobot, yoVariableArrayFromSCS);

      ArrayList<YoVariable<?>> yoVariableFromRobot2 = simpleRobot.getVariables(simpleRobotFirstVariableName);
      ArrayList<YoVariable<?>> yoVariableFromSCS3 = scs.getVariables(simpleRobotFirstVariableName);
      assertEquals(yoVariableFromRobot2, yoVariableFromSCS3);

      ArrayList<YoVariable<?>> yoVariableFromRobot3 = simpleRobot.getVariables(simpleRobotRegistryNameSpace);
      ArrayList<YoVariable<?>> yoVariableFromSCS4 = scs.getVariables(simpleRobotRegistryNameSpace);
      assertEquals(yoVariableFromRobot3, yoVariableFromSCS4);

      boolean hasUniqueVariableRobot = simpleRobot.hasUniqueVariable(simpleRobotFirstVariableName);
      boolean hasUniqueVariableSCS = scs.hasUniqueVariable(simpleRobotFirstVariableName);
      assertEquals(hasUniqueVariableRobot, hasUniqueVariableSCS);

      boolean hasUniqueVariableRobot2 = simpleRobot.hasUniqueVariable(simpleRobotRegistryNameSpace, simpleRobotFirstVariableName);
      boolean hasUniqueVariableSCS2 = scs.hasUniqueVariable(simpleRobotRegistryNameSpace, simpleRobotFirstVariableName);
      assertEquals(hasUniqueVariableRobot2, hasUniqueVariableSCS2);

      ArrayList<YoVariable<?>> arrayOfVariablesContainingRobot = getSimpleRobotVariablesThatContain(searchString, false, simpleRobot);
      ArrayList<YoVariable<?>> arrayOfVariablesContainingSCS = scs.getVariablesThatContain(searchString);
      assertEquals(arrayOfVariablesContainingRobot, arrayOfVariablesContainingSCS);

      ArrayList<YoVariable<?>> arrayOfVariablesContainingRobot2 = getSimpleRobotVariablesThatContain(searchString, true, simpleRobot);
      ArrayList<YoVariable<?>> arrayOfVariablesContainingSCS2 = scs.getVariablesThatContain(searchString, true);
      assertEquals(arrayOfVariablesContainingRobot2, arrayOfVariablesContainingSCS2);

      ArrayList<YoVariable<?>> arrayOfVariablesStartingRobot = getSimpleRobotVariablesThatStartWith(searchStringStart, simpleRobot);
      ArrayList<YoVariable<?>> arrayOfVariablesStartingSCS = scs.getVariablesThatStartWith(searchStringStart);
      assertEquals(arrayOfVariablesStartingRobot, arrayOfVariablesStartingSCS);

      String[] varNames = getVariableNamesGivenArrayListOfYoVariables(arrayOfVariablesContainingRobot);
      ArrayList<YoVariable<?>> arrayOfVariablesRegExprRobot = getSimpleRobotRegExpVariables(varNames, regularExpressions, simpleRobot);
      ArrayList<YoVariable<?>> arrayOfVariablesRegExprSCS = scs.getVars(varNames, regularExpressions);
      assertEquals(arrayOfVariablesRegExprRobot, arrayOfVariablesRegExprSCS);
   }

	@ContinuousIntegrationTest(estimatedDuration = 2.5)
	@Test(timeout = 30000)
   public void testTimingMethods() throws AWTException
   {
      scs.setDT(simulateDT, recordFrequency);
      double simulateDTFromSCS = scs.getDT();
      assertEquals(simulateDT, simulateDTFromSCS, epsilon);

      scs.setRecordDT(recordDT);
      double recordFreqFromSCS = scs.getRecordFreq();
      assertEquals(recordFreq, recordFreqFromSCS, epsilon);

      scs.setPlaybackRealTimeRate(realTimeRate);
      double realTimeRateFromSCS = scs.getPlaybackRealTimeRate();
      assertEquals(realTimeRate, realTimeRateFromSCS, epsilon);

      scs.setPlaybackDesiredFrameRate(frameRate);
      double frameRateFromSCS = scs.getPlaybackFrameRate();
      assertEquals(recomputedSecondsPerFrameRate, frameRateFromSCS, epsilon);

      int ticksPerCycle = computeTicksPerPlayCycle(simulateDT, recordFreq, realTimeRate, frameRate);
      double ticksPerCycleFromSCS = scs.getTicksPerPlayCycle();
      assertEquals(ticksPerCycle, ticksPerCycleFromSCS, epsilon);

      scs.setTime(Math.PI);
      double timeFromSCS = scs.getTime();
      assertEquals(Math.PI, timeFromSCS, epsilon);
   }

	@ContinuousIntegrationTest(estimatedDuration = 3.0)
	@Test(timeout = 30000)
   public void testSimulationTickControl()
   {
      scs.setIndex(index);
      int indexFromSCS = scs.getIndex();
      assertEquals(index, indexFromSCS, epsilon);

      createSimulationRewoundListenerAndAttachToSCS(scs);
      simulateForTime(scs, simulateTime);
      int ticksPerCycle = (int) scs.getTicksPerPlayCycle();

      scs.setIndex(0);
      simulationRewoundListenerHasBeenNotified.set(false);
      scs.tickButDoNotNotifySimulationRewoundListeners(ticksIncrease);
      double currentSCSIndex = scs.getIndex();
      assertFalse(simulationRewoundListenerHasBeenNotified.getBooleanValue());
      assertEquals(ticksIncrease, currentSCSIndex, epsilon);

      scs.setIndex(0);
      simulationRewoundListenerHasBeenNotified.set(false);
      scs.tick();
      double currentSCSIndex2 = scs.getIndex();
      assertTrue(simulationRewoundListenerHasBeenNotified.getBooleanValue());
      assertEquals(ticksPerCycle, currentSCSIndex2, epsilon);

      scs.setIndex(ticksPerCycle);
      simulationRewoundListenerHasBeenNotified.set(false);
      scs.unTick();
      int currentSCSIndex3 = scs.getIndex();
      assertTrue(simulationRewoundListenerHasBeenNotified.getBooleanValue());
      assertEquals(0.0, currentSCSIndex3, epsilon);

      scs.setIndex(0);
      simulationRewoundListenerHasBeenNotified.set(false);
      scs.tick(ticksIncrease);
      double currentSCSIndex4 = scs.getIndex();
      assertTrue(simulationRewoundListenerHasBeenNotified.getBooleanValue());
      assertEquals(ticksIncrease, currentSCSIndex4, epsilon);

      scs.setIndex(0);
      simulationRewoundListenerHasBeenNotified.set(false);
      scs.setTick(ticksIncrease);
      double currentSCSIndex5 = scs.getIndex();
      assertTrue(simulationRewoundListenerHasBeenNotified.getBooleanValue());
      assertEquals(ticksIncrease * ticksPerCycle, currentSCSIndex5, epsilon);

      scs.setIndex(0);
      simulationRewoundListenerHasBeenNotified.set(false);
      scs.tickAndUpdate();
      double currentSCSIndex6 = scs.getIndex();
      assertFalse(simulationRewoundListenerHasBeenNotified.getBooleanValue());
      assertEquals(1.0, currentSCSIndex6, epsilon);

      scs.setIndex(0);
      simulationRewoundListenerHasBeenNotified.set(false);
      scs.updateAndTick();
      double currentSCSIndex7 = scs.getIndex();
      assertTrue(simulationRewoundListenerHasBeenNotified.getBooleanValue());
      assertEquals(1.0, currentSCSIndex7, epsilon);

      scs.setIndex(0);
      simulationRewoundListenerHasBeenNotified.set(false);
      scs.tickAndUpdateLeisurely(ticksIncrease);
      double currentSCSIndex8 = scs.getIndex();
      assertFalse(simulationRewoundListenerHasBeenNotified.getBooleanValue());
      assertEquals(1.0, currentSCSIndex8, epsilon);

      scs.setIndex(0);
      simulationRewoundListenerHasBeenNotified.set(false);
      scs.setIndexButDoNotNotifySimulationRewoundListeners(ticksIncrease);
      double currentSCSIndex9 = scs.getIndex();
      assertFalse(simulationRewoundListenerHasBeenNotified.getBooleanValue());
      assertEquals(ticksIncrease, currentSCSIndex9, epsilon);

      scs.stopSimulationThread();
      boolean isThreadRunningFromSCS = scs.isSimulationThreadRunning();
      assertFalse(isThreadRunningFromSCS);

      setInputAndOutputPointsWithoutCroppingInSCS(scs, inputPoint, outputPoint);
      boolean isMiddleIndexFromSCS = scs.isIndexBetweenInAndOutPoint(middleIndex);
      assertEquals(true, isMiddleIndexFromSCS);
      isMiddleIndexFromSCS = scs.isIndexBetweenInAndOutPoint(nonMiddleIndex);
      assertEquals(false, isMiddleIndexFromSCS);

      setInputPointInSCS(scs, inputPoint);
      scs.setIndex(outputPoint);
      scs.gotoInPointNow();
      int inputPointFromSCS = scs.getIndex();
      assertEquals(inputPoint, inputPointFromSCS);

      setOutputPointInSCS(scs, outputPoint);
      scs.setIndex(inputPoint);
      scs.gotoOutPointNow();
      int outputPointFromSCS = scs.getIndex();
      assertEquals(outputPoint, outputPointFromSCS);

      setInputAndOutputPointsWithoutCroppingInSCS(scs, inputPoint, outputPoint);
      scs.setIndex(keyPoint);
      scs.addKeyPoint();
      Integer keyPointFromSCS = scs.getKeyPoints().get(0);
      assertEquals(keyPoint, keyPointFromSCS.intValue(), epsilon);

      setInputAndOutputPointsWithoutCroppingInSCS(scs, inputPoint, outputPoint);
      scs.setIndex(keyPoint);
      scs.stepBackwardNow();
      int currentIndexFromSCS = scs.getIndex();
      assertEquals(keyPoint - 1, currentIndexFromSCS, epsilon);

      setInputAndOutputPointsWithoutCroppingInSCS(scs, inputPoint, outputPoint);
      scs.setIndex(keyPoint);
      scs.stepForwardNow(indexStep);
      int currentIndexFromSCS2 = scs.getIndex();
      assertEquals(keyPoint + indexStep, currentIndexFromSCS2, epsilon);

      setInputAndOutputPointsWithoutCroppingInSCS(scs, inputPoint, outputPoint);
      scs.setIndex(keyPoint);
      scs.stepForward(indexStep);
      scs.run();
      int currentIndexFromSCS3 = scs.getIndex();
      assertEquals(keyPoint + indexStep, currentIndexFromSCS3, epsilon);

      setInputAndOutputPointsWithoutCroppingInSCS(scs, inputPoint, outputPoint);
      scs.setIndex(keyPoint);
      scs.stepForward();
      scs.run();
      int currentIndexFromSCS4 = scs.getIndex();
      assertEquals(keyPoint + 1, currentIndexFromSCS4, epsilon);

      setInputAndOutputPointsWithoutCroppingInSCS(scs, inputPoint, outputPoint);
      scs.setIndex(keyPoint);
      scs.stepBackward(indexStep);
      scs.run();
      int currentIndexFromSCS5 = scs.getIndex();
      assertEquals(keyPoint - indexStep, currentIndexFromSCS5, epsilon);

      setInputAndOutputPointsWithoutCroppingInSCS(scs, inputPoint, outputPoint);
      scs.setIndex(keyPoint);
      scs.stepBackward();
      scs.run();
      int currentIndexFromSCS6 = scs.getIndex();
      assertEquals(keyPoint - 1, currentIndexFromSCS6, epsilon);
   }

	@ContinuousIntegrationTest(estimatedDuration = 8.7)
	@Test(timeout = 43000)
   public void testVariablesMethods()
   {
      addDoubleYoVariablesInSCSRegistry(cameraDollyXYZVarNames, cameraDollyXYZVarValues, scs);
      YoVariableList yoVariableList = createVarListOfDoubleYoVariableWithDummyRegistry(variableGroup1, variableGroup1Values);
      YoVariableList[] yoVariableLists = createTwoVarListOfDoubleYoVariablesWithDummyRegistry(variableGroup2, variableGroup2Values, variableGroup3,
            variableGroup3Values);
      ArrayList<YoVariableList> yoVariableArrayLists = createArrayListOfDoubleYoVariableWithDummyRegistry(variableGroup4, variableGroup4Values, variableGroup5,
            variableGroup5Values);

      scs.setupEntryBox(simpleRobotFirstVariableName);
      ThreadTools.sleep(THREAD_SLEEP_TIME);
      boolean entryBoxIsInSCS = scsContainsTheEntryBox(scs, simpleRobotFirstVariableName);
      assertTrue(entryBoxIsInSCS);

      scs.setupEntryBox(simpleRobotLastVariableName);
      ThreadTools.sleep(THREAD_SLEEP_TIME);
      boolean entryBoxIsInSCS2 = scsContainsTheEntryBox(scs, simpleRobotLastVariableName);
      assertTrue(entryBoxIsInSCS2);

      scs.setupEntryBox(cameraDollyXYZVarNames);
      ThreadTools.sleep(THREAD_SLEEP_TIME);
      boolean entryBoxesAreInSCS = scsContainsTheEntryBoxes(scs, cameraDollyXYZVarNames);
      assertTrue(entryBoxesAreInSCS);

      scs.setupGraph(simpleRobotFirstVariableName);
      ThreadTools.sleep(THREAD_SLEEP_TIME);
      boolean graphIsInSCS = scsContainsTheGraph(scs, simpleRobotFirstVariableName);
      assertTrue(graphIsInSCS);

      scs.setupGraph(simpleRobotLastVariableName);
      ThreadTools.sleep(THREAD_SLEEP_TIME);
      boolean graphIsInSCS2 = scsContainsTheGraph(scs, simpleRobotLastVariableName);
      assertTrue(graphIsInSCS2);

      scs.setupGraph(cameraDollyXYZVarNames);
      ThreadTools.sleep(THREAD_SLEEP_TIME);
      boolean graphsAreInSCS = scsContainsTheGraphs(scs, cameraDollyXYZVarNames);
      assertTrue(graphsAreInSCS);

      scs.addVarList(yoVariableList);
      YoVariableList yoVariableListFromSCS = scs.getCombinedVarList();
      assertYoVariableListContainsVariables(yoVariableListFromSCS, yoVariableList.getAllVariables());

      scs.addVarLists(yoVariableLists);
      YoVariableList yoVariableListFromSCS2 = scs.getCombinedVarList();
      assertYoVariableListContainsVariables(yoVariableListFromSCS2, yoVariableLists[0].getAllVariables());
      assertYoVariableListContainsVariables(yoVariableListFromSCS2, yoVariableLists[1].getAllVariables());

      scs.addVarLists(yoVariableArrayLists);
      YoVariableList yoVariableListFromSCS3 = scs.getCombinedVarList();
      assertYoVariableListContainsArrayListOfVariables(yoVariableListFromSCS3, yoVariableArrayLists);
   }

	@ContinuousIntegrationTest(estimatedDuration = 2.5)
	@Test(timeout = 30000)
   public void testGroupMethods()
   {
      addDoubleYoVariablesInSCSRegistry(cameraDollyXYZVarNames, cameraDollyXYZVarValues, scs);
      addDoubleYoVariablesInSCSRegistry(cameraTrackingXYZVarNames, cameraTrackingXYZVarValues, scs);

      scs.setupVarGroup(varGroupName, cameraDollyXYZVarNames);
      String[] varGroupNamesFromSCS = getVarGroupNames(scs);
      assertArrayOfStringsContainsTheString(varGroupNamesFromSCS, varGroupName);

      scs.setupVarGroup(varGroupName2, cameraDollyXYZVarNames, regularExpressions);
      String[] varGroupNamesFromSCS2 = getVarGroupNames(scs);
      assertArrayOfStringsContainsTheString(varGroupNamesFromSCS2, varGroupName2);

      scs.setupGraphGroup(graphGroupName, graphGroupVars);
      String[] graphGroupNamesFromSCS = getGraphGroupNames(scs);
      assertArrayOfStringsContainsTheString(graphGroupNamesFromSCS, graphGroupName);

      scs.setupGraphGroup(graphGroupName2, graphGroupVarsWithConfig);
      String[] graphGroupNamesFromSCS2 = getGraphGroupNames(scs);
      assertArrayOfStringsContainsTheString(graphGroupNamesFromSCS2, graphGroupName2);

      scs.setupGraphGroup(graphGroupName3, graphGroupVars, graphGroupNumberOfColumns);
      String[] graphGroupNamesFromSCS3 = getGraphGroupNames(scs);
      assertArrayOfStringsContainsTheString(graphGroupNamesFromSCS3, graphGroupName3);

      scs.setupGraphGroup(graphGroupName4, graphGroupVarsWithConfig);
      String[] graphGroupNamesFromSCS4 = getGraphGroupNames(scs);
      assertArrayOfStringsContainsTheString(graphGroupNamesFromSCS4, graphGroupName4);

      scs.setupEntryBoxGroup(entryBoxGroupName, cameraDollyXYZVarNames);
      String[] entryBoxGroupFromSCS = getEntryBoxGroupListNames(scs);
      assertArrayOfStringsContainsTheString(entryBoxGroupFromSCS, entryBoxGroupName);

      scs.setupEntryBoxGroup(entryBoxGroupName2, cameraDollyXYZVarNames, regularExpressions);
      String[] entryBoxGroupFromSCS2 = getEntryBoxGroupListNames(scs);
      assertArrayOfStringsContainsTheString(entryBoxGroupFromSCS2, entryBoxGroupName2);

      scs.setupConfiguration(configurationName, varGroupName3, graphGroupName5, entryBoxGroupName3);
      String[] configurationNameFromSCS = getConfigurationListNames(scs);
      assertArrayOfStringsContainsTheString(configurationNameFromSCS, configurationName);

      scs.setupExtraPanel(extraPanelConfiguration);
      Component extraPanelConfigurationPanelFromSCS = getExtraPanelConfigurationPanel(scs, extraPanelConfigurationName);
      assertNotNull(extraPanelConfigurationPanelFromSCS);
   }

	@ContinuousIntegrationTest(estimatedDuration = 8.5)
	@Test(timeout = 43000)
   public void testSimulationListeners()
   {
      SimulationDoneListener simulationDoneListener = createSimulationDoneListener();
      SimulationDoneCriterion simulationDoneCriterion = createSimulationDoneCriterion();
      PlaybackListener playbackListener = createPlaybackListener();
      PlayCycleListener playCycleListener = createPlayCycleListener();
      DataProcessingFunction dataProcessingFunction = createDataProcessingFunction();
      ToggleKeyPointModeCommandListener toggleKeyPointModeCommandListener = createToggleKeyPointModeCommandListener();
      scs.attachPlayCycleListener(playCycleListener);
      scs.attachPlaybackListener(playbackListener);
      scs.addSimulateDoneListener(simulationDoneListener);
      scs.setSimulateDoneCriterion(simulationDoneCriterion);
      scs.registerToggleKeyPointModeCommandListener(toggleKeyPointModeCommandListener);

      ArrayList<PlayCycleListener> playCycleListenersFromSCS = scs.getPlayCycleListeners();
      assertArrayOfObjectsContainsTheObject(playCycleListenersFromSCS, playCycleListener);

      simulationDoneListenerHasBeenNotified.set(false);
      callSCSMethodSimulateOneTimeStep(scs);
      ThreadTools.sleep(THREAD_SLEEP_TIME);
      assertTrue(simulationDoneListenerHasBeenNotified.getBooleanValue());

      simulationDoneListenerHasBeenNotified.set(false);
      scs.removeSimulateDoneListener(simulationDoneListener);
      ThreadTools.sleep(THREAD_SLEEP_TIME);
      assertFalse(simulationDoneListenerHasBeenNotified.getBooleanValue());

      simulationDoneListenerHasBeenNotified.set(false);
      scs.simulate(Double.MAX_VALUE);
      boolean isSCSSimulatingBeforeCriterion = scs.isSimulating();
      setSimulationDoneCriterion.set(true);
      ThreadTools.sleep(THREAD_SLEEP_TIME);
      boolean isSCSSimulatingAfterCriterion = scs.isSimulating();
      assertTrue(isSCSSimulatingBeforeCriterion);
      assertFalse(isSCSSimulatingAfterCriterion);

      realTimeRateInSCS.set(Double.POSITIVE_INFINITY);
      scs.play();
      ThreadTools.sleep(THREAD_SLEEP_TIME);
      double realTimeRateFromSCS = scs.getPlaybackRealTimeRate();
      assertEquals(realTimeRateFromSCS, realTimeRateInSCS.getDoubleValue(), epsilon);

      processDataHasBeenCalled.set(false);
      scs.applyDataProcessingFunction(dataProcessingFunction);
      ThreadTools.sleep(THREAD_SLEEP_TIME);
      assertTrue(processDataHasBeenCalled.getBooleanValue());

      toggleKeyPointModeCommandListenerHasBeenCalled.set(false);
      scs.toggleKeyPointMode();
      ThreadTools.sleep(THREAD_SLEEP_TIME);
      assertTrue(toggleKeyPointModeCommandListenerHasBeenCalled.getBooleanValue());
   }

	@ContinuousIntegrationTest(estimatedDuration = 9.5)
	@Test(timeout = 48000)
   public void testDataExporting()
   {
      simulateForTime(scs, simulateTime);
      int initialInPoint = scs.getInPoint();
      int initialOutPoint = scs.getOutPoint();
      int initialInOutBufferLength = getInOutBufferLengthFromSCS(scs);
      int initialBufferSize = getBufferSizeFromSCS(scs);
      CaptureDevice captureDevice = getCaptureDeviceFromSCS(scs);
      addDoubleYoVariablesInSCSRegistry(cameraTrackingXYZVarNames, cameraTrackingXYZVarValues, scs);

      addAndSubtractOneFromInAndOutPointIndexWithoutCrop(scs);
      scs.cropBuffer();
      int currentInPoint = scs.getInPoint();
      int currentBufferSize = scs.getDataBuffer().getBufferSize();
      assertEquals(0, currentInPoint, epsilon);
      assertEquals(initialInOutBufferLength - 2, currentBufferSize, epsilon);

      addAndSubtractOneFromInAndOutPointIndexWithoutCrop(scs);
      scs.packBuffer();
      int currentInPoint2 = scs.getInPoint();
      assertEquals(0, currentInPoint2, epsilon);

      scs.setWrapBuffer(false);
      boolean wrapBufferFromSCS = getWrapBufferFromSCS(scs);
      assertFalse(wrapBufferFromSCS);

      scs.setWrapBuffer(true);
      boolean wrapBufferFromSCS2 = getWrapBufferFromSCS(scs);
      assertTrue(wrapBufferFromSCS2);

      scs.changeBufferSize(initialBufferSize * 2);
      int bufferSizeFromSCS = getBufferSizeFromSCS(scs);
      assertEquals(initialBufferSize * 2, bufferSizeFromSCS, epsilon);

      scs.setMaxBufferSize(initialBufferSize * 3);
      int maxBufferSizeFromSCS = getMaxBufferSizeFromSCS(scs);
      assertEquals(initialBufferSize * 3, maxBufferSizeFromSCS, epsilon);

      BufferedImage bufferedImage = scs.exportSnapshotAsBufferedImage(captureDevice);
      assertNotNull(bufferedImage);

      BufferedImage bufferedImage2 = scs.exportSnapshotAsBufferedImage();
      assertNotNull(bufferedImage2);

      File file = new File(TEST_DIRECTORY + "file.csv");
      scs.writeSpreadsheetFormattedData("all", file);
      assertTheFileContainsTheVariables(file, cameraTrackingXYZVarNames);
      file.delete();

      File file2 = new File(TEST_DIRECTORY + "file2.gz");
      scs.writeData(file2);
      SimulationConstructionSet scs2 = createNewSCSWithEmptyRobot("simpleRobot2");
      scs2.readData(file2);
      assertSCSContainsTheVariables(scs2, cameraTrackingXYZVarNames);
      closeGivenSCSAndDeleteFile(scs2, file2);

      File file3 = new File(TEST_DIRECTORY + "file3.csv");
      scs.writeSpreadsheetFormattedData("all", file3);
      SimulationConstructionSet scs3 = createNewSCSWithEmptyRobot("simpleRobot3");
      scs3.readData(file3);
      assertSCSContainsTheVariables(scs3, cameraTrackingXYZVarNames);
      closeGivenSCSAndDeleteFile(scs3, file3);

      String defaultTimeVariable = scs.getTimeVariableName();
      scs.setTimeVariableName(cameraTrackingXYZVarNames[0]);
      String timeVariableNameFromSCS = scs.getTimeVariableName();
      assertEquals(cameraTrackingXYZVarNames[0], timeVariableNameFromSCS);
      scs.setTimeVariableName(defaultTimeVariable);

      File file4 = new File(TEST_DIRECTORY + "file4.state");
      scs.writeState(file4);
      double initialTime = scs.getTime();
      simulateForTime(scs, simulateTime);
      scs.readState(file4);
      double timeAfterReadState = scs.getTime();
      assertEquals(initialTime, timeAfterReadState, epsilon);
      file4.delete();

      File file5 = new File(TEST_DIRECTORY + "file5.state.gz");
      scs.writeState("all", false, true, file5);
      double initialTime2 = scs.getTime();
      simulateForTime(scs, simulateTime);
      scs.readState(file5);
      double timeAfterReadState2 = scs.getTime();
      assertEquals(initialTime2, timeAfterReadState2, epsilon);
      file5.delete();

      File file6 = new File(TEST_DIRECTORY + "file6.csv");
      scs.writeSpreadsheetFormattedState("all", file6);
      assertTheFileContainsTheVariables(file6, cameraTrackingXYZVarNames);

      scs.writeState(TEST_DIRECTORY + "test.state");
      double initialTime3 = scs.getTime();
      simulateForTime(scs, simulateTime);
      scs.readState(TEST_DIRECTORY + "test.state");
      double timeAfterReadState3 = scs.getTime();
      assertEquals(initialTime3, timeAfterReadState3, epsilon);

      File file7 = new File(TEST_DIRECTORY + "file7.state.gz");
      scs.writeState("all", false, true, file7);
      double initialTime4 = scs.getTime();
      simulateForTime(scs, simulateTime);
      scs.readState(file7, false);
      double timeAfterReadState4 = scs.getTime();
      assertEquals(initialTime4, timeAfterReadState4, epsilon);
      file7.delete();
   }

   // local methods

   private void assertIfGUIComponentsAreDisableOrEnabled(SimulationConstructionSet scs, boolean assertAreEnabled)
   {
      StandardGUIActions standardGUIActions = scs.getStandardGUIActions();
      ArrayList<AbstractAction> guiActions = standardGUIActions.getGuiActions();
      for (int i = 0; i<guiActions.size(); i++)
      {
         AbstractAction guiAction = guiActions.get(i);
         boolean guiActionStatus = guiAction.isEnabled();

         if(assertAreEnabled)
            assertTrue(guiActionStatus);
         else
            assertFalse(guiActionStatus);
      }
   }

   private ScsPhysics createScsPhysics()
   {
      ScsCollisionConfigure collisionConfigure = createScsCollisionConfigure();
      ScsCollisionDetector collisionDetector = createScsCollisionDetector();
      CollisionArbiter collisionArbiter = new DoNothingCollisionArbiter();
      CollisionHandler collisionHandler = new DefaultCollisionHandler(0.3, 0.3);
      DefaultCollisionVisualizer visualize = new DefaultCollisionVisualizer(0.1, 0.1, 0.01, scs, 100);

      ScsPhysics physics = new ScsPhysics(collisionConfigure, collisionDetector, collisionArbiter, collisionHandler, visualize);

      return physics;
   }

   private ScsCollisionDetector createScsCollisionDetector()
   {
      ScsCollisionDetector scsCollisionDetector = new ScsCollisionDetector()
      {
         @Override
         public void initialize()
         {

         }

         @Override
         public CollisionShapeFactory getShapeFactory()
         {
            return null;
         }

         @Override
         public void performCollisionDetection(CollisionDetectionResult result)
         {
         }
      };

      return scsCollisionDetector;
   }

   private ScsCollisionConfigure createScsCollisionConfigure()
   {
      ScsCollisionConfigure scsCollisionConfigure = new ScsCollisionConfigure()
      {
         @Override
         public void setup(Robot robot, ScsCollisionDetector collisionDetector, CollisionHandler collisionHandler)
         {
         }
      };

      return scsCollisionConfigure;
   }

   private YoGraphicsListRegistry createYoGraphicsListRegistryWithObject()
   {
      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();
      YoGraphicsList yoGraphicsList = new YoGraphicsList(yoGraphicsListName);
      yoGraphicsList.add(yoGraphic);
      yoGraphicsListRegistry.registerYoGraphicsList(yoGraphicsList);

      return yoGraphicsListRegistry;
   }

   private ToggleKeyPointModeCommandListener createToggleKeyPointModeCommandListener()
   {
      ToggleKeyPointModeCommandListener toggleKeyPointModeCommandListener = new ToggleKeyPointModeCommandListener()
      {
         @Override
         public void updateKeyPointModeStatus()
         {
            toggleKeyPointModeCommandListenerHasBeenCalled.set(true);
         }

         @Override
         public void closeAndDispose()
         {
         }
      };

      return toggleKeyPointModeCommandListener;
   }

   private void assertBooleansAreOpposite(boolean one, boolean two)
   {
      if (one)
         assertFalse(two);
      else
         assertTrue(two);
   }

   private DataProcessingFunction createDataProcessingFunction()
   {
      DataProcessingFunction dataProcessingFunction = new DataProcessingFunction()
      {
         @Override
         public void processData()
         {
            processDataHasBeenCalled.set(true);
         }

         @Override
         public void initializeProcessing()
         {
            // TODO Auto-generated method stub

         }
      };

      return dataProcessingFunction;
   }

   private PlayCycleListener createPlayCycleListener()
   {
      PlayCycleListener playCycleListener = new PlayCycleListener()
      {
         @Override
         public void update(int tick)
         {

         }
      };

      return playCycleListener;
   }

//   private ArrayList<CollisionGroup> createArrayListOfCollisionGroup(int numberOfElements)
//   {
//      ArrayList<CollisionGroup> arrayListOfCollisionGroup = new ArrayList<CollisionGroup>();
//      for (int i = 0; i < numberOfElements; i++)
//      {
//         arrayListOfCollisionGroup.add(new CollisionGroup());
//      }
//
//      return arrayListOfCollisionGroup;
//   }

   private <T> void assertArrayOfObjectsContainsTheArrayOfObject(ArrayList<T> mainArrayList, ArrayList<T> arrayList)
   {
      int numberOfElements = arrayList.size();

      for (int i = 0; i < numberOfElements; i++)
      {
         assertArrayOfObjectsContainsTheObject(mainArrayList, arrayList.get(i));
      }
   }

   private <T> void assertArrayOfObjectsContainsTheObject(ArrayList<T> arrayList, T object)
   {
      int numberOfElements = arrayList.size();
      boolean ret = false;

      for (int i = 0; i < numberOfElements; i++)
      {
         ret = ret || arrayList.get(i).equals(object);
      }

      assertTrue(ret);
   }

   private void assertYoVariableListContainsArrayListOfVariables(YoVariableList yoVariableList, ArrayList<YoVariableList> arrayLists)
   {
      int numberOfList = arrayLists.size();

      for (int j = 0; j < numberOfList; j++)
      {
         YoVariable<?>[] variables = arrayLists.get(j).getAllVariables();

         int numberOfVariables = variables.length;

         for (int i = 0; i < numberOfVariables; i++)
         {
            boolean containsTheVar = yoVariableList.containsVariable(variables[i]);
            assertTrue(containsTheVar);
         }
      }
   }

   private void assertYoVariableListContainsVariables(YoVariableList yoVariableList, YoVariable<?>[] variables)
   {
      int numberOfVariables = variables.length;

      for (int i = 0; i < numberOfVariables; i++)
      {
         boolean containsTheVar = yoVariableList.containsVariable(variables[i]);
         assertTrue(containsTheVar);
      }
   }

   private ArrayList<YoVariableList> createArrayListOfDoubleYoVariableWithDummyRegistry(String[] variableNames1, double[] varValues1, String[] variableNames2,
         double[] varValues2)
   {
      ArrayList<YoVariableList> arrayLists = new ArrayList<YoVariableList>();
      YoVariableList[] yoVariableList = createTwoVarListOfDoubleYoVariablesWithDummyRegistry(variableNames1, varValues1, variableNames2, varValues2);

      for (int i = 0; i < yoVariableList.length; i++)
      {
         arrayLists.add(yoVariableList[i]);
      }

      return arrayLists;
   }

   private YoVariableList createVarListOfDoubleYoVariableWithDummyRegistry(String[] variableNames, double[] varValues)
   {
      DoubleYoVariable[] doubleYoVariables = null;

      if (variableNames.length == varValues.length)
      {
         YoVariableRegistry registry = new YoVariableRegistry("dummy");
         doubleYoVariables = createAndSetDoubleYoVariableToRegistry(variableNames, varValues, registry);
      }
      else
      {
         System.out.print("Input arrays have different length.");
      }

      return createYoVariableList("yoVariableList", doubleYoVariables);
   }

   private YoVariableList[] createTwoVarListOfDoubleYoVariablesWithDummyRegistry(String[] variableNames1, double[] varValues1, String[] variableNames2,
         double[] varValues2)
   {
      YoVariableRegistry registry = new YoVariableRegistry("dummy");
      DoubleYoVariable[] doubleYoVariables1 = createAndSetDoubleYoVariableToRegistry(variableNames1, varValues1, registry);
      DoubleYoVariable[] doubleYoVariables2 = createAndSetDoubleYoVariableToRegistry(variableNames2, varValues2, registry);

      YoVariableList[] yoVariableLists = new YoVariableList[2];
      yoVariableLists[0] = createYoVariableList("yoVariableList1", doubleYoVariables1);
      yoVariableLists[1] = createYoVariableList("yoVariableList2", doubleYoVariables2);

      return yoVariableLists;
   }

   private DoubleYoVariable[] createAndSetDoubleYoVariableToRegistry(String[] varNames, double[] varValues, YoVariableRegistry registry)
   {
      DoubleYoVariable[] doubleYoVariables = new DoubleYoVariable[varNames.length];

      for (int i = 0; i < varNames.length; i++)
      {
         DoubleYoVariable doubleYoVariable = new DoubleYoVariable(varNames[i], registry);
         doubleYoVariable.set(varValues[i]);
         doubleYoVariables[i] = doubleYoVariable;
      }

      return doubleYoVariables;
   }

   private YoVariableList createYoVariableList(String name, YoVariable<?>[] yoVariables)
   {
      YoVariableList yoVariableList = new YoVariableList(name);
      yoVariableList.addVariables(yoVariables);

      return yoVariableList;
   }

   private NewDataListener createNewDataListener()
   {
      NewDataListener newDataListener = new NewDataListener()
      {
         @Override
         public void newDataHasBeenSent()
         {

         }

         @Override
         public void newDataHasBeenReceived()
         {

         }
      };

      return newDataListener;
   }

   private int getExtendedStateFromSCS(SimulationConstructionSet scs)
   {
      return scs.getGUI().getFrame().getExtendedState();
   }

   private SimulationConstructionSet createNewSCSWithEmptyRobot(String robotName)
   {
      return new SimulationConstructionSet(new Robot(robotName));
   }

   private void closeGivenSCSAndDeleteFile(SimulationConstructionSet scs, File file)
   {
      file.delete();
      closeGivenSCS(scs);
   }

   private void closeGivenSCS(SimulationConstructionSet scs)
   {
      ThreadTools.sleep(CLOSING_SLEEP_TIME);
      scs.closeAndDispose();
      scs = null;
   }

   private void assertSCSContainsTheVariables(SimulationConstructionSet scs, String[] variablesNames)
   {
      YoVariableRegistry registry = scs.getRootRegistry();
      String[] variableNamesFromSCS = getAllVariableNamesFromRegistry(registry);
      assertArrayOfStringsContainsTheStrings(variableNamesFromSCS, variablesNames);
   }

   private String[] getAllVariableNamesFromRegistry(YoVariableRegistry registry)
   {
      int numberOfVariables = registry.getNumberOfYoVariables();
      String[] names = new String[numberOfVariables];
      for (int i = 0; i < numberOfVariables; i++)
      {
         names[i] = registry.getYoVariable(i).getName();
      }

      return names;
   }

   private void assertTheFileContainsTheVariables(File file, String[] variablesNames)
   {
      String header = null;
      try
      {
         BufferedReader bufferedReader = new BufferedReader(new InputStreamReader(new FileInputStream(file)));
         header = bufferedReader.readLine();
         bufferedReader.close();
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }

      String[] numberOfEntries = header.split(",");

      assertArrayOfStringsContainsTheStrings(numberOfEntries, variablesNames);
   }

   private CaptureDevice getCaptureDeviceFromSCS(SimulationConstructionSet scs)
   {
      return scs.getGUI().getActiveCaptureDevice();
   }

   private int getMaxBufferSizeFromSCS(SimulationConstructionSet scs)
   {
      return scs.getDataBuffer().getMaxBufferSize();
   }

   private int getInOutBufferLengthFromSCS(SimulationConstructionSet scs)
   {
      return scs.getDataBuffer().getBufferInOutLength();
   }

   private int getBufferSizeFromSCS(SimulationConstructionSet scs)
   {
      return scs.getDataBuffer().getBufferSize();
   }

   private boolean getWrapBufferFromSCS(SimulationConstructionSet scs)
   {
      return scs.getDataBuffer().getWrapBuffer();
   }

   private void addAndSubtractOneFromInAndOutPointIndexWithoutCrop(SimulationConstructionSet scs)
   {
      addOneToInPointIndexWithoutCrop(scs);
      subtractOneToOutPointIndexWithoutCrop(scs);
   }

   private void addOneToInPointIndexWithoutCrop(SimulationConstructionSet scs)
   {
      int currentInPoint = scs.getInPoint();
      scs.setIndex(currentInPoint + 1);
      scs.setInPoint();
   }

   private void subtractOneToOutPointIndexWithoutCrop(SimulationConstructionSet scs)
   {
      int currentOutPoint = scs.getOutPoint();
      scs.setIndex(currentOutPoint - 1);
      scs.setOutPoint();
   }

   private PlaybackListener createPlaybackListener()
   {
      PlaybackListener listener = new PlaybackListener()
      {
         @Override
         public void indexChanged(int newIndex, double newTime)
         {

         }

         @Override
         public void play(double realTimeRate)
         {
            realTimeRateInSCS.set(realTimeRate);
         }

         @Override
         public void stop()
         {

         }
      };

      return listener;
   }

   private SimulationDoneCriterion createSimulationDoneCriterion()
   {
      SimulationDoneCriterion criterion = new SimulationDoneCriterion()
      {
         @Override
         public boolean isSimulationDone()
         {
            return setSimulationDoneCriterion.getBooleanValue();
         }
      };

      return criterion;
   }

   private SimulationDoneListener createSimulationDoneListener()
   {
      SimulationDoneListener listener = new SimulationDoneListener()
      {
         @Override
         public void simulationDone()
         {
            simulationDoneListenerHasBeenNotified.set(true);
         }

         @Override
         public void simulationDoneWithException(Throwable throwable)
         {

         }
      };

      return listener;
   }

   private void callSCSMethodSimulateOneRecordStepNow(SimulationConstructionSet scs)
   {
      try
      {
         scs.simulateOneRecordStepNow();
      }
      catch (UnreasonableAccelerationException e)
      {
         e.printStackTrace();
      }
   }

   private double getExpectedFinalTime(SimulationConstructionSet scs)
   {
      double initialTime = scs.getRobots()[0].getTime();
      double recordFreq = scs.getRecordFreq();
      double DT = scs.getDT();
      return initialTime + recordFreq * DT;
   }

   private void callSCSMethodSimulateOneRecordStep(SimulationConstructionSet scs)
   {
      try
      {
         scs.simulateOneRecordStep();
      }
      catch (UnreasonableAccelerationException e)
      {
         e.printStackTrace();
      }
   }

   private void callSCSMethodSimulateOneTimeStep(SimulationConstructionSet scs)
   {
      try
      {
         scs.simulateOneTimeStep();
      }
      catch (UnreasonableAccelerationException e)
      {
         e.printStackTrace();
      }
   }

   private boolean stateIfTerrainIsVisible(SimulationConstructionSet scs)
   {
      JMEGraphics3DAdapter graphics3DAdapter = getGraphics3DAdapter(scs);
      return graphics3DAdapter.getRenderer().isGroundVisible();
   }

   private JMEGraphics3DAdapter getGraphics3DAdapter(SimulationConstructionSet scs)
   {
      return (JMEGraphics3DAdapter) scs.getGUI().getGraphics3dAdapter();
   }

   private void assertArrayOfStringsContainsTheStrings(String[] array, String[] strings)
   {
      for (int i = 0; i < strings.length; i++)
      {
         assertArrayOfStringsContainsTheString(array, strings[i]);
      }
   }

   private GraphConfiguration[] createGraphConfigurations(String[] graphConfigurationNames)
   {
      GraphConfiguration[] graphConfigurations = new GraphConfiguration[graphConfigurationNames.length];
      for (int i = 0; i < graphConfigurationNames.length; i++)
      {
         graphConfigurations[i] = new GraphConfiguration(graphConfigurationNames[i]);
      }

      return graphConfigurations;
   }

   private String[] getGraphConfigurationListNames(SimulationConstructionSet scs)
   {
      return scs.getGUI().getGraphConfigurationList().getGraphConfigurationNames();
   }

   private boolean isCurrentView(SimulationConstructionSet scs, String currentViewName)
   {
      String representationOfCurrentView = getRepresentationOfCurrentView(scs);
      return representationOfCurrentView.contains(currentViewName);
   }

   private String getRepresentationOfCurrentView(SimulationConstructionSet scs)
   {
      StandardSimulationGUI gui = scs.getGUI();
      return gui.getXMLStyleRepresentationofMultiViews();
   }

   private String[] getViewportConfigurationNames(SimulationConstructionSet scs)
   {
      return scs.getGUI().getViewportConfigurationList().getViewportConfigurationNames();
   }

   private ViewportConfiguration createViewportConfiguration(String name)
   {
      return new ViewportConfiguration(name);
   }

   private Component getExtraPanelConfigurationPanel(SimulationConstructionSet scs, String panelName)
   {
      return scs.getGUI().getExtraPanel(panelName);
   }

   private ExtraPanelConfiguration createExtraPanelConfigurationWithPanel(String name)
   {
      Button panel = new Button();
      ExtraPanelConfiguration extraPanelConfiguration = new ExtraPanelConfiguration(name, panel, false);

      return extraPanelConfiguration;
   }

   private CameraConfiguration createCameraConfiguration(String name)
   {
      return new CameraConfiguration(name);
   }

   private String[] getCameraConfigurationNames(SimulationConstructionSet scs)
   {
      return scs.getGUI().getCameraConfigurationList().getCameraConfigurationNames();
   }

   private String[] getConfigurationListNames(SimulationConstructionSet scs)
   {
      return scs.getGUI().getConfigurationList().getConfigurationNames();
   }

   private String[] getEntryBoxGroupListNames(SimulationConstructionSet scs)
   {
      return scs.getGUI().getEntryBoxGroupList().getEntryBoxGroupNames();
   }

   private void assertArrayOfStringsContainsTheString(String[] array, String string)
   {
      boolean ret = false;

      for (int i = 0; i < array.length; i++)
      {
         ret = ret || array[i].contains(string);
      }

      assertTrue(ret);
   }

   private boolean scsContainsTheGraphs(SimulationConstructionSet scs, String[] varNames)
   {
      String representationOfGraphArrayPanel = scs.getGUI().getXMLStyleRepresentationOfGraphArrayPanel();
      boolean ret = true;

      for (int i = 0; i < varNames.length; i++)
      {
         ret = ret && representationOfGraphArrayPanel.contains(varNames[i]);
      }

      return ret;
   }

   private boolean scsContainsTheRobot(SimulationConstructionSet scs, String robotName)
   {
      Robot[] robots = scs.getRobots();
      boolean ret = false;

      for (int i = 0; i < robots.length; i++)
      {
         ret = ret || robots[i].getName().equals(robotName);
      }

      return ret;
   }

   private String[] getGraphGroupNames(SimulationConstructionSet scs)
   {
      GraphGroupList graphGroupList = getGraphGroupList(scs);
      return graphGroupList.getGraphGroupNames();
   }

   private String[] getVarGroupNames(SimulationConstructionSet scs)
   {
      return scs.getVarGroupList().getVarGroupNames();
   }

   private boolean scsContainsTheGraph(SimulationConstructionSet scs, String varName)
   {
      String representationOfGraphArrayPanel = scs.getGUI().getXMLStyleRepresentationOfGraphArrayPanel();
      return representationOfGraphArrayPanel.contains(varName);
   }

   private boolean scsContainsTheEntryBoxes(SimulationConstructionSet scs, String[] varNames)
   {
      String representationOfEntryBoxes = scs.getGUI().getXMLStyleRepresentationOfEntryBoxes();
      boolean ret = true;

      for (int i = 0; i < varNames.length; i++)
      {
         ret = ret && representationOfEntryBoxes.contains(varNames[i]);
      }

      return ret;
   }

   private boolean scsContainsTheEntryBox(SimulationConstructionSet scs, String varName)
   {
      String representationOfEntryBoxes = scs.getGUI().getXMLStyleRepresentationOfEntryBoxes();
      return representationOfEntryBoxes.contains(varName);
   }

   private void createSimulationRewoundListenerAndAttachToSCS(SimulationConstructionSet scs)
   {
      RewoundListener simulationRewoundListener = createSimulationRewoundListener();
      scs.attachSimulationRewoundListener(simulationRewoundListener);
   }

   private void simulateForTime(SimulationConstructionSet scs, double simulateTime)
   {
      scs.simulate(simulateTime);
      while (scs.isSimulating())
      {
         ThreadTools.sleep(100L);
      }
   }

   private RewoundListener createSimulationRewoundListener()
   {
      RewoundListener ret = new RewoundListener()
      {
         @Override
         public void wasRewound()
         {
            simulationRewoundListenerHasBeenNotified.set(true);
         }
      };

      return ret;
   }

   private ExitActionListener createExitActionListener()
   {
      ExitActionListener ret = new ExitActionListener()
      {
         @Override
         public void exitActionPerformed()
         {
            exitActionListenerHasBeenNotified.set(true);
         }
      };

      return ret;
   }

   private boolean getCameraKeyMode(SimulationConstructionSet scs)
   {
      ClassicCameraController classicCameraController = getClassicCameraController(scs);
      return classicCameraController.getCameraKeyMode();
   }

   private double getCameraFieldOfView(SimulationConstructionSet scs)
   {
      ClassicCameraController classicCameraController = getClassicCameraController(scs);
      return classicCameraController.getHorizontalFieldOfViewInRadians();
   }

   private double[] getCameraTrackingXYZVars(SimulationConstructionSet scs)
   {
      CameraTrackAndDollyYoVariablesHolder cameraTrackAndDollyYoVariablesHolder = getCameraTrackAndDollyVariablesHolder(scs);
      double x = cameraTrackAndDollyYoVariablesHolder.getTrackingX();
      double y = cameraTrackAndDollyYoVariablesHolder.getTrackingY();
      double z = cameraTrackAndDollyYoVariablesHolder.getTrackingZ();
      return new double[] { x, y, z };
   }

   private double[] getCameraDollyXYZVars(SimulationConstructionSet scs)
   {
      CameraTrackAndDollyYoVariablesHolder cameraTrackAndDollyYoVariablesHolder = getCameraTrackAndDollyVariablesHolder(scs);
      double x = cameraTrackAndDollyYoVariablesHolder.getDollyX();
      double y = cameraTrackAndDollyYoVariablesHolder.getDollyY();
      double z = cameraTrackAndDollyYoVariablesHolder.getDollyZ();
      return new double[] { x, y, z };
   }

   private double[] getCameraTrackingOffsetXYZValues(SimulationConstructionSet scs)
   {
      ClassicCameraController classicCameraController = getClassicCameraController(scs);
      double dx = classicCameraController.getTrackingXOffset();
      double dy = classicCameraController.getTrackingYOffset();
      double dz = classicCameraController.getTrackingZOffset();
      return new double[] { dx, dy, dz };
   }

   private double[] getCameraDollyOffsetXYZValues(SimulationConstructionSet scs)
   {
      ClassicCameraController classicCameraController = getClassicCameraController(scs);
      double dx = classicCameraController.getDollyXOffset();
      double dy = classicCameraController.getDollyYOffset();
      double dz = classicCameraController.getDollyZOffset();
      return new double[] { dx, dy, dz };
   }

   private double[] getCameraFixXYZValues(SimulationConstructionSet scs)
   {
      ClassicCameraController classicCameraController = getClassicCameraController(scs);
      double x = classicCameraController.getFixX();
      double y = classicCameraController.getFixY();
      double z = classicCameraController.getFixZ();
      return new double[] { x, y, z };
   }

   private double[] getCameraNearFarValues(SimulationConstructionSet scs)
   {
      ClassicCameraController classicCameraController = getClassicCameraController(scs);
      double near = classicCameraController.getClipNear();
      double far = classicCameraController.getClipFar();
      return new double[] { near, far };
   }

   private double[] getCameraPositionXYZValues(SimulationConstructionSet scs)
   {
      ClassicCameraController classicCameraController = getClassicCameraController(scs);
      double x = classicCameraController.getCamX();
      double y = classicCameraController.getCamY();
      double z = classicCameraController.getCamZ();
      return new double[] { x, y, z };
   }

   private GraphGroupList getGraphGroupList(SimulationConstructionSet scs)
   {
      return scs.getGUI().getGraphGroupList();
   }

   private ClassicCameraController getClassicCameraController(SimulationConstructionSet scs)
   {
      return (ClassicCameraController) scs.getGUI().getViewportPanel().getActiveView().getCameraController();
   }

   private CameraTrackAndDollyYoVariablesHolder getCameraTrackAndDollyVariablesHolder(SimulationConstructionSet scs)
   {
      ClassicCameraController classicCameraController = getClassicCameraController(scs);
      return (CameraTrackAndDollyYoVariablesHolder) classicCameraController.getCameraTrackAndDollyVariablesHolder();
   }

   private DoubleYoVariable[] addDoubleYoVariablesInSCSRegistry(String[] varNames, double[] varValues, SimulationConstructionSet scs)
   {
      DoubleYoVariable[] doubleYoVariables = null;

      if (varNames.length == varValues.length)
      {
         YoVariableRegistry scsRegistry = scs.getRootRegistry();
         doubleYoVariables = createAndSetDoubleYoVariableToRegistry(varNames, varValues, scsRegistry);
      }
      else
      {
         System.out.print("Input arrays have different length.");
      }

      return doubleYoVariables;
   }

   private String getRegistryNameSpaceFromRobot(Robot robotModel)
   {
      return robotModel.getRobotsYoVariableRegistry().getNameSpace().getName();
   }

   private String getFirstVariableNameFromRobotRegistry(Robot robotModel)
   {
      return robotModel.getRobotsYoVariableRegistry().getAllVariablesArray()[0].getName();
   }

   private String getLastVariableNameFromRobotRegistry(Robot robotModel)
   {
      int lastIndex = robotModel.getRobotsYoVariableRegistry().getAllVariablesArray().length - 1;
      return robotModel.getRobotsYoVariableRegistry().getAllVariablesArray()[lastIndex].getName();
   }

   private void setInputAndOutputPointsWithoutCroppingInSCS(SimulationConstructionSet scs, int inputPointIndex, int outputPointIndex)
   {
      setInputPointInSCS(scs, inputPointIndex);
      setOutputPointInSCS(scs, outputPointIndex);
   }

   private void setInputPointInSCS(SimulationConstructionSet scs, int inputPointIndex)
   {
      scs.setIndex(inputPointIndex);
      scs.setInPoint();
   }

   private void setOutputPointInSCS(SimulationConstructionSet scs, int outputPointIndex)
   {
      scs.setIndex(outputPointIndex);
      scs.setOutPoint();
   }

   private ArrayList<YoVariable<?>> getSimpleRobotVariablesThatContain(String searchString, boolean caseSensitive, Robot robotModel)
   {
      ArrayList<YoVariable<?>> currentlyMatched = robotModel.getAllVariables();
      ArrayList<YoVariable<?>> ret = null;

      if (currentlyMatched != null)
      {
         if (!caseSensitive)
         {
            searchString = searchString.toLowerCase();
         }

         for (int i = 0; i < currentlyMatched.size(); i++)
         {
            YoVariable<?> entry = currentlyMatched.get(i);

            if (entry.getName().toLowerCase().contains((searchString)))
            {
               if (ret == null)
               {
                  ret = new ArrayList<YoVariable<?>>();
               }

               ret.add(entry);
            }
         }
      }

      return ret;
   }

   private ArrayList<YoVariable<?>> getSimpleRobotVariablesThatStartWith(String searchString, Robot robotModel)
   {
      ArrayList<YoVariable<?>> currentlyMatched = robotModel.getAllVariables();
      ArrayList<YoVariable<?>> ret = null;

      for (int i = 0; i < currentlyMatched.size(); i++)
      {
         YoVariable<?> Variable = currentlyMatched.get(i);

         if (Variable.getName().startsWith(searchString))
         {
            if (ret == null)
            {
               ret = new ArrayList<YoVariable<?>>();
            }

            ret.add(Variable);
         }
      }

      return ret;
   }

   private String[] getVariableNamesGivenArrayListOfYoVariables(ArrayList<YoVariable<?>> yoVariableList)
   {
      String[] ret = null;

      for (int i = 0; i < yoVariableList.size(); i++)
      {
         String variableName = yoVariableList.get(i).getName();

         if (ret == null)
         {
            ret = new String[yoVariableList.size()];
         }

         ret[i] = variableName;
      }

      return ret;
   }

   private ArrayList<YoVariable<?>> getSimpleRobotRegExpVariables(String[] varNames, String[] regularExpressions, Robot robotModel)
   {
      ArrayList<YoVariable<?>> currentlyMatched = robotModel.getAllVariables();
      YoVariableList tempList = new YoVariableList("temp");

      for (int i = 0; i < currentlyMatched.size(); i++)
      {
         YoVariable var = currentlyMatched.get(i);

         tempList.addVariable(var);
      }

      return tempList.getMatchingVariables(varNames, regularExpressions);
   }

   private int computeRecordFreq(double recordDT, double simulateDT)
   {
      return (int) Math.round(recordDT / simulateDT);
   }

   private double recomputeTiming(double dt, double recordFreq, double realTimeRate, double frameRate)
   {
      int ticksPerCycle = computeTicksPerPlayCycle(dt, recordFreq, realTimeRate, frameRate);
      double secondsPerFrameRate = ticksPerCycle * (dt * recordFreq) / realTimeRate;
      return secondsPerFrameRate;
   }

   private int computeTicksPerPlayCycle(double dt, double recordFreq, double realTimeRate, double frameRate)
   {
      return Math.max((int) (frameRate * realTimeRate / (dt * recordFreq)), 1);
   }
}
