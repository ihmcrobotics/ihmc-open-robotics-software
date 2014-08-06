package com.yobotics.simulationconstructionset;

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

import org.fest.swing.edt.FailOnThreadViolationRepaintManager;
import org.junit.After;
import org.junit.Before;
import org.junit.BeforeClass;
import org.junit.Test;

import us.ihmc.graphics3DAdapter.camera.CameraConfiguration;
import us.ihmc.graphics3DAdapter.camera.CaptureDevice;
import us.ihmc.graphics3DAdapter.camera.ClassicCameraController;
import us.ihmc.graphics3DAdapter.graphics.Graphics3DObject;
import us.ihmc.graphics3DAdapter.jme.JMEGraphics3DAdapter;
import us.ihmc.graphics3DAdapter.structure.Graphics3DNode;
import us.ihmc.graphics3DAdapter.structure.Graphics3DNodeType;
import us.ihmc.utilities.ThreadTools;

import com.yobotics.simulationconstructionset.examples.FallingBrickRobot;
import com.yobotics.simulationconstructionset.graphics.GraphicsDynamicGraphicsObject;
import com.yobotics.simulationconstructionset.gui.EventDispatchThreadHelper;
import com.yobotics.simulationconstructionset.gui.ViewportWindow;
import com.yobotics.simulationconstructionset.gui.camera.CameraTrackAndDollyYoVariablesHolder;
import com.yobotics.simulationconstructionset.gui.config.GraphGroupList;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObject;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicVector;
import com.yobotics.simulationconstructionset.util.simulationRunner.StateFileComparer;
import com.yobotics.simulationconstructionset.util.simulationRunner.VariableDifference;

public class SimulationConstructionSetUsingDirectCallsTest
{
   // Note: some of the tests assume that:
   // - the simpleRobot has been used to create the SimulationConstructionSet instance
   // - the registry "simpleRegistry" is empty
   
   private static final long CLOSING_SLEEP_TIME = 1000;
   
   private static double epsilon = 1e-10;
   
   private int recordFrequency = 10;
   private int index = 15;
   private int inputPoint = 10;
   private int outputPoint = 50;
   private int middleIndex = outputPoint - 1;
   private int keyPoint = Math.round((inputPoint + outputPoint)/2);
   private int nonMiddleIndex = outputPoint + 1;
   private int ticksIncrease = 11;
   private int numberOfSimulationTicks = 2000;
   private int graphGroupNumberOfColumns = 2;
   private int numberOfTicksBeforeUpdatingGraphs = 2;
   private int numberOfTicksBeforeUpdatingGraphs2 = 5;
   private int indexStep = 2;
   private double simulateDT = 0.001; 
   private double simulateTime = 1.0;
   private double recordDT = 0.05;
   private double recordFreq = computeRecordFreq(recordDT, simulateDT);
   private double realTimeRate = 0.75;
   private double frameRate = 0.01;
   private double recomputedSecondsPerFrameRate = recomputeTiming(simulateDT, recordFreq, realTimeRate, frameRate);
   private double cameraFieldOfView = 1.5;
   private double secondsOfSimulation = 10;
   private double simulateDurationInSeconds = 2.5;
   private double[] cameraDollyOffsetXYZValues = {1.2, 2.2, 3.2};
   private double[] cameraTrackingOffsetXYZValues = {1.5, 2.5, 3.5};
   private double[] cameraDollyXYZVarValues = {1.5, 1.6, 1.7}; 
   private double[] cameraTrackingXYZVarValues = {0.5, 0.6, 0.7};
   private double[] cameraFixXYZValues = {1.8, 2.8, 3.8};
   private double[] cameraFixXYZValues2 = {1.1, 2.1, 3.1};
   private double[] cameraPositionXYZValues = {1.3, 2.3, 3.3};
   private double[] cameraClipDistancesNearFarValues = {1.75, 2.75};
   
   private Point location = new Point(25, 50);
   private FallingBrickRobot simpleRobot = new FallingBrickRobot();
   private String viewportName = "viewportName";
   private String rootRegistryName = "root";
   private String simpleRegistryName = "simpleRegistry";
   private String searchString = "d";
   private String searchStringStart = "q";
   private String[] cameraTrackingXYZVarNames = new String[] {"simpleCameraTrackingVarNameX", "simpleCameraTrackingVarNameY", "simpleCameraTrackingVarNameZ"};
   private String[] cameraDollyXYZVarNames = new String[] {"simpleCameraDollyVarNameX", "simpleCameraDollyVarNameY", "simpleCameraDollyVarNameZ"};
   private String configurationName = "simpleCOnfigurationName";
   private String cameraConfigurationName  = "simpleCameraConfigurationName";
   private String viewportConfigurationName = "simpleViewportConfiguration";
   private String varGroupName = "simpleVarGroup";
   private String varGroupName2 = "simpleVarGroup2";
   private String varGroupName3 = "simpleVarGroup3";
   private String graphGroupName = "simpleGraphGroup";
   private String graphGroupName2 = "simpleGraphGroup2";
   private String graphGroupName3 = "simpleGraphGroup3";
   private String graphGroupName4 = "simpleGraphGroup4";
   private String graphGroupName5 = "simpleGraphGroup5";
   private String entryBoxGroupName = "simpleEntryBoxGroup";
   private String entryBoxGroupName2 = "simpleEntryBoxGroup2";
   private String entryBoxGroupName3 = "simpleEntryBoxGroup3";
   private String[] graphConfigurationNames = {"simpleGraphConfiguration", "simpleGraphConfiguration2"};
   private String extraPanelConfigurationName = "simpleExtraPanelConfigurationName";
   private String[][] graphGroupVars = {cameraTrackingXYZVarNames, cameraDollyXYZVarNames};
   private String[][][] graphGroupVarsWithConfig = {{cameraTrackingXYZVarNames, {"config_1"}}, {cameraDollyXYZVarNames, {"config_2"}}};
   private String simpleRobotFirstVariableName = getFirstVariableNameFromRobotRegistry(simpleRobot);
   private String simpleRobotLastVariableName = getLastVariableNameFromRobotRegistry(simpleRobot);
   private String simpleRobotRegistryNameSpace = getRegistryNameSpaceFromRobot(simpleRobot);
   private String[] regularExpressions = new String[] {"gc.*.fs"};
   private Dimension dimension = new Dimension(250, 350);
   private NameSpace simpleRegistryNameSpace = new NameSpace(rootRegistryName + "." + simpleRegistryName);
   private YoVariableRegistry simpleRegistry = new YoVariableRegistry(simpleRegistryName);
   private YoVariableRegistry dummyRegistry = new YoVariableRegistry("dummyRegistry");
   private Link staticLink = new Link("simpleLink");
   private Graphics3DObject staticLinkGraphics = staticLink.getLinkGraphics();
   private Graphics3DNodeType graphics3DNodeType = Graphics3DNodeType.GROUND;
   private ExternalForcePoint simpleExternalForcePoint = new ExternalForcePoint("simpleExternalForcePoint", dummyRegistry);
   private DynamicGraphicObject dynamicGraphicObject = new DynamicGraphicVector("simpleDynamicGraphicObject", simpleExternalForcePoint);
   private BooleanYoVariable exitActionListenerHasBeenNotified = new BooleanYoVariable("exitActionListenerHasBeenNotified", dummyRegistry); 
   private BooleanYoVariable simulationRewoundListenerHasBeenNotified = new BooleanYoVariable("simulationRewoundListenerHasBeenNotified", dummyRegistry);
   private BooleanYoVariable simulationDoneListenerHasBeenNotified = new BooleanYoVariable("simulationDoneListenerHasBeenNotified", dummyRegistry);
   private BooleanYoVariable setSimulationDoneCriterion = new BooleanYoVariable("setSimulationDoneCriterion", dummyRegistry);
   private ExtraPanelConfiguration extraPanelConfiguration = createExtraPanelConfigurationWithPanel(extraPanelConfigurationName);
   private CameraConfiguration cameraConfiguration = createCameraConfiguration(cameraConfigurationName);
   private ViewportConfiguration viewportConfiguration = createViewportConfiguration(viewportConfigurationName);
   private GraphConfiguration[] graphConfigurations = createGraphConfigurations(graphConfigurationNames);
   private SimulationConstructionSet scs;
   
   @BeforeClass 
   public static void setUpOnce() 
   {
      FailOnThreadViolationRepaintManager.install();
   }
   
   @Before
   public void createAndStartSCSWithRobot()
   {
      scs = new SimulationConstructionSet(simpleRobot);
      scs.setFrameMaximized();
      scs.startOnAThread();
   }
   
   @After
   public void closeSCS()
   {
      ThreadTools.sleep(CLOSING_SLEEP_TIME);
      scs.closeAndDispose();
      scs = null;
   }

   @Test
   public void testSimulationConstructionSetMiscellaneous() throws AWTException
   {
      Robot[] robotFromSCS = scs.getRobots();
      assertEquals(simpleRobot, robotFromSCS[0]);

      YoVariableRegistry rootRegistryFromSCS = scs.getRootRegistry();
      String  rootRegistryNameFromSCS = rootRegistryFromSCS.getName();
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
      boolean isGraphsUpdatedDuringPlaybackFromSCS = scs.isGraphsUpdatedDuringPlayback();
      assertFalse(isGraphsUpdatedDuringPlaybackFromSCS);
      
      scs.setGraphsUpdatedDuringPlayback(true);
      boolean isGraphsUpdatedDuringPlaybackFromSCS2 = scs.isGraphsUpdatedDuringPlayback();
      assertTrue(isGraphsUpdatedDuringPlaybackFromSCS2);   
      
      scs.setScrollGraphsEnabled(true);
      boolean isScrollGraphsEnabled = scs.isSafeToScroll();
      assertTrue(isScrollGraphsEnabled);
      
      scs.setScrollGraphsEnabled(false);
      boolean isScrollGraphsEnabled2 = scs.isSafeToScroll();
      assertFalse(isScrollGraphsEnabled2);
   }
   
   @Test
   public void testSimulationManagement()
   {
      double startTime = scs.getTime();
      simulateForTime(scs, simulateTime);
      double endTime = scs.getTime();
      assertEquals(simulateTime, endTime-startTime, 1e-7);
           
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
      assertEquals(initialTime+DT, finalTime, epsilon);
      
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
   
   
   @Test
   public void testFrameMethods()
   {
      scs.setFrameSize(dimension);
      Dimension dimensionFromSCS = scs.getJFrame().getBounds().getSize();
      assertEquals(dimension.height, dimensionFromSCS.height, epsilon);
      assertEquals(dimension.width, dimensionFromSCS.width, epsilon);

      scs.setFrameLocation(location.x, location.y);
      Point locationFromSCS = scs.getJFrame().getLocation();
      assertEquals(location.x, locationFromSCS.x, epsilon);
      assertEquals(location.y, locationFromSCS.y, epsilon);

      scs.setFrameMaximized();
      int frameStateFromSCS = scs.getJFrame().getExtendedState();
      assertEquals(Frame.MAXIMIZED_BOTH, frameStateFromSCS);

      scs.setFrameAlwaysOnTop(true);
      boolean alwaysOnTopFromSCS = scs.getJFrame().isAlwaysOnTop();
      assertEquals(true, alwaysOnTopFromSCS);
      
      scs.setFrameAlwaysOnTop(false);
      alwaysOnTopFromSCS = scs.getJFrame().isAlwaysOnTop();
      assertEquals(false, alwaysOnTopFromSCS);
   }
   
   @Test
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
   }
   
   @Test
   public void test3DGraphicsMethods()
   {
      Graphics3DNode graphics3DNodeFromSCS = scs.addStaticLinkGraphics(staticLinkGraphics);
      assertNotNull(graphics3DNodeFromSCS);
      
      Graphics3DNode graphics3DNodeFromSCS2 = scs.addStaticLinkGraphics(staticLinkGraphics, graphics3DNodeType);
      assertNotNull(graphics3DNodeFromSCS2);
      
      GraphicsDynamicGraphicsObject graphicsDynamicGraphicsObjectFromSCS = scs.addDynamicGraphicObject(dynamicGraphicObject);
      assertNotNull(graphicsDynamicGraphicsObjectFromSCS);
      
      GraphicsDynamicGraphicsObject graphicsDynamicGraphicsObjectFromSCS2 = scs.addDynamicGraphicObject(dynamicGraphicObject, true);
      assertNotNull(graphicsDynamicGraphicsObjectFromSCS2);
      
      GraphicsDynamicGraphicsObject graphicsDynamicGraphicsObjectFromSCS3 = scs.addDynamicGraphicObject(dynamicGraphicObject, false);
      assertNotNull(graphicsDynamicGraphicsObjectFromSCS3);
      
      scs.setGroundVisible(false);
      boolean isGroundVisibleFromSCS = stateIfTerrainIsVisible(scs);
      assertTrue(isGroundVisibleFromSCS);
      
      scs.setGroundVisible(true);
      boolean isGroundVisibleFromSCS2 = stateIfTerrainIsVisible(scs);
      assertTrue(isGroundVisibleFromSCS2);
   }
   
   @Test
   public void testGetVariableMethods() throws AWTException
   {
      ArrayList<YoVariable> allVariablesFromRobot = simpleRobot.getAllVariables();
      ArrayList<YoVariable> allVariablesFromSCS = scs.getAllVariables();
      assertEquals(allVariablesFromRobot, allVariablesFromSCS);
      
      int allVariablesArrayFromRobot = simpleRobot.getAllVariablesArray().length;
      int allVariablesArrayFromSCS = scs.getAllVariablesArray().length;
      assertEquals(allVariablesArrayFromRobot, allVariablesArrayFromSCS);
      
      YoVariable yoVariableFromSCS = scs.getVariable(simpleRobotFirstVariableName);
      String variableNameFromSCS = yoVariableFromSCS.getName();
      assertEquals(simpleRobotFirstVariableName, variableNameFromSCS);
      
      YoVariable yoVariableFromRobot = simpleRobot.getVariable(simpleRobotFirstVariableName);
      YoVariable yoVariableFromSCS2 = scs.getVariable(simpleRobotRegistryNameSpace, simpleRobotFirstVariableName);
      assertEquals(yoVariableFromRobot, yoVariableFromSCS2);
      
      ArrayList<YoVariable> yoVariableArrayFromRobot =  simpleRobot.getVariables(simpleRobotRegistryNameSpace, simpleRobotFirstVariableName);
      ArrayList<YoVariable> yoVariableArrayFromSCS =  scs.getVariables(simpleRobotRegistryNameSpace, simpleRobotFirstVariableName);
      assertEquals(yoVariableArrayFromRobot, yoVariableArrayFromSCS);
      
      ArrayList<YoVariable> yoVariableFromRobot2 = simpleRobot.getVariables(simpleRobotFirstVariableName);
      ArrayList<YoVariable> yoVariableFromSCS3 = scs.getVariables(simpleRobotFirstVariableName);
      assertEquals(yoVariableFromRobot2, yoVariableFromSCS3); 
      
      ArrayList<YoVariable> yoVariableFromRobot3 = simpleRobot.getVariables(simpleRobotRegistryNameSpace);
      ArrayList<YoVariable> yoVariableFromSCS4 = scs.getVariables(simpleRobotRegistryNameSpace);
      assertEquals(yoVariableFromRobot3, yoVariableFromSCS4); 
      
      boolean hasUniqueVariableRobot = simpleRobot.hasUniqueVariable(simpleRobotFirstVariableName);
      boolean hasUniqueVariableSCS = scs.hasUniqueVariable(simpleRobotFirstVariableName);
      assertEquals(hasUniqueVariableRobot, hasUniqueVariableSCS);
      
      boolean hasUniqueVariableRobot2 = simpleRobot.hasUniqueVariable(simpleRobotRegistryNameSpace, simpleRobotFirstVariableName);
      boolean hasUniqueVariableSCS2 = scs.hasUniqueVariable(simpleRobotRegistryNameSpace, simpleRobotFirstVariableName);
      assertEquals(hasUniqueVariableRobot2, hasUniqueVariableSCS2);
      
      ArrayList<YoVariable> arrayOfVariablesContainingRobot = getSimpleRobotVariablesThatContain(searchString, false, simpleRobot);
      ArrayList<YoVariable> arrayOfVariablesContainingSCS = scs.getVariablesThatContain(searchString);
      assertEquals(arrayOfVariablesContainingRobot, arrayOfVariablesContainingSCS);
      
      ArrayList<YoVariable> arrayOfVariablesContainingRobot2 = getSimpleRobotVariablesThatContain(searchString, true, simpleRobot);
      ArrayList<YoVariable> arrayOfVariablesContainingSCS2 = scs.getVariablesThatContain(searchString, true);
      assertEquals(arrayOfVariablesContainingRobot2, arrayOfVariablesContainingSCS2);
      
      ArrayList<YoVariable> arrayOfVariablesStartingRobot = getSimpleRobotVariablesThatStartWith(searchStringStart, simpleRobot);
      ArrayList<YoVariable> arrayOfVariablesStartingSCS = scs.getVariablesThatStartWith(searchStringStart);
      assertEquals(arrayOfVariablesStartingRobot, arrayOfVariablesStartingSCS);
      
      String[] varNames =  getVariableNamesGivenArrayListOfYoVariables(arrayOfVariablesContainingRobot);
      ArrayList<YoVariable>  arrayOfVariablesRegExprRobot = getSimpleRobotRegExpVariables(varNames, regularExpressions, simpleRobot);
      ArrayList<YoVariable>  arrayOfVariablesRegExprSCS = scs.getVars(varNames, regularExpressions);
      assertEquals(arrayOfVariablesRegExprRobot, arrayOfVariablesRegExprSCS);
   }
   
   @Test
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
   }
   
   @Test
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
      assertEquals(ticksIncrease*ticksPerCycle, currentSCSIndex5, epsilon);
      
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

      scs.stopSimulationThread();
      boolean isThreadRunningFromSCS = scs.isSimulationThreadUpAndRunning();
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
      assertEquals(keyPoint-1, currentIndexFromSCS, epsilon);
      
      setInputAndOutputPointsWithoutCroppingInSCS(scs, inputPoint, outputPoint);
      scs.setIndex(keyPoint);
      scs.stepForwardNow(indexStep);
      int currentIndexFromSCS2 = scs.getIndex();
      assertEquals(keyPoint+indexStep, currentIndexFromSCS2, epsilon);
      
      setInputAndOutputPointsWithoutCroppingInSCS(scs, inputPoint, outputPoint);
      scs.setIndex(keyPoint);
      scs.stepForward(indexStep);
      scs.run();
      int currentIndexFromSCS3 = scs.getIndex();
      assertEquals(keyPoint+indexStep, currentIndexFromSCS3, epsilon);
      
      setInputAndOutputPointsWithoutCroppingInSCS(scs, inputPoint, outputPoint);
      scs.setIndex(keyPoint);
      scs.stepForward();
      scs.run();
      int currentIndexFromSCS4 = scs.getIndex();
      assertEquals(keyPoint+1, currentIndexFromSCS4, epsilon);
      
      setInputAndOutputPointsWithoutCroppingInSCS(scs, inputPoint, outputPoint);
      scs.setIndex(keyPoint);
      scs.stepBackward(indexStep);
      scs.run();
      int currentIndexFromSCS5 = scs.getIndex();
      assertEquals(keyPoint-indexStep, currentIndexFromSCS5, epsilon);
      
      setInputAndOutputPointsWithoutCroppingInSCS(scs, inputPoint, outputPoint);
      scs.setIndex(keyPoint);
      scs.stepBackward();
      scs.run();
      int currentIndexFromSCS6 = scs.getIndex();
      assertEquals(keyPoint-1, currentIndexFromSCS6, epsilon);
   }
   
   @Test
   public void testVariablesMethods()
   {
      addDoubleYoVariablesInSCSRegistry(cameraDollyXYZVarNames, cameraDollyXYZVarValues, scs);
      
      scs.setupEntryBox(simpleRobotFirstVariableName);
      boolean entryBoxIsInSCS = scsContainsTheEntryBox(scs, simpleRobotFirstVariableName);
      assertTrue(entryBoxIsInSCS);
      
      scs.setupEntryBox(simpleRobotLastVariableName);
      boolean entryBoxIsInSCS2 = scsContainsTheEntryBox(scs, simpleRobotLastVariableName);
      assertTrue(entryBoxIsInSCS2);
      
      scs.setupEntryBox(cameraDollyXYZVarNames);   
      boolean entryBoxesAreInSCS = scsContainsTheEntryBoxes(scs, cameraDollyXYZVarNames);
      assertTrue(entryBoxesAreInSCS);
      
      scs.setupGraph(simpleRobotFirstVariableName);
      boolean graphIsInSCS = scsContainsTheGraph(scs, simpleRobotFirstVariableName);
      assertTrue(graphIsInSCS);
      
      scs.setupGraph(simpleRobotLastVariableName);
      boolean graphIsInSCS2 = scsContainsTheGraph(scs, simpleRobotLastVariableName);
      assertTrue(graphIsInSCS2);

      scs.setupGraph(cameraDollyXYZVarNames);
      boolean graphsAreInSCS = scsContainsTheGraphs(scs, cameraDollyXYZVarNames);
      assertTrue(graphsAreInSCS); 
   }
   
   @Test
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
   
   @Test
   public void testSimulationListeners()
   { 
      SimulationDoneListener simulationDoneListener = createSimulationDoneListener();
      SimulationDoneCriterion simulationDoneCriterion = createSimulationDoneCriterion();
      scs.addSimulateDoneListener(simulationDoneListener);
      scs.setSimulateDoneCriterion(simulationDoneCriterion);
      
      simulationDoneListenerHasBeenNotified.set(false);
      callSCSMethodSimulateOneTimeStep(scs);
      assertTrue(simulationDoneListenerHasBeenNotified.getBooleanValue());
      
      simulationDoneListenerHasBeenNotified.set(false);
      scs.removeSimulateDoneListener(simulationDoneListener);
      assertFalse(simulationDoneListenerHasBeenNotified.getBooleanValue());

      simulationDoneListenerHasBeenNotified.set(false);
      scs.simulate(Double.MAX_VALUE);
      boolean isSCSSimulatingBeforeCriterion = scs.isSimulating();
      setSimulationDoneCriterion.set(true);
      askThreadToSleep(100);
      boolean isSCSSimulatingAfterCriterion = scs.isSimulating();
      assertTrue(isSCSSimulatingBeforeCriterion);
      assertFalse(isSCSSimulatingAfterCriterion);    
   }
   
   @Test
   public void testDatatExporting()
   {
      simulateForTime(scs, simulateTime);
      int initialInPoint = scs.getInPoint();
      int initialOutPoint = scs.getOutPoint();
      int initialInOutBufferLength = getInOutBufferLengthFromSCS(scs);
      int initialBufferSize = getBufferSizeFromSCS(scs);
      CaptureDevice captureDevice = getCaptureDeviceFromSCS(scs);

      addAndSubtractOneFromInAndOutPointIndexWithoutCrop(scs);
      scs.cropBuffer();
      int currentInPoint = scs.getInPoint();
      int currentBufferSize = scs.getDataBuffer().getBufferSize();
      assertEquals(0, currentInPoint, epsilon);
      assertEquals(initialInOutBufferLength-2, currentBufferSize, epsilon);
      
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
      
      scs.changeBufferSize(initialBufferSize*2);
      int bufferSizeFromSCS = getBufferSizeFromSCS(scs);
      assertEquals(initialBufferSize*2, bufferSizeFromSCS, epsilon);
      
      scs.setMaxBufferSize(initialBufferSize*3);
      int maxBufferSizeFromSCS = getMaxBufferSizeFromSCS(scs);
      assertEquals(initialBufferSize*3, maxBufferSizeFromSCS, epsilon);
      
      BufferedImage  bufferedImage = scs.exportSnapshotAsBufferedImage(captureDevice);
      assertNotNull(bufferedImage);
      
      BufferedImage  bufferedImage2 = scs.exportSnapshotAsBufferedImage();
      assertNotNull(bufferedImage2);
      
      addDoubleYoVariablesInSCSRegistry(cameraTrackingXYZVarNames, cameraTrackingXYZVarValues, scs);
      File fileOne2 = new File("fileOne");
      File fileTwo2 = new File("fileTwo");
      scs.writeSpreadsheetFormattedData("all", fileOne2);
      scs.writeSpreadsheetFormattedData("all", fileTwo2);
      assertTheFileContainsTheVariables(fileOne2, cameraTrackingXYZVarNames);
      fileOne2.delete();
      fileTwo2.delete();  
   }
   
      
   // local methods
   
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
      scs.setIndex(currentInPoint+1);
      scs.setInPoint();
   }
   
   private void subtractOneToOutPointIndexWithoutCrop(SimulationConstructionSet scs)
   {
      int currentOutPoint = scs.getOutPoint();
      scs.setIndex(currentOutPoint-1);
      scs.setOutPoint();
   }
   
   private void askThreadToSleep(long milliseconds)
   {
      try
      {
         Thread.sleep(milliseconds);
      }
      catch (InterruptedException e)
      {
         e.printStackTrace();
      }
   }
   
   private SimulationDoneCriterion createSimulationDoneCriterion()
   {
      SimulationDoneCriterion criterion = new SimulationDoneCriterion()
      {
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
         public void simulationDone()
         {
            simulationDoneListenerHasBeenNotified.set(true);
         }

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
      for(int i = 0; i<strings.length; i++)
      {
         assertArrayOfStringsContainsTheString(array, strings[i]);
      }
   }
   
   private GraphConfiguration[] createGraphConfigurations(String[] graphConfigurationNames)
   {
      GraphConfiguration[] graphConfigurations = new GraphConfiguration[graphConfigurationNames.length];
      for(int i = 0; i<graphConfigurationNames.length; i++)
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
      return getRepresentationOfCurrentView(scs).contains(currentViewName);
   }
   
   private String getRepresentationOfCurrentView(SimulationConstructionSet scs)
   {
      return scs.getGUI().getXMLStyleRepresentationofMultiViews();
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
      ExtraPanelConfiguration extraPanelConfiguration = new ExtraPanelConfiguration(name);
      Button panel = new Button();
      extraPanelConfiguration.setupPanel(panel);
      
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
      
      for(int i = 0; i<array.length; i++)
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
      
      for(int i = 0; i<robots.length; i++)
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
      SimulationRewoundListener simulationRewoundListener = createSimulationRewoundListener();
      scs.attachSimulationRewoundListener(simulationRewoundListener);
   }
   
   private void simulateForTime(SimulationConstructionSet scs, double simulateTime)
   {
      scs.simulate(simulateTime);
      while(scs.isSimulating())
      {
         ThreadTools.sleep(100L);
      }
   }

   private SimulationRewoundListener createSimulationRewoundListener()
   {
      SimulationRewoundListener ret = new SimulationRewoundListener()
      {
         public void simulationWasRewound()
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
         public void exitActionPerformed()
         {
            exitActionListenerHasBeenNotified.set(true);
         }
      };
      
      return ret;
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
      return new double[] {x, y, z};
   }
   
   private double[] getCameraDollyXYZVars(SimulationConstructionSet scs)
   {
      CameraTrackAndDollyYoVariablesHolder cameraTrackAndDollyYoVariablesHolder = getCameraTrackAndDollyVariablesHolder(scs); 
      double x = cameraTrackAndDollyYoVariablesHolder.getDollyX();
      double y = cameraTrackAndDollyYoVariablesHolder.getDollyY();
      double z = cameraTrackAndDollyYoVariablesHolder.getDollyZ();
      return new double[] {x, y, z};
   }
   
   private double[] getCameraTrackingOffsetXYZValues(SimulationConstructionSet scs)
   {
      ClassicCameraController classicCameraController = getClassicCameraController(scs); 
      double dx = classicCameraController.getTrackingXOffset();
      double dy = classicCameraController.getTrackingYOffset();
      double dz = classicCameraController.getTrackingZOffset();
      return new double[] {dx, dy, dz};
   }

   private double[] getCameraDollyOffsetXYZValues(SimulationConstructionSet scs)
   {
      ClassicCameraController classicCameraController = getClassicCameraController(scs); 
      double dx = classicCameraController.getDollyXOffset();
      double dy = classicCameraController.getDollyYOffset();
      double dz = classicCameraController.getDollyZOffset();
      return new double[] {dx, dy, dz};
   }
   
   private double[] getCameraFixXYZValues(SimulationConstructionSet scs)
   {
      ClassicCameraController classicCameraController = getClassicCameraController(scs); 
      double x = classicCameraController.getFixX();
      double y = classicCameraController.getFixY();
      double z = classicCameraController.getFixZ();
      return new double[] {x, y, z};
   }
   
   private double[] getCameraNearFarValues(SimulationConstructionSet scs)
   {
      ClassicCameraController classicCameraController = getClassicCameraController(scs); 
      double near = classicCameraController.getClipNear();
      double far = classicCameraController.getClipFar();
      return new double[] {near, far};
   }
   
   private double[] getCameraPositionXYZValues(SimulationConstructionSet scs)
   {
      ClassicCameraController classicCameraController = getClassicCameraController(scs); 
      double x = classicCameraController.getCamX();
      double y = classicCameraController.getCamY();
      double z = classicCameraController.getCamZ();
      return new double[] {x, y, z};
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
   
   private void addDoubleYoVariablesInSCSRegistry(String[] varNames, double[] varValues, SimulationConstructionSet scs)
   {
      if(varNames.length == varValues.length)
      {
         YoVariableRegistry scsRegistry = scs.getRootRegistry();
         for(int i = 0; i< varNames.length; i++)
         {
            DoubleYoVariable doubleYoVariable = new DoubleYoVariable(varNames[i], scsRegistry);
            doubleYoVariable.set(varValues[i]);
         } 
      }
      else
      {
         System.out.print("Input arrays have different length.");
      }
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
      int lastIndex = robotModel.getRobotsYoVariableRegistry().getAllVariablesArray().length -1;
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
   
   private ArrayList<YoVariable> getSimpleRobotVariablesThatContain(String searchString, boolean caseSensitive, Robot robotModel)
   {
      ArrayList<YoVariable> currentlyMatched = robotModel.getAllVariables();
      ArrayList<YoVariable> ret = null;

      if (currentlyMatched != null)
      {
         if (!caseSensitive)
         {
            searchString = searchString.toLowerCase();
         }

         for (int i = 0; i < currentlyMatched.size(); i++)
         {
            YoVariable entry = currentlyMatched.get(i);

            if (entry.getName().toLowerCase().contains((searchString)))
            {
               if (ret == null)
               {
                  ret = new ArrayList<YoVariable>();
               }

               ret.add(entry);
            }
         }
      }

      return ret;
   }
   
   private ArrayList<YoVariable> getSimpleRobotVariablesThatStartWith(String searchString, Robot robotModel)
   {
      ArrayList<YoVariable> currentlyMatched = robotModel.getAllVariables();
      ArrayList<YoVariable> ret = null;

      for (int i = 0; i < currentlyMatched.size(); i++)
      {
         YoVariable Variable = currentlyMatched.get(i);

         if (Variable.getName().startsWith(searchString))
         {
            if (ret == null)
            {
               ret = new ArrayList<YoVariable>();
            }

            ret.add(Variable);
         }
      }

      return ret;
   }
   
   private String[] getVariableNamesGivenArrayListOfYoVariables(ArrayList<YoVariable> yoVariableList)
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
   
   private ArrayList<YoVariable> getSimpleRobotRegExpVariables(String[] varNames, String[] regularExpressions, Robot robotModel)
   {
      ArrayList<YoVariable> currentlyMatched = robotModel.getAllVariables();
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