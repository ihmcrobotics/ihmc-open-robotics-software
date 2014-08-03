package com.yobotics.simulationconstructionset;

import static org.junit.Assert.assertArrayEquals;
import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertTrue;

import java.awt.AWTException;
import java.awt.Dimension;
import java.awt.Frame;
import java.awt.Point;
import java.util.ArrayList;

import org.fest.swing.edt.FailOnThreadViolationRepaintManager;
import org.junit.After;
import org.junit.Before;
import org.junit.BeforeClass;
import org.junit.Test;

import us.ihmc.graphics3DAdapter.camera.ClassicCameraController;
import us.ihmc.graphics3DAdapter.graphics.Graphics3DObject;
import us.ihmc.graphics3DAdapter.structure.Graphics3DNode;
import us.ihmc.graphics3DAdapter.structure.Graphics3DNodeType;
import us.ihmc.utilities.ThreadTools;

import com.yobotics.simulationconstructionset.examples.FallingBrickRobot;
import com.yobotics.simulationconstructionset.graphics.GraphicsDynamicGraphicsObject;
import com.yobotics.simulationconstructionset.gui.StandardSimulationGUI;
import com.yobotics.simulationconstructionset.gui.camera.CameraTrackAndDollyYoVariablesHolder;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObject;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicVector;

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
   private int outputPoint = 30;
   private int middleIndex = 22;
   private int nonMiddleIndex = 35;
   private int ticksIncrease = 11;
   private double simulateDT = 0.001; 
   private double simulateTime = 1.0;
   private double recordDT = 0.05;
   private double recordFreq = computeRecordFreq(recordDT, simulateDT);
   private double realTimeRate = 0.75;
   private double frameRate = 0.01;
   private double recomputedSecondsPerFrameRate = recomputeTiming(simulateDT, recordFreq, realTimeRate, frameRate);
   private double[] cameraDollyOffsetXYZValues = {1.2, 2.2, 3.2};
   private double[] cameraTrackingOffsetXYZValues = {1.5, 2.5, 3.5};
   private double[] cameraDollyXYZVarValues = {1.5, 1.6, 1.7}; 
   private double[] cameraTrackingXYZVarValues = {0.5, 0.6, 0.7};
   private double[] cameraFixXYZValues = {1.8, 2.8, 3.8};
   private double[] cameraPositionXYZValues = {1.3, 2.3, 3.3};
   
   private Dimension dimension = new Dimension(250, 350);
   private FallingBrickRobot simpleRobot = new FallingBrickRobot();
   private Point location = new Point(25, 50);
   private String rootRegistryName = "root";
   private String simpleRegistryName = "simpleRegistry";
   private String searchString = "d";
   private String searchStringStart = "q";
   private String[] regularExpressions = new String[] {"gc.*.fs"};
   private String simpleRobotFirstVariableName = getFirstVariableNameFromRobotRegistry(simpleRobot);
   private String simpleRobotRegistryNameSpace = getRegistryNameSpaceFromRobot(simpleRobot);
   private String[] cameraTrackingXYZVarNames = new String[] {"simpleCameraTrackingVarNameX", "simpleCameraTrackingVarNameY", "simpleCameraTrackingVarNameZ"};
   private String[] cameraDollyXYZVarNames = new String[] {"simpleCameraDollyVarNameX", "simpleCameraDollyVarNameY", "simpleCameraDollyVarNameZ"};
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
   SimulationConstructionSet scs;
   
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
   public void testSimulationConstructionSetUsingDirectCalls() throws AWTException
   {
      double startTime = scs.getTime();
      simulateForTime(scs, simulateTime);
      double endTime = scs.getTime();
      assertEquals(simulateTime, endTime-startTime, 1e-7);
      
      Robot[] robotFromSCS = scs.getRobots();
      assertEquals(simpleRobot, robotFromSCS[0]);
      
      scs.setIndex(index);
      int indexFromSCS = scs.getIndex();
      assertEquals(index, indexFromSCS, epsilon);
      
      setInputAndOutputPointsInSCS(scs, inputPoint, outputPoint);
      boolean isMiddleIndexFromSCS = scs.isIndexBetweenInAndOutPoint(middleIndex);
      assertEquals(true, isMiddleIndexFromSCS);
      isMiddleIndexFromSCS = scs.isIndexBetweenInAndOutPoint(nonMiddleIndex);
      assertEquals(false, isMiddleIndexFromSCS);

      setInputPointInSCS(scs, inputPoint);
      scs.setIndex(outputPoint);
      StandardSimulationGUI GUIFromSCS = scs.getStandardSimulationGUI();
      GUIFromSCS.gotoInPointNow();
      int inputPointFromSCS = scs.getIndex();
      assertEquals(inputPoint, inputPointFromSCS);

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
      
//    
//    SimulationConstructionSet scs2 = new SimulationConstructionSet(robots);
//    generateSimulationFromDataFile
//     
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
   }
   
   // local methods

   
   private void createSimulationRewoundListenerAndAttachToSCS(SimulationConstructionSet scs)
   {
      SimulationRewoundListener simulationRewoundListener = createSimulationRewoundListener();
      scs.attachSimulationRewoundListener(simulationRewoundListener);
   }
   
//   private void setAllTimingValuesInSCS(SimulationConstructionSet scs, double simulateDT, 
//                                        int recordFrequency, double recordDT, double realTimeRate, double frameRate)
//   {
//      scs.setDT(simulateDT, recordFrequency);
//      scs.setRecordDT(recordDT);
//      scs.setPlaybackRealTimeRate(realTimeRate);
//      scs.setPlaybackDesiredFrameRate(frameRate);
//   }
   
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
   
   private double[] getCameraPositionXYZValues(SimulationConstructionSet scs)
   {
      ClassicCameraController classicCameraController = getClassicCameraController(scs); 
      double x = classicCameraController.getCamX();
      double y = classicCameraController.getCamY();
      double z = classicCameraController.getCamZ();
      return new double[] {x, y, z};
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
         for(int i =0; i< varNames.length; i++)
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
   
   private void setInputAndOutputPointsInSCS(SimulationConstructionSet scs, int inputPointIndex, int outputPointIndex)
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