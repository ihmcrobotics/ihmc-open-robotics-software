package com.yobotics.simulationconstructionset;

import static org.junit.Assert.assertEquals;

import java.awt.AWTException;
import java.awt.Dimension;
import java.awt.Frame;
import java.awt.Point;
import java.util.ArrayList;

import org.junit.Test;

import us.ihmc.utilities.ThreadTools;

import com.yobotics.simulationconstructionset.examples.FallingBrickRobot;
import com.yobotics.simulationconstructionset.gui.StandardSimulationGUI;

public class SimulationConstructionSetUsingDirectCallsTest
{
   // all tests assume that:
   // - the simpleRobot has been used to create the SimulationConstructionSet instance
   // - the registry "simpleRegistry" is empty
   
   public static double epsilon = 1e-10;
   
   int recordFrequency = 10;
   int index = 15;
   int inputPoint = 10;
   int outputPoint = 30;
   int middleIndex = 22;
   int nonMiddleIndex = 35;
   double simulateDT = 0.1; 
   double simulateTime = 1.0;
   double recordDT = 0.5;
   double recordFreq = computeRecordFreq(recordDT, simulateDT);
   double realTimeRate = 0.75;
   double frameRate = 0.5;
   double recomputedSecondsPerFrameRate = recomputeTiming(simulateDT, recordFreq, realTimeRate, frameRate);
   
   Dimension dimension = new Dimension(250, 350);
   FallingBrickRobot simpleRobot = new FallingBrickRobot();
   Point location = new Point(25, 50);
   String rootRegistryName = "root";
   String simpleRegistryName = "simpleRegistry";
   String searchString = "d";
   String searchStringStart = "q";
   String[] regularExpressions = new String[] {"gc.*.fs"};
   String simpleRobotFirstVariableName = getFirstVariableNameFromRobotRegistry(simpleRobot);
   String simpleRobotRegistryNameSpace = getRegistryNameSpaceFromRobot(simpleRobot);
   NameSpace simpleRegistryNameSpace = new NameSpace(rootRegistryName + "." + simpleRegistryName);
   YoVariableRegistry simpleRegistry = new YoVariableRegistry(simpleRegistryName);
   
   SimulationConstructionSet scs = createAndStartSCSWithRobot(simpleRobot);

   @Test
   public void testSimulationConstructionSetUsingDirectCalls() throws AWTException
   {
      double startTime = scs.getTime();
      scs.simulate(simulateTime);
      while(scs.isSimulating())
      {
         ThreadTools.sleep(100L);
      }
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
      
      YoVariableRegistry rootRegistryFromSCS = scs.getRootRegistry();
      String  rootRegistryNameFromSCS = rootRegistryFromSCS.getName();
      assertEquals(rootRegistryName, rootRegistryNameFromSCS);
      
      scs.addYoVariableRegistry(simpleRegistry);
      YoVariableRegistry simpleRegistryFromSCS = scs.getRootRegistry().getRegistry(simpleRegistryNameSpace);
      assertEquals(simpleRegistry, simpleRegistryFromSCS);
      
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

//      ThreadTools.sleepForever();
//    Robot simpleRobot2 = new Robot("simpleRobot2");
//    Robot[] robots = {simpleRobot, simpleRobot2};
//    
//    SimulationConstructionSet scs2 = new SimulationConstructionSet(robots);
//    generateSimulationFromDataFile
//     JButton button = new JButton("test");
      
      scs.closeAndDispose();
      scs = null;
   }
   
   @Test
   public void testTimingMethods()
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
      
      double ticksPerCycle = computeTicksPerPlayCycle(simulateDT, recordFreq, realTimeRate, frameRate);
      double ticksPerCycleFromSCS = scs.getTicksPerPlayCycle();
      assertEquals(ticksPerCycle, ticksPerCycleFromSCS, epsilon);
   }
   
   private String getRegistryNameSpaceFromRobot(Robot robotModel)
   {
      return robotModel.getRobotsYoVariableRegistry().getNameSpace().getName();
   }
   
   private String getFirstVariableNameFromRobotRegistry(Robot robotModel)
   {
      return robotModel.getRobotsYoVariableRegistry().getAllVariablesArray()[0].getName();
   }
   
   private SimulationConstructionSet createAndStartSCSWithRobot(Robot robotModel)
   {
      SimulationConstructionSet scs = new SimulationConstructionSet(robotModel);
      scs.setFrameMaximized();
      scs.startOnAThread();
      return scs;
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
      double ticksPerCycle = computeTicksPerPlayCycle(dt, recordFreq, realTimeRate, frameRate);
      double secondsPerFrameRate = ticksPerCycle * (dt * recordFreq) / realTimeRate;
      return secondsPerFrameRate;
   }
   
   private double computeTicksPerPlayCycle(double dt, double recordFreq, double realTimeRate, double frameRate)
   {
      return Math.max((int) (frameRate * realTimeRate / (dt * recordFreq)), 1);
   }

}