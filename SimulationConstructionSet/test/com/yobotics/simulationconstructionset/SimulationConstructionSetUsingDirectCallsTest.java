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
   public static double epsilon = 1e-10;

   @Test
   public void testSimulationConstructionSetUsingDirectCalls() throws AWTException
   {
      int recordFrequency = 10;
      int index = 15;
      int inputPoint = 10;
      int outputPoint = 30;
      int middleIndex = 22;
      double simulateDT = 0.1; 
      double simulateTime = 1.0;
      
      FallingBrickRobot simpleRobot = new FallingBrickRobot();
      Dimension dimension = new Dimension(250, 350);
      Point location = new Point(25, 50);
      String rootRegistryName = "root";
      String simpleRegistryName = "simpleRegistry";
      YoVariableRegistry simpleRegistry = new YoVariableRegistry(simpleRegistryName);
      NameSpace simpleRegistryNameSpace = new NameSpace(rootRegistryName + "." + simpleRegistryName);

      SimulationConstructionSet scs = createAndStartSCSWithRobot(simpleRobot);

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
      
      ArrayList<YoVariable> allVariablesFromRobot = simpleRobot.getAllVariables();
      ArrayList<YoVariable> allVariablesFromSCS = scs.getAllVariables();
      assertEquals(allVariablesFromRobot, allVariablesFromSCS);
      
      int allVariablesArrayFromRobot = simpleRobot.getAllVariablesArray().length;
      int allVariablesArrayFromSCS = scs.getAllVariablesArray().length;
      assertEquals(allVariablesArrayFromRobot, allVariablesArrayFromSCS);
      
      scs.setDT(simulateDT, recordFrequency);
      double simulateDTFromSCS = scs.getDT();
      assertEquals(simulateDT, simulateDTFromSCS , epsilon); 
      
      scs.setIndex(index);
      int indexFromSCS = scs.getIndex();
      assertEquals(index, indexFromSCS, epsilon);
      
      setInputAndOutputPointsInSCS(scs, inputPoint, outputPoint);
      boolean isMiddleIndexFromSCS = scs.isIndexBetweenInAndOutPoint(middleIndex);
      assertEquals(true, isMiddleIndexFromSCS);

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
      
      YoVariableRegistry rootRegistryFromSCS = scs.getRootRegistry();
      String  rootRegistryNameFromSCS = rootRegistryFromSCS.getName();
      assertEquals(rootRegistryName, rootRegistryNameFromSCS);
      
      scs.addYoVariableRegistry(simpleRegistry);
      YoVariableRegistry simpleRegistryFromSCS = scs.getRootRegistry().getRegistry(simpleRegistryNameSpace);
      assertEquals(simpleRegistry, simpleRegistryFromSCS);
       
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
}
