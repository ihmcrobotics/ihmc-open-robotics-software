package com.yobotics.simulationconstructionset;

import static org.junit.Assert.assertEquals;

import java.awt.AWTException;
import java.awt.Dimension;
import java.awt.Frame;
import java.awt.Point;

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

      SimulationConstructionSet scs = new SimulationConstructionSet(simpleRobot);
      scs.setFrameMaximized();
      scs.startOnAThread();

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
      
      scs.setDT(simulateDT, recordFrequency);
      double simulateDTFromSCS = scs.getDT();
      assertEquals(simulateDT, simulateDTFromSCS , epsilon); 
      
      scs.setIndex(index);
      int indexFromSCS = scs.getIndex();
      assertEquals(index, indexFromSCS, epsilon);
      
      scs.setIndex(inputPoint);
      scs.setInPoint();
      scs.setIndex(outputPoint);
      scs.setOutPoint();
      boolean isMiddleIndexFromSCS = scs.isIndexBetweenInAndOutPoint(middleIndex);
      assertEquals(true, isMiddleIndexFromSCS);

      scs.setIndex(inputPoint);
      scs.setInPoint();
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
 

}
