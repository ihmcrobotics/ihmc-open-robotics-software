package com.yobotics.simulationconstructionset;


import java.awt.AWTException;
import java.awt.Dimension;
import java.awt.MouseInfo;
import java.awt.event.InputEvent;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

public class SimulationConstructionSetTest
{

   @Before
   public void setUp() throws Exception
   {
   }

   @After
   public void tearDown() throws Exception
   {
   }
   
   @Test
   public void testOne() throws AWTException
   {
      SimpleRobot simpleRobot = new SimpleRobot();
      
      SimulationConstructionSet scs = new SimulationConstructionSet(simpleRobot);
      scs.setFrameMaximized();
      scs.startOnAThread();
      
      java.awt.Robot guiRobot = new java.awt.Robot();
      
      guiRobot.delay(2000);
      
      singleLeftClickAt(guiRobot, 10, 682);
      singleMiddleClickAt(guiRobot, 514, 838);
      singleLeftClickAt(guiRobot, 712, 600);

      sleepForever();
   }
   
   private void singleLeftClickAt(java.awt.Robot guiRobot, int x, int y)
   {
      guiRobot.mouseMove(x, y);
      guiRobot.delay(2000);
      guiRobot.mousePress(InputEvent.BUTTON1_MASK);
      guiRobot.mouseRelease(InputEvent.BUTTON1_MASK);
   }
   
   private void singleMiddleClickAt(java.awt.Robot guiRobot, int x, int y)
   {
      guiRobot.mouseMove(x, y);
      guiRobot.delay(2000);
      guiRobot.mousePress(InputEvent.BUTTON2_MASK);
      guiRobot.mouseRelease(InputEvent.BUTTON2_MASK);
   }
   
   private class SimpleRobot extends Robot
   {
      public SimpleRobot()
      {
         super("SimpleRobot");
         
      }
   }
   
   private void sleepForever()
   {
      while(true)
      {
         try
         {
            Thread.sleep(1000);
            
           
//            System.out.println(MouseInfo.getPointerInfo().getLocation());
         } 
         catch (InterruptedException e)
         {
         }
      }
   }

}
