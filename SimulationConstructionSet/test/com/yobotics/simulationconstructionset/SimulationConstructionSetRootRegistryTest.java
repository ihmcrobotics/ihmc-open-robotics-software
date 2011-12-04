package com.yobotics.simulationconstructionset;

import static org.junit.Assert.*;

import org.junit.Test;

import com.yobotics.simulationconstructionset.gui.StandardSimulationGUI;

public class SimulationConstructionSetRootRegistryTest
{
   private static final boolean SHOW_GUI = false;
   
   @Test
   public void testRootRegistry()
   {
      Robot robot = new Robot("RobotsRootRegistry");
      
      YoVariableRegistry registryBeforeConstructionOne = new YoVariableRegistry("RegistryBeforeConstructionOne");
      robot.getRobotsYoVariableRegistry().addChild(registryBeforeConstructionOne);
      DoubleYoVariable variableBeforeConstructionOne = new DoubleYoVariable("variableBeforeConstructionOne", registryBeforeConstructionOne);
      
      SimulationConstructionSet scs = new SimulationConstructionSet(robot, SHOW_GUI);
      
      DoubleYoVariable variableAfterConstructionZero = new DoubleYoVariable("variableAfterConstructionZero", registryBeforeConstructionOne);

      YoVariableRegistry registryAfterConstructionOne = new YoVariableRegistry("RegistryAfterConstructionOne");
      DoubleYoVariable variableAfterConstructionOne = new DoubleYoVariable("variableAfterConstructionOne", registryAfterConstructionOne);
      scs.getRootRegistry().addChild(registryAfterConstructionOne);
      DoubleYoVariable variableAfterConstructionTwo = new DoubleYoVariable("variableAfterConstructionTwo", registryAfterConstructionOne);
      
      scs.startOnAThread();
      
      DoubleYoVariable variableAfterThreadZero = new DoubleYoVariable("variableAfterThreadZero", registryAfterConstructionOne);
//      sleep(100000);

      YoVariableRegistry registryAfterThreadOne = new YoVariableRegistry("RegistryAfterThreadOne");
      DoubleYoVariable variableAfterThreadOne = new DoubleYoVariable("variableAfterThreadOne", registryAfterThreadOne);
      registryAfterConstructionOne.addChild(registryAfterThreadOne);
      DoubleYoVariable variableAfterThreadTwo = new DoubleYoVariable("variableAfterThreadTwo", registryAfterThreadOne);

      YoVariableRegistry rootRegistry = scs.getRootRegistry();
      
      // Make sure the variables are in the registry chain...
      assertTrue(variableBeforeConstructionOne == rootRegistry.getVariable("variableBeforeConstructionOne"));
      assertTrue(variableAfterConstructionZero == rootRegistry.getVariable("variableAfterConstructionZero"));
      assertTrue(variableAfterConstructionOne == rootRegistry.getVariable("variableAfterConstructionOne"));
      assertTrue(variableAfterConstructionTwo == rootRegistry.getVariable("variableAfterConstructionTwo"));
      assertTrue(variableAfterThreadZero == rootRegistry.getVariable("variableAfterThreadZero"));
      assertTrue(variableAfterThreadOne == rootRegistry.getVariable("variableAfterThreadOne"));
      assertTrue(variableAfterThreadTwo == rootRegistry.getVariable("variableAfterThreadTwo"));
      
      // Make sure the variables are in the DataBuffer:
      DataBuffer dataBuffer = scs.getDataBuffer();
      assertTrue(variableBeforeConstructionOne == dataBuffer.getVariable("variableBeforeConstructionOne"));
      assertTrue(variableAfterConstructionZero == dataBuffer.getVariable("variableAfterConstructionZero"));
      assertTrue(variableAfterConstructionOne == dataBuffer.getVariable("variableAfterConstructionOne"));
      assertTrue(variableAfterConstructionTwo == dataBuffer.getVariable("variableAfterConstructionTwo"));
      assertTrue(variableAfterThreadZero == dataBuffer.getVariable("variableAfterThreadZero"));
      assertTrue(variableAfterThreadOne == dataBuffer.getVariable("variableAfterThreadOne"));
      assertTrue(variableAfterThreadTwo == dataBuffer.getVariable("variableAfterThreadTwo"));

      // Make sure the variables are on the GUI:
      StandardSimulationGUI gui = scs.getStandardSimulationGUI();
//      gui.get
      
//      sleep(100000);
   }

   private static void sleep(long sleepMillis)
   {
      try
      {
         Thread.sleep(sleepMillis);
      } 
      catch (InterruptedException e)
      {
      }
      
   }
}
