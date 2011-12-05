package com.yobotics.simulationconstructionset;

import static org.junit.Assert.assertTrue;

import org.junit.Ignore;
import org.junit.Test;

import com.yobotics.simulationconstructionset.gui.CombinedVarPanel;
import com.yobotics.simulationconstructionset.gui.StandardSimulationGUI;
import com.yobotics.simulationconstructionset.gui.VarPanel;

public class SimulationConstructionSetRootRegistryTest
{
   private static final boolean SHOW_GUI = false;
   
   @Test
   public void testRootRegistryNothingFancy()
   {
      Robot robot = new Robot("RobotsRootRegistry");

      YoVariableRegistry registryOne = new YoVariableRegistry("RegistryOne");
      robot.getRobotsYoVariableRegistry().addChild(registryOne);
      DoubleYoVariable variableOne = new DoubleYoVariable("variableOne", registryOne);
      
      SimulationConstructionSet scs = new SimulationConstructionSet(robot, SHOW_GUI);
      scs.startOnAThread();

      YoVariableRegistry rootRegistry = scs.getRootRegistry();
      DataBuffer dataBuffer = scs.getDataBuffer();

      assertTrue(variableOne == rootRegistry.getVariable("variableOne"));
      assertTrue(variableOne == dataBuffer.getVariable("variableOne"));

      if (SHOW_GUI)
      {
         StandardSimulationGUI standardSimulationGUI = scs.getStandardSimulationGUI();
         CombinedVarPanel combinedVarPanel = standardSimulationGUI.getCombinedVarPanel();

         combinedVarPanel.setVisibleVarPanel("root.RobotsRootRegistry.RegistryOne");
         VarPanel visibleVarPanel = combinedVarPanel.getVisibleVarPanel();
         assertTrue(visibleVarPanel != null);
      }
//      sleep(100000);
   }
   
   @Ignore
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
      if (SHOW_GUI)
      {
         StandardSimulationGUI standardSimulationGUI = scs.getStandardSimulationGUI();
         CombinedVarPanel combinedVarPanel = standardSimulationGUI.getCombinedVarPanel();

         combinedVarPanel.setVisibleVarPanel("root.RobotsRootRegistry.RegistryBeforeConstructionOne");
         VarPanel visibleVarPanel = combinedVarPanel.getVisibleVarPanel();
         assertTrue(visibleVarPanel != null);
      }
//      sleep(100000);
//
      
//      visibleVarPanel.get
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
