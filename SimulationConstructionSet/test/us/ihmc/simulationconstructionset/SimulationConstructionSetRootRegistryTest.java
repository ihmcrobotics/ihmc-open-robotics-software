package us.ihmc.simulationconstructionset;

import static org.junit.Assert.assertTrue;

import org.junit.Test;

import us.ihmc.simulationconstructionset.gui.CombinedVarPanel;
import us.ihmc.simulationconstructionset.gui.StandardSimulationGUI;
import us.ihmc.simulationconstructionset.gui.VarPanel;
import us.ihmc.tools.testing.TestPlanTarget;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestClass;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;

@DeployableTestClass(targets={TestPlanTarget.UI})
public class SimulationConstructionSetRootRegistryTest
{
   private static final boolean SHOW_GUI = false;

	@DeployableTestMethod(estimatedDuration = 1.1)
	@Test(timeout = 30000)
   public void testRootRegistryNothingFancy()
   {
      Robot robot = new Robot("RobotsRootRegistry");

      YoVariableRegistry registryOne = new YoVariableRegistry("RegistryOne");
      robot.getRobotsYoVariableRegistry().addChild(registryOne);
      DoubleYoVariable variableOne = new DoubleYoVariable("variableOne", registryOne);
      
      SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters();
      parameters.setCreateGUI(SHOW_GUI);
      SimulationConstructionSet scs = new SimulationConstructionSet(robot, parameters);
      sleep(1000);
      scs.startOnAThread();
      
      YoVariableRegistry rootRegistry = scs.getRootRegistry();
      DataBuffer dataBuffer = scs.getDataBuffer();

      assertTrue(variableOne == rootRegistry.getVariable("variableOne"));
      assertTrue(variableOne == dataBuffer.getVariable("variableOne"));

      if (SHOW_GUI)
      {
         StandardSimulationGUI standardSimulationGUI = scs.getStandardSimulationGUI();
         CombinedVarPanel combinedVarPanel = standardSimulationGUI.getCombinedVarPanel();

         // This also fails when the Search Panel doesn't come up...
         sleep(2000);  //+++JEP: Not sure why need this sleep, but it fails if we don't...

         combinedVarPanel.setVisibleVarPanel("root.RobotsRootRegistry.RegistryOne");
         VarPanel visibleVarPanel = combinedVarPanel.getVisibleVarPanel();
        
         assertTrue(visibleVarPanel != null);
         assertTrue(variableOne == visibleVarPanel.getYoVariable("variableOne"));
      }
      
      scs.closeAndDispose();
   }

	@DeployableTestMethod(estimatedDuration = 0.0)
	@Test(timeout = 30000) 
   public void testVarGroups()
   {
      Robot robot = new Robot("testVarGroups");
      
      YoVariableRegistry registryOne = new YoVariableRegistry("registryOne");
      YoVariableRegistry registryTwo = new YoVariableRegistry("registryTwo");

      DoubleYoVariable variableOneA = new DoubleYoVariable("variableOneA", registryOne);
      DoubleYoVariable variableOneB = new DoubleYoVariable("variableOneB", registryOne);
      
      DoubleYoVariable variableTwoA = new DoubleYoVariable("variableTwoA", registryTwo);
      DoubleYoVariable variableTwoB = new DoubleYoVariable("variableTwoB", registryTwo);
      
      robot.getRobotsYoVariableRegistry().addChild(registryOne);
      robot.getRobotsYoVariableRegistry().addChild(registryTwo);
      
      SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters();
      parameters.setCreateGUI(SHOW_GUI);
      SimulationConstructionSet scs = new SimulationConstructionSet(robot, parameters);
      scs.setupVarGroup("VarGroupToTest", new String[]{"variableOneA", "variableTwoB"});
      
      scs.startOnAThread();

      if (SHOW_GUI)
      {
         StandardSimulationGUI standardSimulationGUI = scs.getStandardSimulationGUI();
         CombinedVarPanel combinedVarPanel = standardSimulationGUI.getCombinedVarPanel();

         sleep(2000);  //+++JEP: Not sure why need this sleep, but it fails if we don't...
         // This also fails when the Search Panel doesn't come up...
         
         standardSimulationGUI.selectVarGroup("VarGroupToTest");
         VarPanel visibleVarPanel = combinedVarPanel.getVisibleVarPanel();
         assertTrue(visibleVarPanel.getName().equals("VarGroupToTest"));
         assertTrue(variableOneA == visibleVarPanel.getYoVariable("variableOneA"));
         assertTrue(variableTwoB == visibleVarPanel.getYoVariable("variableTwoB"));
         assertTrue(null == visibleVarPanel.getYoVariable("variableOneB"));
         assertTrue(null == visibleVarPanel.getYoVariable("variableTwoA"));
         
//         sleepForever();
      }
      
      scs.closeAndDispose();
   }

	@DeployableTestMethod(estimatedDuration = 0.1)
	@Test(timeout = 30000)
   public void testRootRegistryAddYoVariablesAfterConstruction()
   {
      Robot robot = new Robot("TestAfterConstruction");
      
      YoVariableRegistry registryBeforeConstructionOne = new YoVariableRegistry("RegistryBeforeConstructionOne");
      robot.getRobotsYoVariableRegistry().addChild(registryBeforeConstructionOne);
      DoubleYoVariable variableBeforeConstructionOne = new DoubleYoVariable("variableBeforeConstructionOne", registryBeforeConstructionOne);
      
      YoVariableRegistry registryBeforeConstructionOneOne = new YoVariableRegistry("RegistryBeforeConstructionOneOne");
      registryBeforeConstructionOne.addChild(registryBeforeConstructionOneOne);
      DoubleYoVariable variableBeforeConstructionOneOne = new DoubleYoVariable("variableBeforeConstructionOneOne", registryBeforeConstructionOneOne);
      
      SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters();
      parameters.setCreateGUI(SHOW_GUI);
      SimulationConstructionSet scs = new SimulationConstructionSet(robot, parameters);
      
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
  
         sleep(2000);  //+++JEP: Not sure why need this sleep, but it fails if we don't...

         CombinedVarPanel combinedVarPanel = standardSimulationGUI.getCombinedVarPanel();
         combinedVarPanel.setVisibleVarPanel("root.TestAfterConstruction.RegistryBeforeConstructionOne");
//         sleep(2000);
         
         VarPanel visibleVarPanel = combinedVarPanel.getVisibleVarPanel();
         System.out.println("visibleVarPanel = " + visibleVarPanel.getName());
         assertTrue(visibleVarPanel.getName().equals("RegistryBeforeConstructionOne"));
         assertTrue(variableBeforeConstructionOne == visibleVarPanel.getYoVariable("variableBeforeConstructionOne"));
         assertTrue(variableAfterConstructionZero == visibleVarPanel.getYoVariable("variableAfterConstructionZero"));

         combinedVarPanel.setVisibleVarPanel("root.TestAfterConstruction.RegistryBeforeConstructionOne.RegistryBeforeConstructionOneOne");
         visibleVarPanel = combinedVarPanel.getVisibleVarPanel();
         assertTrue(visibleVarPanel.getName().equals("RegistryBeforeConstructionOneOne"));
         assertTrue(variableBeforeConstructionOneOne == visibleVarPanel.getYoVariable("variableBeforeConstructionOneOne"));
         
         combinedVarPanel.setVisibleVarPanel("root.RegistryAfterConstructionOne");         
         visibleVarPanel = combinedVarPanel.getVisibleVarPanel();
         assertTrue(visibleVarPanel.getName().equals("RegistryAfterConstructionOne"));

         assertTrue(variableAfterConstructionOne == visibleVarPanel.getYoVariable("variableAfterConstructionOne"));
         assertTrue(variableAfterConstructionTwo == visibleVarPanel.getYoVariable("variableAfterConstructionTwo"));
         assertTrue(variableAfterThreadZero == visibleVarPanel.getYoVariable("variableAfterThreadZero"));

         combinedVarPanel.setVisibleVarPanel("root.RegistryAfterConstructionOne.RegistryAfterThreadOne");
         visibleVarPanel = combinedVarPanel.getVisibleVarPanel();
         assertTrue(visibleVarPanel.getName().equals("RegistryAfterThreadOne"));

         assertTrue(variableAfterThreadOne == visibleVarPanel.getYoVariable("variableAfterThreadOne"));
         assertTrue(variableAfterThreadTwo == visibleVarPanel.getYoVariable("variableAfterThreadTwo"));
         
         sleepForever();
      }
      
      scs.closeAndDispose();
   }

   private static void sleepForever()
   {
      while(true)
      {
         sleep(10000);
      }
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
