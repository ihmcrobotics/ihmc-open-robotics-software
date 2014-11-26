package us.ihmc.exampleSimulations.springflamingo;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.gui.SimulationGUITestFixture;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;

public class SpringFlamingoSimulationTest
{
   private SimulationGUITestFixture testFixture;
   private SimulationConstructionSet scs;

   @Before
   public void setUp() throws SimulationExceededMaximumTimeException
   {
      SpringFlamingoSimulation springFlamingoSimulation = new SpringFlamingoSimulation();
      scs = springFlamingoSimulation.getSimulationConstructionSet();
      
//      ThreadTools.sleep(1000L);
      testFixture = new SimulationGUITestFixture(scs);
   }

   @After
   public void tearDown()
   {
      testFixture.closeAndDispose();
      scs.closeAndDispose();
      scs = null;
      testFixture = null;
   }

   @Test
   public void testSpringFlamingoSimulationAndGUI() throws SimulationExceededMaximumTimeException
   {
//      testFixture.showWindow();
//      testFixture.clickFileMenu();
//      
//      testFixture.selectNameSpaceTab();
//      ThreadTools.sleep(1000);
      
//      testFixture.selectNewGraphWindowMenu();
      
      testFixture.selectSearchTab();
      
      testFixture.deleteSearchText();
      testFixture.enterSearchText("q_");
      
      testFixture.selectVariableAndSetValueInSearchTab("q_pitch", 1.0);
//      ThreadTools.sleep(500);
      testFixture.selectVariableInSearchTab("q_x");
//      ThreadTools.sleep(500);
      
      testFixture.clickRemoveEmptyGraphButton();
      testFixture.clickNewGraphButton();
      testFixture.middleClickInEmptyGraph();
      
      testFixture.selectVariableInSearchTab("q_z");
      testFixture.middleClickInNthGraph(1);
      testFixture.removeVariableFromNthGraph("q_z", 1);
      
//      testFixture.selectNewGraphWindowMenu();
      
//      testFixture.sel
//      testFixture.selectVariableInSearchTab("q_z");
//      ThreadTools.sleep(3000);

      
//      testFixture.clickRemoveEmptyGraphButton();
//      testFixture.clickNewGraphButton();

      
//  

//      testFixture.clickNewGraphButton();
//      testFixture.clickSimulateButton();
//
//      ThreadTools.sleep(500);
//      testFixture.clickStopButton();
//      testFixture.clickPlayMenu(); //Button();
//
//      ThreadTools.sleep(500);
//      testFixture.clickStopButton();
//
//      testFixture.clickGotoInPointButton();
//
//      ThreadTools.sleep(500);
//
//      int index = scs.getIndex();
//      int inPoint = scs.getInPoint();
//      assertEquals(index, inPoint);
//
//      testFixture.clickPlaybackPropertiesMenu();

//      
//    ThreadTools.sleep(1000);
//    ThreadTools.sleepForever();
//    window.menuItem("foo").
   }

}
