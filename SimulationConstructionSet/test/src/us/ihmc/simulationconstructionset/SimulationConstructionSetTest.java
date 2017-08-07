package us.ihmc.simulationconstructionset;

import java.awt.AWTException;

import org.junit.Assume;
import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.simulationconstructionset.examples.FallingBrickRobot;
import us.ihmc.simulationconstructionset.gui.SimulationGUITestFixture;
import us.ihmc.tools.thread.ThreadTools;

@ContinuousIntegrationPlan(categories = IntegrationCategory.MANUAL)
public class SimulationConstructionSetTest
{
   private boolean isGradleBuild()
   {
      String property = System.getProperty("bamboo.gradle");
      if (property != null && property.contains("yes"))
      {
         return true;
      }

      return false;
   }
   
	@ContinuousIntegrationTest(estimatedDuration = 0.1, categoriesOverride = IntegrationCategory.EXCLUDE)
	@Test(timeout=300000)
   public void testSimulationConstructionSetNewViewportWindowUsingGUITestFixture() throws AWTException
   {
      Assume.assumeTrue(!isGradleBuild());
      FallingBrickRobot robot = new FallingBrickRobot();

      SimulationConstructionSet scs = new SimulationConstructionSet(robot);
      scs.setDT(0.0001, 100);
      scs.setFrameMaximized();
      scs.startOnAThread();
      scs.setSimulateDuration(2.0);

      ThreadTools.sleep(2000);
      SimulationGUITestFixture testFixture = new SimulationGUITestFixture(scs);
      
      testFixture.closeAllViewportWindows();
      testFixture.selectNewViewportWindowMenu();
      
      testFixture.focusNthViewportWindow(0);

      ThreadTools.sleepForever();
      
      testFixture.closeAndDispose();
      scs.closeAndDispose();
      scs = null;
      testFixture = null;

   }

	@ContinuousIntegrationTest(estimatedDuration = 0.1, categoriesOverride = IntegrationCategory.EXCLUDE)
	@Test(timeout=300000)
   public void testSimulationConstructionSetVideoGenerationUsingGUITestFixture() throws AWTException
   {
      Assume.assumeTrue(!isGradleBuild());
      FallingBrickRobot robot = new FallingBrickRobot();

      SimulationConstructionSet scs = new SimulationConstructionSet(robot);
      scs.setDT(0.0001, 100);
      scs.setFrameMaximized();
      scs.startOnAThread();
      scs.setSimulateDuration(2.0);

      SimulationGUITestFixture testFixture = new SimulationGUITestFixture(scs);
      testFixture.clickSimulateButton();
      ThreadTools.sleep(1000);
      
      testFixture.clickMediaCaptureButton();

      testFixture.focusDialog("Export Video");
      testFixture.clickPlayButton();
      
      ThreadTools.sleepForever();

      testFixture.closeAndDispose();
      scs.closeAndDispose();
      scs = null;
      testFixture = null;
   }
}
