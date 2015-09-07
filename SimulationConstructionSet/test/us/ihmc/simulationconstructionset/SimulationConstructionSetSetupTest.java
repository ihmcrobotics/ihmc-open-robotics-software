package us.ihmc.simulationconstructionset;

import javax.swing.JWindow;

import org.junit.Test;

import us.ihmc.simulationconstructionset.gui.SplashPanel;
import us.ihmc.tools.agileTesting.BambooAnnotations.BambooPlan;
import us.ihmc.tools.agileTesting.BambooAnnotations.EstimatedDuration;
import us.ihmc.tools.agileTesting.BambooPlanType;

@BambooPlan(planType = {BambooPlanType.UI})
public class SimulationConstructionSetSetupTest
{
   private static final int pauseTimeForGUIs = 5000;

	@EstimatedDuration(duration = 5.0)
	@Test(timeout = 30000)
   public void testSplashScreen()
   {
      SplashPanel splashPanel = new SplashPanel();
      JWindow window = splashPanel.showSplashScreen();

      sleep(pauseTimeForGUIs);
      window.dispose();
   }

	@EstimatedDuration(duration = 5.2)
	@Test(timeout = 30000)
   public void testSimulationConstructionSetWithoutARobot()
   {
      SimulationConstructionSet scs = new SimulationConstructionSet();
      Thread thread = new Thread(scs);
      thread.start();

      sleep(pauseTimeForGUIs);
      scs.closeAndDispose();
   }

	@EstimatedDuration(duration = 5.6)
	@Test(timeout = 30000)
   public void testSimulationConstructionSetWithARobot()
   {
      Robot robot = new Robot("NullRobot");
      SimulationConstructionSet scs = new SimulationConstructionSet(robot);
      Thread thread = new Thread(scs);
      thread.start();

      sleep(pauseTimeForGUIs);
      scs.closeAndDispose();
   }

   private void sleep(long sleepMillis)
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
