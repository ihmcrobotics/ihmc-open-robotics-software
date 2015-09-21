package us.ihmc.simulationconstructionset;

import javax.swing.*;

import org.junit.Test;

import us.ihmc.simulationconstructionset.gui.SplashPanel;
import us.ihmc.tools.testing.TestPlanTarget;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestClass;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;

import java.lang.reflect.InvocationTargetException;

@DeployableTestClass(targets = {TestPlanTarget.UI})
public class SimulationConstructionSetSetupTest
{
   private static final int pauseTimeForGUIs = 5000;

	@DeployableTestMethod(estimatedDuration = 5.0)
	@Test(timeout = 30000)
   public void testSplashScreen() throws Exception
    {
      SwingUtilities.invokeAndWait(new Runnable()
      {
         @Override
         public void run()
         {
            SplashPanel splashPanel = new SplashPanel();
            JWindow window = splashPanel.showSplashScreen();

            sleep(pauseTimeForGUIs);
            window.dispose();
         }
      });
   }

	@DeployableTestMethod(estimatedDuration = 5.2)
	@Test(timeout = 30000)
   public void testSimulationConstructionSetWithoutARobot()
   {
      SimulationConstructionSet scs = new SimulationConstructionSet();
      Thread thread = new Thread(scs);
      thread.start();

      sleep(pauseTimeForGUIs);
      scs.closeAndDispose();
   }

	@DeployableTestMethod(estimatedDuration = 5.6)
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
