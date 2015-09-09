package us.ihmc.simulationconstructionset;

import static org.junit.Assert.assertEquals;

import org.junit.Ignore;
import org.junit.Test;

import us.ihmc.tools.testing.BambooPlanType;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestClass;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;
import us.ihmc.tools.thread.ThreadTools;
import us.ihmc.yoUtilities.dataStructure.listener.RewoundListener;

@DeployableTestClass(planType = BambooPlanType.Exclude)
public class SimulationRewoundListenerTest
{
   /**
    * Hang forever
    */
   @Ignore
	@DeployableTestMethod(duration = 30.0, quarantined = true)
	@Test(timeout=300000)
   public void testSimulationRewoundListener()
   {
      boolean showGUI = false;

      SimpleSimulationRewoundListener simulationRewoundListener = new SimpleSimulationRewoundListener();

      Robot robot = new Robot("Test");
      SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters();
      parameters.setCreateGUI(showGUI);
      SimulationConstructionSet scs = new SimulationConstructionSet(robot, parameters);
      scs.setDT(0.001, 10);

      scs.attachSimulationRewoundListener(simulationRewoundListener);
      scs.startOnAThread();

      assertEquals(0, simulationRewoundListener.getCount());
      scs.simulate(1.0);
      while(scs.isSimulating())
      {
         ThreadTools.sleep(10);
      }
      assertEquals(1, simulationRewoundListener.getCount());
      assertEquals(100, scs.getIndex());

      scs.gotoInPointNow();
      assertEquals(2, simulationRewoundListener.getCount());
      assertEquals(0, scs.getIndex());

      ThreadTools.sleep(100);
      scs.tick(1);
      assertEquals(3, simulationRewoundListener.getCount());
      assertEquals(1, scs.getIndex());

      scs.tick(5);
      assertEquals(4, simulationRewoundListener.getCount());
      assertEquals(6, scs.getIndex());

      scs.tick(-1);
      assertEquals(5, simulationRewoundListener.getCount());
      assertEquals(5, scs.getIndex());

      scs.tickAndUpdate();
      assertEquals(5, simulationRewoundListener.getCount());
      assertEquals(6, scs.getIndex());

      scs.gotoOutPointNow();
      assertEquals(6, simulationRewoundListener.getCount());
      assertEquals(6, scs.getIndex());

      scs.play();
      ThreadTools.sleep(1000);
      assertEquals(6, simulationRewoundListener.getCount());

      scs.closeAndDispose();
   }
   
   private class SimpleSimulationRewoundListener implements RewoundListener
   {
      private int count = 0;
      public void wasRewound()
      {
//         System.out.println(count + ": Sim was rewound");
         count++;
      }
      
      public int getCount()
      {
         return count;
      }
   };
}
