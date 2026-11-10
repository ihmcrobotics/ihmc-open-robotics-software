package us.ihmc.stateEstimation.consistency;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class RollingANISTest
{
   private static final double DT = 0.001;

   private static RollingANIS create(double window)
   {
      YoRegistry registry = new YoRegistry("test");
      YoDouble windowVariable = new YoDouble("window", registry);
      windowVariable.set(window);
      return new RollingANIS("anis", windowVariable, registry);
   }

   @Test
   public void testNaNUntilFirstSampleThenExact()
   {
      RollingANIS anis = create(5.0);
      assertTrue(Double.isNaN(anis.getValue()));
      anis.commit(DT); // nothing added
      assertTrue(Double.isNaN(anis.getValue()));
      anis.update(6.0, 3.0, DT); // bias-corrected: the first sample is the average, not 1/5000 of it
      assertEquals(2.0, anis.getValue(), 1.0e-12);
   }

   @Test
   public void testConstantSequence()
   {
      RollingANIS anis = create(5.0);
      for (int i = 0; i < 1000; i++)
         anis.update(4.0, 2.0, DT);
      assertEquals(2.0, anis.getValue(), 1.0e-12);
   }

   @Test
   public void testChiSquaredSequenceConvergesToOne()
   {
      // NIS ~ chi^2_3 (sum of three squared unit normals): ANIS/d -> 1 for a consistent channel.
      RollingANIS anis = create(5.0);
      Random random = new Random(42);
      for (int i = 0; i < 50000; i++)
      {
         double nis = 0.0;
         for (int j = 0; j < 3; j++)
         {
            double g = random.nextGaussian();
            nis += g * g;
         }
         anis.update(nis, 3, DT);
      }
      // Window 5 s at 1 kHz ~ 2 * 5000 effective samples of chi^2_3/3 (var 2/3): std ~ 0.008.
      assertEquals(1.0, anis.getValue(), 0.05);
   }

   @Test
   public void testStepResponseHasWindowTimeConstant()
   {
      RollingANIS anis = create(1.0);
      for (int i = 0; i < 20000; i++)
         anis.update(1.0, 1.0, DT);
      int ticksPerWindow = (int) Math.round(1.0 / DT);
      for (int i = 0; i < ticksPerWindow; i++)
         anis.update(11.0, 1.0, DT);
      // After one time constant the step has covered 1 - 1/e of the way from 1 to 11.
      assertEquals(1.0 + 10.0 * (1.0 - Math.exp(-1.0)), anis.getValue(), 1.0e-3);
   }

   @Test
   public void testNaNAndNonAppliedTicksDoNotPoisonOrDecay()
   {
      RollingANIS anis = create(5.0);
      for (int i = 0; i < 1000; i++)
         anis.update(2.0, 1.0, DT);
      double before = anis.getValue();
      for (int i = 0; i < 10000; i++)
      {
         anis.update(Double.NaN, 3.0, DT);
         anis.update(Double.POSITIVE_INFINITY, 3.0, DT);
         anis.update(5.0, 0.0, DT); // zero dimension
         anis.commit(DT); // a tick on which the channel did not run
      }
      assertEquals(before, anis.getValue(), 0.0);
   }

   @Test
   public void testPooledTickIsSumOverSum()
   {
      RollingANIS anis = create(5.0);
      anis.add(3.0, 3.0); // left foot NIS/d = 1
      anis.add(Double.NaN, 3.0); // a foot whose update was not applied
      anis.add(27.0, 3.0); // right foot NIS/d = 9
      anis.commit(DT);
      assertEquals(5.0, anis.getValue(), 1.0e-12);
   }

   @Test
   public void testDisabledWindowShowsLatestSample()
   {
      RollingANIS anis = create(0.0);
      anis.update(10.0, 1.0, DT);
      anis.update(4.0, 2.0, DT);
      assertEquals(2.0, anis.getValue(), 1.0e-12);
   }

   @Test
   public void testReset()
   {
      RollingANIS anis = create(5.0);
      anis.update(10.0, 1.0, DT);
      anis.reset();
      assertTrue(Double.isNaN(anis.getValue()));
      anis.update(3.0, 1.0, DT);
      assertEquals(3.0, anis.getValue(), 1.0e-12);
   }
}
