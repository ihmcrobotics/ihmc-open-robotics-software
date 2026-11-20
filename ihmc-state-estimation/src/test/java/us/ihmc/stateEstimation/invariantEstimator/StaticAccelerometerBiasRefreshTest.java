package us.ihmc.stateEstimation.invariantEstimator;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;

/** staticAccelBiasRefreshAtRest: with it on, every completed window of stillness replaces the held bias. */
public class StaticAccelerometerBiasRefreshTest
{
   private static final double DT = 0.001;
   private static final double G = 9.81;

   private static void stand(StaticAccelerometerBiasEstimator estimator, double norm, double seconds)
   {
      Vector3D f = new Vector3D(0.0, 0.0, norm);
      for (int k = 0; k < Math.round(seconds / DT); k++)
         estimator.update(f, 0.0, DT);
   }

   private static void walk(StaticAccelerometerBiasEstimator estimator, double seconds)
   {
      Vector3D f = new Vector3D(1.0, 0.0, G);
      for (int k = 0; k < Math.round(seconds / DT); k++)
         estimator.update(f, 0.5, DT);
   }

   @Test
   public void testWithoutRefreshTheStartUpValueIsHeld()
   {
      StaticAccelerometerBiasEstimator estimator = new StaticAccelerometerBiasEstimator(G, 3.0, 0.05, new YoRegistry("test"));
      stand(estimator, 9.67, 3.5);
      assertEquals(9.67 - G, estimator.getBias().getZ(), 1e-9);
      walk(estimator, 2.0);
      stand(estimator, 9.66, 10.0);
      assertEquals(9.67 - G, estimator.getBias().getZ(), 1e-9);
   }

   @Test
   public void testWithRefreshEachRestReplacesTheBias()
   {
      YoRegistry registry = new YoRegistry("test");
      StaticAccelerometerBiasEstimator estimator = new StaticAccelerometerBiasEstimator(G, 3.0, 0.05, registry);
      ((YoBoolean) registry.findVariable("staticAccelBiasRefreshAtRest")).set(true);
      stand(estimator, 9.67, 3.5);
      assertEquals(9.67 - G, estimator.getBias().getZ(), 1e-9);
      walk(estimator, 2.0);
      stand(estimator, 9.66, 2.0); // too short: no refresh yet
      assertEquals(9.67 - G, estimator.getBias().getZ(), 1e-9);
      walk(estimator, 1.0);
      stand(estimator, 9.66, 3.5);
      assertEquals(9.66 - G, estimator.getBias().getZ(), 1e-9);
      assertEquals(StaticAccelerometerBiasEstimator.State.HELD, estimator.getState());
   }
}
