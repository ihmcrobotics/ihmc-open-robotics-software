package us.ihmc.yoVariables.filters;

import org.junit.jupiter.api.Test;
import us.ihmc.yoVariables.registry.YoRegistry;

import static org.junit.jupiter.api.Assertions.*;

public class SecondOrderFilteredYoDoubleTest
{
   YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   @Test
   public void testLowPassFilterCoefficients()
   {
      double dt = 0.001;
      double dampingRatio = 1.0;
      double naturalFrequencyInHz = 10.0;
      double[] bAssert = {3947.8417604357433, 7895.6835208714865, 3947.8417604357433};
      double[] aAssert = {4255275.254047619, -7992104.316479129, 3752620.429473252};

      SecondOrderFilteredYoDouble filteredYoVariable = new SecondOrderFilteredYoDouble("lowPass", registry, dt, naturalFrequencyInHz, dampingRatio,
                                                                                       SecondOrderFilterType.LOW_PASS);

      double[] b = new double[3];
      double[] a = new double[3];
      filteredYoVariable.getFilterCoefficients(b, a);
      assertArrayEquals(b, bAssert, 1e-8);
      assertArrayEquals(a, aAssert, 1e-8);
   }

   @Test
   public void testNotchFilterCoefficients()
   {
      double dt = 0.001;
      double dampingRatio = 1.0;
      double naturalFrequencyInHz = 10.0;
      double[] bAssert = {4003947.8417604356, -7992104.316479129, 4003947.8417604356};
      double[] aAssert = {4255275.254047619, -7992104.316479129, 3752620.429473252};

      SecondOrderFilteredYoDouble filteredYoVariable = new SecondOrderFilteredYoDouble("notch", registry, dt, naturalFrequencyInHz, dampingRatio,
                                                                                       SecondOrderFilterType.NOTCH);

      double[] b = new double[3];
      double[] a = new double[3];
      filteredYoVariable.getFilterCoefficients(b, a);
      assertArrayEquals(b, bAssert, 1e-8);
      assertArrayEquals(a, aAssert, 1e-8);
   }

   @Test
   public void testHighPassFilterCoefficients()
   {
      double dt = 0.001;
      double dampingRatio = 1.0;
      double naturalFrequencyInHz = 10.0;
      double[] bAssert = {4000000.0, -8000000.0, 4000000.0};
      double[] aAssert = {4255275.254047619, -7992104.316479129, 3752620.429473252};

      SecondOrderFilteredYoDouble filteredYoVariable = new SecondOrderFilteredYoDouble("highPass", registry, dt, naturalFrequencyInHz, dampingRatio,
                                                                                       SecondOrderFilterType.HIGH_PASS);

      double[] b = new double[3];
      double[] a = new double[3];
      filteredYoVariable.getFilterCoefficients(b, a);
      assertArrayEquals(b, bAssert, 1e-8);
      assertArrayEquals(a, aAssert, 1e-8);
   }
}
