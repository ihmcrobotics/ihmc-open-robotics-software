package us.ihmc.robotics.filters;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.robotics.math.filters.SecondOrderFilterType;
import us.ihmc.robotics.math.filters.SecondOrderFilteredYoVariable;
import us.ihmc.robotics.testing.JUnitTools;

public class SecondOrderFilteredYoVariableTest
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

      SecondOrderFilteredYoVariable filteredYoVariable = new SecondOrderFilteredYoVariable("lowPass", registry, dt, naturalFrequencyInHz, dampingRatio,
            SecondOrderFilterType.LOW_PASS);

      double[] b = new double[3];
      double[] a = new double[3];
      filteredYoVariable.getFilterCoefficients(b, a);
      JUnitTools.assertDoubleArrayEquals(b, bAssert, 1e-8);
      JUnitTools.assertDoubleArrayEquals(a, aAssert, 1e-8);
   }

   @Test
   public void testNotchFilterCoefficients()
   {
      double dt = 0.001;
      double dampingRatio = 1.0;
      double naturalFrequencyInHz = 10.0;
      double[] bAssert = {4003947.8417604356, -7992104.316479129, 4003947.8417604356};
      double[] aAssert = {4255275.254047619, -7992104.316479129, 3752620.429473252};

      SecondOrderFilteredYoVariable filteredYoVariable = new SecondOrderFilteredYoVariable("notch", registry, dt, naturalFrequencyInHz, dampingRatio,
            SecondOrderFilterType.NOTCH);

      double[] b = new double[3];
      double[] a = new double[3];
      filteredYoVariable.getFilterCoefficients(b, a);
      JUnitTools.assertDoubleArrayEquals(b, bAssert, 1e-8);
      JUnitTools.assertDoubleArrayEquals(a, aAssert, 1e-8);
   }

   @Test
   public void testHighPassFilterCoefficients()
   {
      double dt = 0.001;
      double dampingRatio = 1.0;
      double naturalFrequencyInHz = 10.0;
      double[] bAssert = {4000000.0, -8000000.0, 4000000.0};
      double[] aAssert = {4255275.254047619, -7992104.316479129, 3752620.429473252};

      SecondOrderFilteredYoVariable filteredYoVariable = new SecondOrderFilteredYoVariable("highPass", registry, dt, naturalFrequencyInHz, dampingRatio,
            SecondOrderFilterType.HIGH_PASS);

      double[] b = new double[3];
      double[] a = new double[3];
      filteredYoVariable.getFilterCoefficients(b, a);
      JUnitTools.assertDoubleArrayEquals(b, bAssert, 1e-8);
      JUnitTools.assertDoubleArrayEquals(a, aAssert, 1e-8);
   }
}
