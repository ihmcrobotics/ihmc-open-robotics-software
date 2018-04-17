package us.ihmc.robotics.filters;

import org.junit.Test;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.robotics.math.filters.SecondOrderFilterType;
import us.ihmc.robotics.math.filters.SecondOrderFilteredYoVariable;
import us.ihmc.robotics.testing.JUnitTools;

@ContinuousIntegrationPlan(categories = {IntegrationCategory.FAST})
public class SecondOrderFilteredYoVariableTest
{
   YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
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

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
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

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
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
