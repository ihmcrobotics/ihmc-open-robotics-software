package us.ihmc.robotics.math.filters;

import org.junit.jupiter.api.Test;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.Random;

import static org.junit.jupiter.api.Assertions.*;

/**
 * @author Doug Stephen <a href="mailto:dstephen@ihmc.us">(dstephen@ihmc.us)</a>
 */
public class DeltaLimitedYoVariableTest
{
   private static final int RANDOM_LOWER_BOUND = 10;
   private static final int RANDOM_UPPER_BOUND = 30000;
   private YoRegistry registry;
   private DeltaLimitedYoVariable variable;

   @Test
   public void testReferenceAndInputBothNegativeNoOvershootInputGreaterThanReference()
   {
      Random random = new Random(1976L);
      registry = new YoRegistry("registry");
      variable = new DeltaLimitedYoVariable("testVar", registry, 0.0);

      for (int i = 0; i < 60000; i++)
      {
         double reference = RandomNumbers.nextInt(random, RANDOM_LOWER_BOUND, RANDOM_UPPER_BOUND) * -1.0;
         double input = reference / 2.0;
         double delta = Math.abs(input - reference);

         variable.setMaxDelta(delta);
         variable.updateOutput(reference, input);

         assertTrue(input < 0.0);
         assertTrue(reference < 0.0);
         assertTrue(input > reference);
         assertFalse(variable.isLimitingActive());
         assertEquals(input, variable.getDoubleValue(), 1e-8);
      }
   }

   @Test
   public void testReferenceAndInputBothNegativeNoOvershootReferenceGreaterThanInput()
   {
      Random random = new Random(1976L);
      registry = new YoRegistry("registry");
      variable = new DeltaLimitedYoVariable("testVar", registry, 0.0);

      for (int i = 0; i < 60000; i++)
      {
         double reference = RandomNumbers.nextInt(random, RANDOM_LOWER_BOUND, RANDOM_UPPER_BOUND) * -1.0;
         double input = reference * 2;
         double delta = Math.abs(input - reference);

         variable.setMaxDelta(delta);
         variable.updateOutput(reference, input);

         assertTrue(input < 0.0);
         assertTrue(reference < 0.0);
         assertTrue(input < reference);
         assertFalse(variable.isLimitingActive());
         assertEquals(input, variable.getDoubleValue(), 1e-8);
      }
   }

   @Test
   public void testReferenceAndInputBothPositiveNoOvershootReferenceGreaterThanInput()
   {
      Random random = new Random(1976L);
      registry = new YoRegistry("registry");
      variable = new DeltaLimitedYoVariable("testVar", registry, 0.0);

      for (int i = 0; i < 60000; i++)
      {
         double reference = RandomNumbers.nextInt(random, RANDOM_LOWER_BOUND, RANDOM_UPPER_BOUND);
         double input = reference / 2.0;
         double delta = Math.abs(input - reference);

         variable.setMaxDelta(delta);
         variable.updateOutput(reference, input);

         assertTrue(input > 0.0);
         assertTrue(reference > 0.0);
         assertTrue(input < reference);
         assertFalse(variable.isLimitingActive());
         assertEquals(input, variable.getDoubleValue(), 1e-8);
      }
   }

   @Test
   public void testReferenceAndInputBothPositiveNoOvershootInputGreaterThanReference()
   {
      Random random = new Random(1976L);
      registry = new YoRegistry("registry");
      variable = new DeltaLimitedYoVariable("testVar", registry, 0.0);

      for (int i = 0; i < 60000; i++)
      {
         double reference = RandomNumbers.nextInt(random, RANDOM_LOWER_BOUND, RANDOM_UPPER_BOUND);
         double input = reference * 2;
         double delta = Math.abs(input - reference);

         variable.setMaxDelta(delta);
         variable.updateOutput(reference, input);

         assertTrue(input > 0.0);
         assertTrue(reference > 0.0);
         assertTrue(input > reference);
         assertFalse(variable.isLimitingActive());
         assertEquals(input, variable.getDoubleValue(), 1e-8);
      }
   }

   @Test
   public void testPositiveReferenceNegativeInputNoOvershoot()
   {
      Random random = new Random(1976L);
      registry = new YoRegistry("registry");
      variable = new DeltaLimitedYoVariable("testVar", registry, 0.0);

      for (int i = 0; i < 60000; i++)
      {
         double reference = RandomNumbers.nextInt(random, RANDOM_LOWER_BOUND, RANDOM_UPPER_BOUND);
         double input = reference * -0.5;
         double delta = Math.abs(input - reference);

         variable.setMaxDelta(delta);
         variable.updateOutput(reference, input);

         assertTrue(input < 0.0);
         assertTrue(reference > 0.0);
         assertTrue(input < reference);
         assertFalse(variable.isLimitingActive());
         assertEquals(input, variable.getDoubleValue(), 1e-8);
      }
   }

   @Test
   public void testNegativeReferencePositiveInputNoOvershoot()
   {
      Random random = new Random(1976L);
      registry = new YoRegistry("registry");
      variable = new DeltaLimitedYoVariable("testVar", registry, 0.0);

      for (int i = 0; i < 60000; i++)
      {
         double reference = RandomNumbers.nextInt(random, RANDOM_LOWER_BOUND, RANDOM_UPPER_BOUND) * -1.0;
         double input = reference * -0.5;
         double delta = Math.abs(input - reference);

         variable.setMaxDelta(delta);
         variable.updateOutput(reference, input);

         assertTrue(input > 0.0);
         assertTrue(reference < 0.0);
         assertTrue(input > reference);
         assertFalse(variable.isLimitingActive());
         assertEquals(input, variable.getDoubleValue(), 1e-8);
      }
   }

   @Test
   public void testReferenceAndInputBothNegativeWithOvershootInputGreaterThanReference()
   {
      Random random = new Random(1976L);
      registry = new YoRegistry("registry");
      variable = new DeltaLimitedYoVariable("testVar", registry, 0.0);

      for (int i = 0; i < 60000; i++)
      {
         double reference = RandomNumbers.nextInt(random, RANDOM_LOWER_BOUND, RANDOM_UPPER_BOUND) * -1.0;
         double input = reference / 2.0;
         double delta = Math.abs(input - reference) / 2.0;

         variable.setMaxDelta(delta);
         variable.updateOutput(reference, input);

         double overshoot = Math.abs(input - reference) - delta;

         double expectedClip = input - overshoot;

         assertTrue(input < 0.0);
         assertTrue(reference < 0.0);
         assertTrue(input > reference);
         assertTrue(variable.isLimitingActive());
         assertEquals(expectedClip,
                      variable.getDoubleValue(),
                      1e-8,
                      "Variable not clipped correctly\nReference: " + reference + "\nInput: " + input + "\nMagnitude of requested delta: " + Math.abs(
                            input - reference) + "\nMax Allowed Delta: " + delta + "\nOvershoot: " + overshoot);
      }
   }

   @Test
   public void testReferenceAndInputBothNegativeWithOvershootReferenceGreaterThanInput()
   {
      Random random = new Random(1976L);
      registry = new YoRegistry("registry");
      variable = new DeltaLimitedYoVariable("testVar", registry, 0.0);

      for (int i = 0; i < 60000; i++)
      {
         double reference = RandomNumbers.nextInt(random, RANDOM_LOWER_BOUND, RANDOM_UPPER_BOUND) * -1.0;
         double input = reference * 2;
         double delta = Math.abs(input - reference) / 2.0;

         variable.setMaxDelta(delta);
         variable.updateOutput(reference, input);

         double overshoot = Math.abs(input - reference) - delta;

         double expectedClip = input + overshoot;

         assertTrue(input < 0.0);
         assertTrue(reference < 0.0);
         assertTrue(input < reference);
         assertTrue(variable.isLimitingActive());
         assertEquals(expectedClip,
                      variable.getDoubleValue(),
                      1e-8,
                      "Variable not clipped correctly\nReference: " + reference + "\nInput: " + input + "\nMagnitude of requested delta: " + Math.abs(
                            input - reference) + "\nMax Allowed Delta: " + delta + "\nOvershoot: " + overshoot);
      }
   }

   @Test
   public void testReferenceAndInputBothPositiveWithOvershootInputGreaterThanReference()
   {
      Random random = new Random(1976L);
      registry = new YoRegistry("registry");
      variable = new DeltaLimitedYoVariable("testVar", registry, 0.0);

      for (int i = 0; i < 60000; i++)
      {
         double reference = RandomNumbers.nextInt(random, RANDOM_LOWER_BOUND, RANDOM_UPPER_BOUND);
         double input = reference * 2;
         double delta = Math.abs(input - reference) / 2.0;

         variable.setMaxDelta(delta);
         variable.updateOutput(reference, input);

         double overshoot = Math.abs(input - reference) - delta;

         double expectedClip = input - overshoot;

         assertTrue(input > 0.0);
         assertTrue(reference > 0.0);
         assertTrue(input > reference);
         assertTrue(variable.isLimitingActive());
         assertEquals(expectedClip,
                      variable.getDoubleValue(),
                      1e-8,
                      "Variable not clipped correctly\nReference: " + reference + "\nInput: " + input + "\nMagnitude of requested delta: " + Math.abs(
                            input - reference) + "\nMax Allowed Delta: " + delta + "\nOvershoot: " + overshoot);
      }
   }

   @Test
   public void testReferenceAndInputBothPositiveWithOvershootReferenceGreaterThanInput()
   {
      Random random = new Random(1976L);
      registry = new YoRegistry("registry");
      variable = new DeltaLimitedYoVariable("testVar", registry, 0.0);

      for (int i = 0; i < 60000; i++)
      {
         double reference = RandomNumbers.nextInt(random, RANDOM_LOWER_BOUND, RANDOM_UPPER_BOUND);
         double input = reference / 2.0;
         double delta = Math.abs(input - reference) / 2.0;

         variable.setMaxDelta(delta);
         variable.updateOutput(reference, input);

         double overshoot = Math.abs(input - reference) - delta;

         double expectedClip = input + overshoot;

         assertTrue(input > 0.0);
         assertTrue(reference > 0.0);
         assertTrue(input < reference);
         assertTrue(variable.isLimitingActive());
         assertEquals(expectedClip,
                      variable.getDoubleValue(),
                      1e-8,
                      "Variable not clipped correctly\nReference: " + reference + "\nInput: " + input + "\nMagnitude of requested delta: " + Math.abs(
                            input - reference) + "\nMax Allowed Delta: " + delta + "\nOvershoot: " + overshoot);
      }
   }

   @Test
   public void testPositiveReferenceNegativeInputWithOvershoot()
   {
      Random random = new Random(1976L);
      registry = new YoRegistry("registry");
      variable = new DeltaLimitedYoVariable("testVar", registry, 0.0);

      for (int i = 0; i < 60000; i++)
      {
         double reference = RandomNumbers.nextInt(random, RANDOM_LOWER_BOUND, RANDOM_UPPER_BOUND);
         double input = reference * -0.5;
         double delta = Math.abs(input - reference) / 2.0;

         variable.setMaxDelta(delta);
         variable.updateOutput(reference, input);

         double overshoot = Math.abs(input - reference) - delta;

         double expectedClip = input + overshoot;

         assertTrue(input < 0.0);
         assertTrue(reference > 0.0);
         assertTrue(input < reference);
         assertTrue(variable.isLimitingActive());
         assertEquals(expectedClip,
                      variable.getDoubleValue(),
                      1e-8,
                      "Variable not clipped correctly\nReference: " + reference + "\nInput: " + input + "\nMagnitude of requested delta: " + Math.abs(
                            input - reference) + "\nMax Allowed Delta: " + delta + "\nOvershoot: " + overshoot);
      }
   }

   @Test
   public void testNegativeReferencePositiveInputWithOvershoot()
   {
      Random random = new Random(1976L);
      registry = new YoRegistry("registry");
      variable = new DeltaLimitedYoVariable("testVar", registry, 0.0);

      for (int i = 0; i < 60000; i++)
      {
         double reference = RandomNumbers.nextInt(random, RANDOM_LOWER_BOUND, RANDOM_UPPER_BOUND) * -1.0;
         double input = reference * -0.5;
         double delta = Math.abs(input - reference) / 2.0;

         variable.setMaxDelta(delta);
         variable.updateOutput(reference, input);

         double overshoot = Math.abs(input - reference) - delta;

         double expectedClip = input - overshoot;

         assertTrue(input > 0.0);
         assertTrue(reference < 0.0);
         assertTrue(input > reference);
         assertTrue(variable.isLimitingActive());
         assertEquals(expectedClip,
                      variable.getDoubleValue(),
                      1e-8,
                      "Variable not clipped correctly\nReference: " + reference + "\nInput: " + input + "\nMagnitude of requested delta: " + Math.abs(
                            input - reference) + "\nMax Allowed Delta: " + delta + "\nOvershoot: " + overshoot);
      }
   }

   @Test
   public void testOvershootThenNoOvershoot()
   {
      Random random = new Random(1976L);
      registry = new YoRegistry("registry");
      variable = new DeltaLimitedYoVariable("testVar", registry, 0.0);

      for (int i = 0; i < 60000; i++)
      {
         double reference = RandomNumbers.nextInt(random, RANDOM_LOWER_BOUND, RANDOM_UPPER_BOUND) * -1.0;
         double input = reference * -0.5;
         double delta = Math.abs(input - reference) / 2.0;

         variable.setMaxDelta(delta);
         variable.updateOutput(reference, input);

         double overshoot = Math.abs(input - reference) - delta;

         double expectedClip = input - overshoot;

         assertTrue(input > 0.0);
         assertTrue(reference < 0.0);
         assertTrue(input > reference);
         assertTrue(variable.isLimitingActive());
         assertEquals(expectedClip,
                      variable.getDoubleValue(),
                      1e-8,
                      "Variable not clipped correctly\nReference: " + reference + "\nInput: " + input + "\nMagnitude of requested delta: " + Math.abs(
                            input - reference) + "\nMax Allowed Delta: " + delta + "\nOvershoot: " + overshoot);

         input = variable.getDoubleValue();

         variable.updateOutput(reference, input);
         double newRequestedDelta = Math.abs(input - reference);
         assertFalse(newRequestedDelta > delta);
         assertFalse(variable.isLimitingActive());
         assertEquals(expectedClip,
                      variable.getDoubleValue(),
                      1e-8,
                      "Variable not clipped correctly\nReference: " + reference + "\nInput: " + input + "\nMagnitude of requested delta: " + Math.abs(
                            input - reference) + "\nMax Allowed Delta: " + delta + "\nOvershoot: " + overshoot);
      }
   }
}