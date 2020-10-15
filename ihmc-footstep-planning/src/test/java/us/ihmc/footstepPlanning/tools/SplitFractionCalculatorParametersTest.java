package us.ihmc.footstepPlanning.tools;

import controller_msgs.msg.dds.FootstepPlannerParametersPacket;
import controller_msgs.msg.dds.SplitFractionCalculatorParametersPacket;
import org.junit.jupiter.api.Test;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.footstepPlanning.icp.DefaultSplitFractionCalculatorParameters;
import us.ihmc.footstepPlanning.icp.SplitFractionCalculatorParameterKeys;
import us.ihmc.footstepPlanning.icp.SplitFractionCalculatorParametersBasics;
import us.ihmc.footstepPlanning.icp.SplitFractionCalculatorParametersReadOnly;
import us.ihmc.tools.property.BooleanStoredPropertyKey;
import us.ihmc.tools.property.DoubleStoredPropertyKey;
import us.ihmc.tools.property.IntegerStoredPropertyKey;
import us.ihmc.tools.property.StoredPropertyKey;

import java.util.Random;

import static us.ihmc.robotics.Assert.assertEquals;
import static us.ihmc.robotics.Assert.assertNotEquals;

public class SplitFractionCalculatorParametersTest
{
   private static final int iters = 1000;
   private static final double epsilon = 1e-8;

   @Test
   public void testVariableCopying()
   {
      Random random = new Random(1738L);

      SplitFractionCalculatorParametersBasics parametersToSet = new DefaultSplitFractionCalculatorParameters();

      for (int iter = 0; iter < iters; iter++)
      {
         SplitFractionCalculatorParametersReadOnly randomParameters = getRandomParameters(random);
         SplitFractionCalculatorParametersPacket packet = randomParameters.getAsPacket();
         parametersToSet.set(packet);

         assertParametersEqual(randomParameters, parametersToSet, epsilon);
      }
   }

   private static void assertParametersEqual(SplitFractionCalculatorParametersReadOnly parametersA, SplitFractionCalculatorParametersReadOnly parametersB, double epsilon)
   {
      for (StoredPropertyKey<?> key : SplitFractionCalculatorParameterKeys.keys.keys())
      {
         String failureMessage = key.getTitleCasedName() + " has the wrong value.";
         if (key instanceof DoubleStoredPropertyKey)
         {
            DoubleStoredPropertyKey doubleKey = (DoubleStoredPropertyKey) key;
            assertEquals(failureMessage, parametersA.get(doubleKey), parametersB.get(doubleKey), epsilon);
            assertNotEquals(FootstepPlannerParametersPacket.DEFAULT_NO_VALUE, parametersB.get(doubleKey), epsilon);
         }
         else if (key instanceof IntegerStoredPropertyKey)
         {
            IntegerStoredPropertyKey integerKey = (IntegerStoredPropertyKey) key;
            assertEquals(failureMessage, parametersA.get(integerKey), parametersB.get(integerKey));
         }
         else if (key instanceof BooleanStoredPropertyKey)
         {
            BooleanStoredPropertyKey booleanKey = (BooleanStoredPropertyKey) key;
            assertEquals(failureMessage, parametersA.get(booleanKey), parametersB.get(booleanKey));
         }
      }
   }

   private static SplitFractionCalculatorParametersReadOnly getRandomParameters(Random random)
   {
      SplitFractionCalculatorParametersBasics parameters = new DefaultSplitFractionCalculatorParameters();

      for (StoredPropertyKey<?> key : SplitFractionCalculatorParameterKeys.keys.keys())
      {
         if (key instanceof DoubleStoredPropertyKey)
         {
            double randomValue = RandomNumbers.nextDouble(random, -10.0, 10.0);
            parameters.set(((DoubleStoredPropertyKey) key), randomValue);
         }
         else if (key instanceof IntegerStoredPropertyKey)
         {
            int randomValue = RandomNumbers.nextInt(random, 1, 10);
            parameters.set(((IntegerStoredPropertyKey) key), randomValue);
         }
         else if (key instanceof BooleanStoredPropertyKey)
         {
            boolean randomValue = RandomNumbers.nextBoolean(random, 0.5);
            parameters.set(((BooleanStoredPropertyKey) key), randomValue);
         }
      }

      return parameters;
   }
}
