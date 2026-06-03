package us.ihmc.footstepPlanning.tools;

import org.junit.jupiter.api.Assertions;
import toolbox_msgs.FootstepPlannerParametersPacket;
import toolbox_msgs.SwingPlannerParametersPacket;
import org.junit.jupiter.api.Test;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.footstepPlanning.swing.DefaultSwingPlannerParameters;
import us.ihmc.footstepPlanning.swing.SwingPlannerParameterKeys;
import us.ihmc.footstepPlanning.swing.SwingPlannerParametersBasics;
import us.ihmc.footstepPlanning.swing.SwingPlannerParametersReadOnly;
import us.ihmc.tools.property.BooleanStoredPropertyKey;
import us.ihmc.tools.property.DoubleStoredPropertyKey;
import us.ihmc.tools.property.IntegerStoredPropertyKey;
import us.ihmc.tools.property.StoredPropertyKey;

import java.util.Random;

public class SwingPlannerParametersTest
{
   private static final int iterations = 10;
   private final Random random = new Random(1738L);

   @Test
   public void testVariableCopying()
   {
      SwingPlannerParametersBasics parametersToSet = new DefaultSwingPlannerParameters();

      for (int iter = 0; iter < iterations; iter++)
      {
         SwingPlannerParametersReadOnly randomParameters = getRandomParameters(random);
         SwingPlannerParametersPacket packet = randomParameters.getAsPacket();
         parametersToSet.set(packet);

         assertParametersEqual(randomParameters, parametersToSet);
      }
   }

   private static void assertParametersEqual(SwingPlannerParametersReadOnly parametersA, SwingPlannerParametersReadOnly parametersB)
   {
      for (StoredPropertyKey<?> key : SwingPlannerParameterKeys.keys.keys())
      {
         String failureMessage = key.getTitleCasedName() + " has the wrong value.";
         if (key instanceof DoubleStoredPropertyKey)
         {
            Assertions.assertEquals(parametersA.get(key), parametersB.get(key), failureMessage);
            Assertions.assertNotEquals(FootstepPlannerParametersPacket.DEFAULT_NO_VALUE, parametersB.get(key), failureMessage);
         }
         else if (key instanceof IntegerStoredPropertyKey)
         {
            Assertions.assertEquals(parametersA.get(key), parametersB.get(key), failureMessage);
         }
         else if (key instanceof BooleanStoredPropertyKey)
         {
            Assertions.assertEquals(parametersA.get(key), parametersB.get(key), failureMessage);
         }
      }
   }

   private static SwingPlannerParametersReadOnly getRandomParameters(Random random)
   {
      SwingPlannerParametersBasics parameters = new DefaultSwingPlannerParameters();

      for (StoredPropertyKey<?> key : SwingPlannerParameterKeys.keys.keys())
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
