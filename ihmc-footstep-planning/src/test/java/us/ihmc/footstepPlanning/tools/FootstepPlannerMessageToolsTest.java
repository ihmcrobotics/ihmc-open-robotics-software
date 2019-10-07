package us.ihmc.footstepPlanning.tools;

import controller_msgs.msg.dds.FootstepPlannerParametersPacket;
import org.junit.jupiter.api.Test;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.footstepPlanning.graphSearch.parameters.DefaultFootstepPlannerParameters;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParameterKeys;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersBasics;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersReadOnly;
import us.ihmc.tools.property.BooleanStoredPropertyKey;
import us.ihmc.tools.property.DoubleStoredPropertyKey;
import us.ihmc.tools.property.IntegerStoredPropertyKey;
import us.ihmc.tools.property.StoredPropertyKey;

import java.util.Random;

import static us.ihmc.robotics.Assert.assertEquals;

public class FootstepPlannerMessageToolsTest
{
   private static final int iters = 1000;

   @Test
   public void testVariableCopying()
   {
      Random random = new Random(1738L);

      FootstepPlannerParametersPacket packet = new FootstepPlannerParametersPacket();
      FootstepPlannerParametersBasics parametersToSet = new DefaultFootstepPlannerParameters();

      for (int iter = 0; iter < iters; iter++)
      {
         FootstepPlannerParametersReadOnly randomParameters = getRandomParameters(random);
         FootstepPlannerMessageTools.copyParametersToPacket(packet, randomParameters);
         parametersToSet.set(packet);

         assertParametersEqual(randomParameters, parametersToSet);
      }
   }

   private static void assertParametersEqual(FootstepPlannerParametersReadOnly parametersA, FootstepPlannerParametersReadOnly parametersB)
   {
      for (StoredPropertyKey<?> key : FootstepPlannerParameterKeys.keys.keys())
      {
         String failureMessage = key.getTitleCasedName() + " has the wrong value.";
         if (key instanceof DoubleStoredPropertyKey)
         {
            DoubleStoredPropertyKey doubleKey = (DoubleStoredPropertyKey) key;
            assertEquals(failureMessage, parametersA.get(doubleKey), parametersB.get(doubleKey), 1e-5);
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

   private static FootstepPlannerParametersReadOnly getRandomParameters(Random random)
   {
      DefaultFootstepPlannerParameters footstepPlannerParameters = new DefaultFootstepPlannerParameters();

      for (StoredPropertyKey<?> key : FootstepPlannerParameterKeys.keys.keys())
      {
         if (key instanceof DoubleStoredPropertyKey)
         {
            double randomValue = RandomNumbers.nextDouble(random, -10.0, 10.0);
            footstepPlannerParameters.set(((DoubleStoredPropertyKey) key), randomValue);
         }
         else if (key instanceof IntegerStoredPropertyKey)
         {
            int randomValue = RandomNumbers.nextInt(random, -10, 10);
            footstepPlannerParameters.set(((IntegerStoredPropertyKey) key), randomValue);
         }
         else if (key instanceof BooleanStoredPropertyKey)
         {
            boolean randomValue = RandomNumbers.nextBoolean(random, 0.5);
            footstepPlannerParameters.set(((BooleanStoredPropertyKey) key), randomValue);
         }
      }

      return footstepPlannerParameters;
   }
}
