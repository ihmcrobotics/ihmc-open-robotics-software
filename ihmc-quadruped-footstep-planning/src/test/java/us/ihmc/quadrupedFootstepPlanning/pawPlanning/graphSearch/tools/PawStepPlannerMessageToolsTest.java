package us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.tools;

import controller_msgs.msg.dds.VisibilityGraphsParametersPacket;
import org.junit.jupiter.api.Test;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.pathPlanning.visibilityGraphs.parameters.DefaultVisibilityGraphParameters;
import us.ihmc.pathPlanning.visibilityGraphs.parameters.VisibilityGraphParametersKeys;
import us.ihmc.pathPlanning.visibilityGraphs.parameters.VisibilityGraphsParametersBasics;
import us.ihmc.pathPlanning.visibilityGraphs.parameters.VisibilityGraphsParametersReadOnly;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.tools.PawStepPlannerMessageTools;
import us.ihmc.tools.property.BooleanStoredPropertyKey;
import us.ihmc.tools.property.DoubleStoredPropertyKey;
import us.ihmc.tools.property.IntegerStoredPropertyKey;
import us.ihmc.tools.property.StoredPropertyKey;

import java.util.Random;

import static us.ihmc.robotics.Assert.assertEquals;

public class PawStepPlannerMessageToolsTest
{
   @Test
   public void testPacketCopying()
   {
      VisibilityGraphsParametersBasics parametersToSet = new DefaultVisibilityGraphParameters();
      VisibilityGraphsParametersPacket packet = new VisibilityGraphsParametersPacket();
      Random random = new Random(1738L);
      for (int i = 0; i < 1000; i++)
      {
         VisibilityGraphsParametersReadOnly randomParameters = createRandomParameters(random);
         PawStepPlannerMessageTools.copyParametersToPacket(packet, randomParameters);
         parametersToSet.set(packet);

         assertParametersEqual(randomParameters, parametersToSet, 1e-5);
      }
   }

   private static VisibilityGraphsParametersReadOnly createRandomParameters(Random random)
   {
      VisibilityGraphsParametersBasics parameters = new DefaultVisibilityGraphParameters();

      for (StoredPropertyKey<?> key : VisibilityGraphParametersKeys.keys.keys())
      {
         if (key instanceof DoubleStoredPropertyKey)
         {
            DoubleStoredPropertyKey doubleKey = (DoubleStoredPropertyKey) key;
            parameters.set(doubleKey, RandomNumbers.nextDouble(random, 10.0));
         }
         else if (key instanceof IntegerStoredPropertyKey)
         {
            IntegerStoredPropertyKey integerKey = (IntegerStoredPropertyKey) key;
            parameters.set(integerKey, RandomNumbers.nextInt(random, 0, 10));
         }
         else if (key instanceof BooleanStoredPropertyKey)
         {
            BooleanStoredPropertyKey booleanKey = (BooleanStoredPropertyKey) key;
            parameters.set(booleanKey, RandomNumbers.nextBoolean(random, 0.5));
         }
      }

      return parameters;
   }

   private static void assertParametersEqual(VisibilityGraphsParametersReadOnly parametersA, VisibilityGraphsParametersReadOnly parametersB, double epsilon)
   {
      for (StoredPropertyKey<?> key : VisibilityGraphParametersKeys.keys.keys())
      {
         String message = "Key " + key.getCamelCasedName() + " was not copied correctly.";
         if (key instanceof DoubleStoredPropertyKey)
         {
            DoubleStoredPropertyKey doubleKey = (DoubleStoredPropertyKey) key;
            assertEquals(message, parametersA.get(doubleKey), parametersB.get(doubleKey), epsilon);
         }
         else if (key instanceof IntegerStoredPropertyKey)
         {
            IntegerStoredPropertyKey integerKey = (IntegerStoredPropertyKey) key;
            assertEquals(message, parametersA.get(integerKey), parametersB.get(integerKey));
         }
         else if (key instanceof BooleanStoredPropertyKey)
         {
            BooleanStoredPropertyKey booleanKey = (BooleanStoredPropertyKey) key;
            assertEquals(message, parametersA.get(booleanKey), parametersB.get(booleanKey));
         }
      }
   }
}
