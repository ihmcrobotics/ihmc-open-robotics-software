package us.ihmc.footstepPlanning.tools;

import controller_msgs.msg.dds.FootstepPlannerParametersPacket;
import controller_msgs.msg.dds.VisibilityGraphsParametersPacket;
import org.junit.jupiter.api.Test;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.footstepPlanning.graphSearch.parameters.DefaultFootstepPlannerParameters;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParameterKeys;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersBasics;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersReadOnly;
import us.ihmc.pathPlanning.visibilityGraphs.parameters.DefaultVisibilityGraphParameters;
import us.ihmc.pathPlanning.visibilityGraphs.parameters.VisibilityGraphParametersKeys;
import us.ihmc.pathPlanning.visibilityGraphs.parameters.VisibilityGraphsParametersBasics;
import us.ihmc.pathPlanning.visibilityGraphs.parameters.VisibilityGraphsParametersReadOnly;
import us.ihmc.tools.property.BooleanStoredPropertyKey;
import us.ihmc.tools.property.DoubleStoredPropertyKey;
import us.ihmc.tools.property.IntegerStoredPropertyKey;
import us.ihmc.tools.property.StoredPropertyKey;

import java.util.Random;

import static us.ihmc.robotics.Assert.assertEquals;
import static us.ihmc.robotics.Assert.assertNotEquals;

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

   @Test
   public void testNotSettingFromPacket()
   {
      FootstepPlannerParametersPacket packet = new FootstepPlannerParametersPacket();
      FootstepPlannerParametersBasics parametersToSet = new DefaultFootstepPlannerParameters();
      FootstepPlannerParametersBasics expectedParameters = new DefaultFootstepPlannerParameters();

      parametersToSet.set(packet);

      assertDoubleParametersDontContainNoValue(expectedParameters, parametersToSet);
   }

   @Test
   public void testVisibilityGraphPacketCopying()
   {
      VisibilityGraphsParametersBasics parametersToSet = new DefaultVisibilityGraphParameters();
      VisibilityGraphsParametersPacket packet = new VisibilityGraphsParametersPacket();
      Random random = new Random(1738L);
      for (int i = 0; i < 1000; i++)
      {
         VisibilityGraphsParametersReadOnly randomParameters = getRandomVisibilityGraphParameters(random);
         FootstepPlannerMessageTools.copyParametersToPacket(packet, randomParameters);
         parametersToSet.set(packet);

         assertParametersEqual(randomParameters, parametersToSet, 1e-5);
      }
   }

   @Test
   public void testNotSettingVisibilityGraphFromPacket()
   {
      VisibilityGraphsParametersPacket packet = new VisibilityGraphsParametersPacket();
      VisibilityGraphsParametersBasics parametersToSet = new DefaultVisibilityGraphParameters();
      VisibilityGraphsParametersBasics expectedParameters = new DefaultVisibilityGraphParameters();

      parametersToSet.set(packet);

      assertDoubleParametersDontContainNoValue(expectedParameters, parametersToSet);
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
            assertNotEquals(FootstepPlannerParametersPacket.DEFAULT_NO_VALUE, parametersB.get(doubleKey), 1e-5);
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

   private static void assertParametersEqual(VisibilityGraphsParametersReadOnly parametersA, VisibilityGraphsParametersReadOnly parametersB, double epsilon)
   {
      for (StoredPropertyKey<?> key : VisibilityGraphParametersKeys.keys.keys())
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

   private static void assertDoubleParametersDontContainNoValue(FootstepPlannerParametersReadOnly parametersA, FootstepPlannerParametersReadOnly parametersB)
   {
      for (StoredPropertyKey<?> key : FootstepPlannerParameterKeys.keys.keys())
      {
         String failureMessage = key.getTitleCasedName() + " contains a no value.";
         if (key instanceof DoubleStoredPropertyKey)
         {
            DoubleStoredPropertyKey doubleKey = (DoubleStoredPropertyKey) key;
            assertNotEquals(failureMessage, FootstepPlannerParametersPacket.DEFAULT_NO_VALUE, parametersB.get(doubleKey), 1e-5);
            assertNotEquals(failureMessage, FootstepPlannerParametersPacket.DEFAULT_NO_VALUE, parametersA.get(doubleKey), 1e-5);
         }
      }
   }

   private static void assertDoubleParametersDontContainNoValue(VisibilityGraphsParametersReadOnly parametersA, VisibilityGraphsParametersReadOnly parametersB)
   {
      for (StoredPropertyKey<?> key : VisibilityGraphParametersKeys.keys.keys())
      {
         String failureMessage = key.getTitleCasedName() + " contains a no value.";
         if (key instanceof DoubleStoredPropertyKey)
         {
            DoubleStoredPropertyKey doubleKey = (DoubleStoredPropertyKey) key;
            assertNotEquals(failureMessage, VisibilityGraphsParametersPacket.DEFAULT_NO_VALUE, parametersB.get(doubleKey), 1e-5);
            assertNotEquals(failureMessage, VisibilityGraphsParametersPacket.DEFAULT_NO_VALUE, parametersA.get(doubleKey), 1e-5);
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
            int randomValue = RandomNumbers.nextInt(random, 1, 10);
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

   private static VisibilityGraphsParametersReadOnly getRandomVisibilityGraphParameters(Random random)
   {
      DefaultVisibilityGraphParameters visibilityGraph = new DefaultVisibilityGraphParameters();

      for (StoredPropertyKey<?> key : VisibilityGraphParametersKeys.keys.keys())
      {
         if (key instanceof DoubleStoredPropertyKey)
         {
            double randomValue = RandomNumbers.nextDouble(random, -10.0, 10.0);
            visibilityGraph.set(((DoubleStoredPropertyKey) key), randomValue);
         }
         else if (key instanceof IntegerStoredPropertyKey)
         {
            int randomValue = RandomNumbers.nextInt(random, 1, 10);
            visibilityGraph.set(((IntegerStoredPropertyKey) key), randomValue);
         }
         else if (key instanceof BooleanStoredPropertyKey)
         {
            boolean randomValue = RandomNumbers.nextBoolean(random, 0.5);
            visibilityGraph.set(((BooleanStoredPropertyKey) key), randomValue);
         }
      }

      return visibilityGraph;
   }
}
