package us.ihmc.footstepPlanning.tools;

import org.junit.jupiter.api.Assertions;
import toolbox_msgs.msg.dds.FootstepPlannerParametersPacket;
import toolbox_msgs.msg.dds.VisibilityGraphsParametersPacket;
import org.junit.jupiter.api.Test;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.footstepPlanning.graphSearch.parameters.DefaultFootstepPlannerParameters;
import us.ihmc.footstepPlanning.graphSearch.parameters.DefaultFootstepPlannerParametersBasics;
import us.ihmc.footstepPlanning.graphSearch.parameters.DefaultFootstepPlannerParametersReadOnly;
import us.ihmc.pathPlanning.visibilityGraphs.parameters.DefaultVisibilityGraphParameters;
import us.ihmc.pathPlanning.visibilityGraphs.parameters.VisibilityGraphParametersKeys;
import us.ihmc.pathPlanning.visibilityGraphs.parameters.VisibilityGraphsParametersBasics;
import us.ihmc.pathPlanning.visibilityGraphs.parameters.VisibilityGraphsParametersReadOnly;
import us.ihmc.tools.property.BooleanStoredPropertyKey;
import us.ihmc.tools.property.DoubleStoredPropertyKey;
import us.ihmc.tools.property.IntegerStoredPropertyKey;
import us.ihmc.tools.property.StoredPropertyKey;

import java.util.Random;

public class FootstepPlannerMessageToolsTest
{
   /**
    * The larger this value gets the more that gets printed to the terminal.
    * We don't really need to check this more then a few times.
    */
   private static final int iterations = 10;

   private static final Random RANDOM = new Random(1738L);

   @Test
   public void testVariableCopying()
   {
      FootstepPlannerParametersPacket packet = new FootstepPlannerParametersPacket();
      DefaultFootstepPlannerParametersBasics parametersToSet = new DefaultFootstepPlannerParameters();

      for (int i = 0; i < iterations; i++)
      {
         DefaultFootstepPlannerParametersReadOnly randomParameters = getRandomParameters();
         FootstepPlannerMessageTools.copyParametersToPacket(packet, randomParameters);
         parametersToSet.set(packet);

         assertParametersEqual(randomParameters, parametersToSet);
      }
   }

   @Test
   public void testNotSettingFromPacket()
   {
      FootstepPlannerParametersPacket packet = new FootstepPlannerParametersPacket();
      DefaultFootstepPlannerParametersBasics parametersToSet = new DefaultFootstepPlannerParameters();
      DefaultFootstepPlannerParametersBasics expectedParameters = new DefaultFootstepPlannerParameters();

      parametersToSet.set(packet);

      assertDoubleParametersDontContainNoValue(expectedParameters, parametersToSet);
   }

   @Test
   public void testVisibilityGraphPacketCopying()
   {
      VisibilityGraphsParametersBasics parametersToSet = new DefaultVisibilityGraphParameters();
      VisibilityGraphsParametersPacket packet = new VisibilityGraphsParametersPacket();

      for (int i = 0; i < iterations; i++)
      {
         VisibilityGraphsParametersReadOnly randomParameters = getRandomVisibilityGraphParameters();
         FootstepPlannerMessageTools.copyParametersToPacket(packet, randomParameters);
         parametersToSet.set(packet);

         assertParametersEqual(randomParameters, parametersToSet);
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

   private static void assertParametersEqual(DefaultFootstepPlannerParametersReadOnly parametersA, DefaultFootstepPlannerParametersReadOnly parametersB)
   {
      for (StoredPropertyKey<?> key : DefaultFootstepPlannerParameters.keys.keys())
      {
         String failureMessage = key.getTitleCasedName() + " has the wrong value.";
         if (key instanceof DoubleStoredPropertyKey doubleKey)
         {

            Assertions.assertEquals(parametersA.get(doubleKey), parametersB.get(doubleKey), failureMessage);
            Assertions.assertNotEquals(FootstepPlannerParametersPacket.DEFAULT_NO_VALUE, parametersB.get(doubleKey), failureMessage);
         }
         else if (key instanceof IntegerStoredPropertyKey integerKey)
         {
            Assertions.assertEquals(parametersA.get(integerKey), parametersB.get(integerKey), failureMessage);
         }
         else if (key instanceof BooleanStoredPropertyKey booleanKey)
         {
            Assertions.assertEquals(parametersA.get(booleanKey), parametersB.get(booleanKey), failureMessage);
         }
      }
   }

   private static void assertParametersEqual(VisibilityGraphsParametersReadOnly parametersA, VisibilityGraphsParametersReadOnly parametersB)
   {
      for (StoredPropertyKey<?> key : VisibilityGraphParametersKeys.keys.keys())
      {
         String failureMessage = key.getTitleCasedName() + " has the wrong value.";
         if (key instanceof DoubleStoredPropertyKey doubleKey)
         {
            Assertions.assertEquals(parametersA.get(doubleKey), parametersB.get(doubleKey), failureMessage);
            Assertions.assertNotEquals(FootstepPlannerParametersPacket.DEFAULT_NO_VALUE, parametersB.get(doubleKey), failureMessage);
         }
         else if (key instanceof IntegerStoredPropertyKey integerKey)
         {
            Assertions.assertEquals(parametersA.get(integerKey), parametersB.get(integerKey), failureMessage);
         }
         else if (key instanceof BooleanStoredPropertyKey booleanKey)
         {
            Assertions.assertEquals(parametersA.get(booleanKey), parametersB.get(booleanKey), failureMessage);
         }
      }
   }

   private static void assertDoubleParametersDontContainNoValue(DefaultFootstepPlannerParametersReadOnly parametersA,
                                                                DefaultFootstepPlannerParametersReadOnly parametersB)
   {
      for (StoredPropertyKey<?> key : DefaultFootstepPlannerParameters.keys.keys())
      {
         String failureMessage = key.getTitleCasedName() + " contains a no value.";
         if (key instanceof DoubleStoredPropertyKey doubleKey)
         {
            Assertions.assertNotEquals(FootstepPlannerParametersPacket.DEFAULT_NO_VALUE, parametersA.get(doubleKey), failureMessage);
            Assertions.assertNotEquals(FootstepPlannerParametersPacket.DEFAULT_NO_VALUE, parametersB.get(doubleKey), failureMessage);
         }
      }
   }

   private static void assertDoubleParametersDontContainNoValue(VisibilityGraphsParametersReadOnly parametersA, VisibilityGraphsParametersReadOnly parametersB)
   {
      for (StoredPropertyKey<?> key : VisibilityGraphParametersKeys.keys.keys())
      {
         String failureMessage = key.getTitleCasedName() + " contains a no value.";
         if (key instanceof DoubleStoredPropertyKey doubleKey)
         {
            Assertions.assertNotEquals(VisibilityGraphsParametersPacket.DEFAULT_NO_VALUE, parametersA.get(doubleKey), failureMessage);
            Assertions.assertNotEquals(VisibilityGraphsParametersPacket.DEFAULT_NO_VALUE, parametersB.get(doubleKey), failureMessage);
         }
      }
   }

   private static DefaultFootstepPlannerParametersReadOnly getRandomParameters()
   {
      DefaultFootstepPlannerParameters footstepPlannerParameters = new DefaultFootstepPlannerParameters();

      for (StoredPropertyKey<?> key : DefaultFootstepPlannerParameters.keys.keys())
      {
         if (key instanceof DoubleStoredPropertyKey)
         {
            double randomValue = RandomNumbers.nextDouble(FootstepPlannerMessageToolsTest.RANDOM, -10.0, 10.0);
            footstepPlannerParameters.set(((DoubleStoredPropertyKey) key), randomValue);
         }
         else if (key instanceof IntegerStoredPropertyKey)
         {
            int randomValue = RandomNumbers.nextInt(FootstepPlannerMessageToolsTest.RANDOM, 1, 10);
            footstepPlannerParameters.set(((IntegerStoredPropertyKey) key), randomValue);
         }
         else if (key instanceof BooleanStoredPropertyKey)
         {
            boolean randomValue = RandomNumbers.nextBoolean(FootstepPlannerMessageToolsTest.RANDOM, 0.5);
            footstepPlannerParameters.set(((BooleanStoredPropertyKey) key), randomValue);
         }
      }

      return footstepPlannerParameters;
   }

   private static VisibilityGraphsParametersReadOnly getRandomVisibilityGraphParameters()
   {
      DefaultVisibilityGraphParameters visibilityGraph = new DefaultVisibilityGraphParameters();

      for (StoredPropertyKey<?> key : VisibilityGraphParametersKeys.keys.keys())
      {
         if (key instanceof DoubleStoredPropertyKey)
         {
            double randomValue = RandomNumbers.nextDouble(FootstepPlannerMessageToolsTest.RANDOM, -10.0, 10.0);
            visibilityGraph.set(((DoubleStoredPropertyKey) key), randomValue);
         }
         else if (key instanceof IntegerStoredPropertyKey)
         {
            int randomValue = RandomNumbers.nextInt(FootstepPlannerMessageToolsTest.RANDOM, 1, 10);
            visibilityGraph.set(((IntegerStoredPropertyKey) key), randomValue);
         }
         else if (key instanceof BooleanStoredPropertyKey)
         {
            boolean randomValue = RandomNumbers.nextBoolean(FootstepPlannerMessageToolsTest.RANDOM, 0.5);
            visibilityGraph.set(((BooleanStoredPropertyKey) key), randomValue);
         }
      }

      return visibilityGraph;
   }
}
