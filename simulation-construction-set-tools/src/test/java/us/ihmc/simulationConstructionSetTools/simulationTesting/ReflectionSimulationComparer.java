package us.ihmc.simulationConstructionSetTools.simulationTesting;

import static us.ihmc.robotics.Assert.assertEquals;

import java.lang.reflect.Field;
import java.util.AbstractList;
import java.util.ArrayList;
import java.util.Collection;
import java.util.LinkedHashSet;

import us.ihmc.robotics.math.filters.AlphaFilteredYoVariable;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.UnreasonableAccelerationException;
import us.ihmc.tools.reflect.RecursiveObjectComparer;
import us.ihmc.tools.reflect.StringFieldMatcher;
import us.ihmc.yoVariables.buffer.YoBuffer;
import us.ihmc.yoVariables.listener.YoRegistryChangedListener;
import us.ihmc.yoVariables.registry.YoVariableList;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;
import us.ihmc.yoVariables.variable.YoVariable;

/**
 * SimulationComparer wrapper around RecursiveObjectComparer
 */
public class ReflectionSimulationComparer implements SimulationComparer
{
   private RecursiveObjectComparer comparer;

   public ReflectionSimulationComparer(int maxDepth, int maxSize)
   {
      comparer = new RecursiveObjectComparer(maxDepth, maxSize);
   }

   @Override
   public boolean compare(SimulationConstructionSet scs0, SimulationConstructionSet scs1)
   {
      try
      {
         return comparer.compare(scs0, scs1);
      }
      catch (IllegalArgumentException e)
      {
         e.printStackTrace();
      }
      catch (IllegalAccessException e)
      {
         e.printStackTrace();
      }

      return false;
   }

   public void addFieldToIgnore(Field fieldToIgnore)
   {
      comparer.addFieldToIgnore(fieldToIgnore);
   }

   public void addFieldsToIgnore(Collection<Field> fieldsToIgnore)
   {
      comparer.addFieldsToIgnore(fieldsToIgnore);
   }

   public void addClassToIgnore(Class<?> classToIgnore)
   {
      comparer.addClassToIgnore(classToIgnore);
   }

   public void addClassesToIgnore(Collection<Class<?>> classesToIgnore)
   {
      comparer.addClassesToIgnore(classesToIgnore);
   }

   public void addStringFieldsToIgnore(StringFieldMatcher stringFieldMatcherToIgnore)
   {
      comparer.addStringFieldsToIgnore(stringFieldMatcherToIgnore);
   }

   public Collection<Field> getDifferingFields()
   {
      return comparer.getDifferingFields();
   }

   @Override
   public String toString()
   {
      return comparer.toString();
   }

   public static void compareTwoSimulations(SimulationConstructionSet scs0, SimulationConstructionSet scs1, SimulationComparisonScript script,
           boolean expectedResult, boolean closeAndDispose)
           throws IllegalArgumentException, IllegalAccessException, SecurityException, NoSuchFieldException, UnreasonableAccelerationException
   {
      int maxDepth = Integer.MAX_VALUE;
      int maxSize = Integer.MAX_VALUE;

      script.doInitialAction(scs0, scs1);

      ReflectionSimulationComparer preRewindComparer = new ReflectionSimulationComparer(maxDepth, maxSize);

      addStandardStuffToIgnore(preRewindComparer);
      System.out.println("Comparing to find fields to ignore...");

      Collection<Field> preRewindFieldDifferences = compareToFindFieldsToIgnore(scs0, scs1, preRewindComparer);

//    System.err.println("PreRewindComparer:\n" + preRewindComparer.toString());

      preRewindComparer = null;
      System.out.println("Pre rewind field differences:\n" + preRewindFieldDifferences);
      System.out.println("Pre-rewind compare done");

      script.doFinalAction(scs0, scs1);
      assertEquals("Indices are not equal after final action. Most likely that you overran the data buffer. Check the data buffer size!", scs0.getCurrentIndex(),
                   scs1.getCurrentIndex());

      Collection<Field> fieldsToIgnore = new ArrayList<Field>();
      Collection<String> fieldNamesToNeverIgnore = new ArrayList<String>();
      fieldNamesToNeverIgnore.add("private int us.ihmc.simulationconstructionset.YoInteger.val");
      fieldNamesToNeverIgnore.add("private double us.ihmc.simulationconstructionset.YoDouble.val");
      fieldNamesToNeverIgnore.add("private boolean us.ihmc.simulationconstructionset.YoBoolean.val");

      for (Field field : preRewindFieldDifferences)
      {
         String fieldName = field.toString();
         if (!fieldNamesToNeverIgnore.contains(fieldName))
         {
            fieldsToIgnore.add(field);
         }
         else
         {
            System.err.println("There was a field to never ignore in the preRewind comparison! Field = " + field);
         }
      }

      System.out.println("Fields to ignore:\n" + fieldsToIgnore);


      ReflectionSimulationComparer postRewindComparer = new ReflectionSimulationComparer(maxDepth, maxSize);

      addStandardStuffToIgnore(postRewindComparer);
      postRewindComparer.addFieldsToIgnore(fieldsToIgnore);
      System.out.println("Comparing...");

      boolean result = postRewindComparer.compare(scs0, scs1);
      String comparerToString = postRewindComparer.toString();

      System.out.println("Post-rewind compare done");

      preRewindComparer = null;
      postRewindComparer = null;

      scs0.closeAndDispose();
      scs1.closeAndDispose();

      if (!result)
      {
         System.err.println(comparerToString);
      }

      assertEquals(comparerToString, expectedResult, result);

   }

   private static void addStandardStuffToIgnore(ReflectionSimulationComparer comparer)
   {
      comparer.addClassToIgnore(YoBuffer.class);
      comparer.addClassToIgnore(YoVariableList.class);
      comparer.addClassToIgnore(YoRegistryChangedListener.class);    // to break the link from a controller to the SCS object

      try
      {
         StringFieldMatcher stringFieldMatcherToIgnore = new StringFieldMatcher();
         stringFieldMatcherToIgnore.addStringFieldToMatchRegularExpression(YoInteger.class, YoVariable.class.getDeclaredField("name"),
                 ".*StartTimeNano");
         stringFieldMatcherToIgnore.addStringFieldToMatchRegularExpression(YoInteger.class, YoVariable.class.getDeclaredField("name"),
                 ".*StopTimeNano");

         stringFieldMatcherToIgnore.addStringFieldToMatchRegularExpression(YoDouble.class, YoVariable.class.getDeclaredField("name"),
                 ".*DurationMilli");
         stringFieldMatcherToIgnore.addStringFieldToMatchRegularExpression(AlphaFilteredYoVariable.class, YoVariable.class.getDeclaredField("name"),
                 ".*DurationMilli");

         comparer.addStringFieldsToIgnore(stringFieldMatcherToIgnore);

         comparer.addFieldToIgnore(AbstractList.class.getDeclaredField("modCount"));
      }
      catch (SecurityException e)
      {
         // TODO Auto-generated catch block
         e.printStackTrace();
      }
      catch (NoSuchFieldException e)
      {
         // TODO Auto-generated catch block
         e.printStackTrace();
      }
   }

   private static Collection<Field> compareToFindFieldsToIgnore(SimulationConstructionSet scs0, SimulationConstructionSet scs1,
           ReflectionSimulationComparer comparer)
   {
      comparer.compare(scs0, scs1);

      return new LinkedHashSet<Field>(comparer.getDifferingFields());
   }
}
