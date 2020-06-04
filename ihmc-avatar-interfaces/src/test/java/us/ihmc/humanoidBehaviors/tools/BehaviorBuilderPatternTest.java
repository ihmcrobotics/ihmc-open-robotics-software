package us.ihmc.humanoidBehaviors.tools;

import org.junit.jupiter.api.Test;
import us.ihmc.log.LogTools;

import java.util.Objects;

import static org.junit.jupiter.api.Assertions.*;

public class BehaviorBuilderPatternTest
{
   @Test
   public void testBuilder()
   {
      AllRequiredClass allRequiredClass = new AllRequiredClass();

      assertThrows(RuntimeException.class, () ->
      {
         try
         {
            allRequiredClass.run();
         }
         catch (Throwable t)
         {
            LogTools.info("Exception thrown: {}", t.getMessage());
            assertTrue(t.getMessage().startsWith("Field not set"));
            throw t;
         }
      });

      allRequiredClass.setFieldOne(new Object());
      allRequiredClass.setFieldTwo(new Object());
      allRequiredClass.run();

      allRequiredClass.invalidateChanging();
      assertThrows(RuntimeException.class, () ->
      {
         try
         {
            allRequiredClass.run();
         }
         catch (Throwable t)
         {
            LogTools.info("Exception thrown: {}", t.getMessage());
            assertTrue(t.getMessage().startsWith("Field not set"));
            throw t;
         }
      });

      allRequiredClass.setFieldTwo(new Object());
      allRequiredClass.run();
   }

   @Test
   public void testBuildWithThread()
   {

   }

   class AllRequiredClass implements BehaviorBuilderPattern
   {
      private final Field<Object> fieldOne = required();
      private final Field<Object> fieldTwo = requiredChanging();

      public void run()
      {
         validate();
      }

      public void setFieldOne(Object object)
      {
         this.fieldOne.set(object);
      }

      public void setFieldTwo(Object object)
      {
         this.fieldTwo.set(object);
      }
   }

   class ExampleClass implements BehaviorBuilderPattern
   {
      BooleanField optionalBooleanField = optionalBoolean(false);
      BooleanField requiredBooleanField = requiredBoolean();
//      BooleanField optionalBooleanField = optionalInt(requiredBooleanFieldq)
   }
}
