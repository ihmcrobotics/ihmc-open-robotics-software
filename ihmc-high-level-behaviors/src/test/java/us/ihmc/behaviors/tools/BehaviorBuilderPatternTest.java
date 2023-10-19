package us.ihmc.behaviors.tools;

import org.junit.jupiter.api.Test;
import us.ihmc.behaviors.tools.BehaviorBuilderPattern;
import us.ihmc.log.LogTools;
import us.ihmc.tools.thread.MissingThreadTools;
import us.ihmc.tools.thread.ResettableExceptionHandlingExecutorService;

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
   public void testExtendingClass()
   {
      ExtendingClass extendingClass = new ExtendingClass();

      assertThrows(RuntimeException.class, () ->
      {
         try
         {
            extendingClass.run();
         }
         catch (Throwable t)
         {
            LogTools.info("Exception thrown: {}", t.getMessage());
            assertTrue(t.getMessage().startsWith("Field not set"));
            throw t;
         }
      });

      extendingClass.setFieldOne(new Object());
      extendingClass.setFieldTwo(new Object());

      assertThrows(RuntimeException.class, () ->
      {
         try
         {
            extendingClass.run();
         }
         catch (Throwable t)
         {
            LogTools.info("Exception thrown: {}", t.getMessage());
            assertTrue(t.getMessage().startsWith("Field not set"));
            throw t;
         }
      });

      extendingClass.setParentField(new Object());
      extendingClass.run();

      ExtendingClass extendingClass2 = new ExtendingClass();

      extendingClass2.setParentField(new Object());

      assertThrows(RuntimeException.class, () ->
      {
         try
         {
            extendingClass2.run();
         }
         catch (Throwable t)
         {
            LogTools.info("Exception thrown: {}", t.getMessage());
            assertTrue(t.getMessage().startsWith("Field not set"));
            throw t;
         }
      });

      extendingClass2.setFieldOne(new Object());
      extendingClass2.setFieldTwo(new Object());
      extendingClass2.run();
   }

   @Test
   public void testTiered()
   {
      TieredParent teired = new TieredParent();

      assertThrows(RuntimeException.class, () ->
      {
         try
         {
            teired.run();
         }
         catch (Throwable t)
         {
            LogTools.info("Exception thrown: {}", t.getMessage());
            assertTrue(t.getMessage().startsWith("Field not set"));
            throw t;
         }
      });

      teired.setFieldOne(new Object());
      teired.setParentField(new Object());

      teired.runParent();
      teired.runParent();

      assertThrows(RuntimeException.class, () ->
      {
         try
         {
            teired.run();
         }
         catch (Throwable t)
         {
            LogTools.info("Exception thrown: {}", t.getMessage());
            assertTrue(t.getMessage().startsWith("Field not set"));
            throw t;
         }
      });
   }

   @Test
   public void testBuildWithThread()
   {
      ResettableExceptionHandlingExecutorService executor = MissingThreadTools.newSingleThreadExecutor(getClass().getSimpleName(), true, 1);

      AllRequiredClass allRequired = new AllRequiredClass();

      LogTools.info("Exceptions are supposed to print out here.");
      executor.clearQueueAndExecute(() -> allRequired.run());
      executor.clearQueueAndExecute(() -> allRequired.run());
      executor.clearQueueAndExecute(() -> allRequired.run());
   }

   class ExtendingClass extends AllRequiredClass
   {
      private final Field<Object> parentField = required();

      public void setParentField(Object parentField)
      {
         this.parentField.set(parentField);
      }
   }

   class AllRequiredClass implements BehaviorBuilderPattern
   {
      private final Field<Object> fieldOne = required();
      private final Field<Object> fieldTwo = requiredChanging();

      public void run()
      {
         validateAll();
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

   class TieredParent extends TieredChild
   {
      private final Field<Object> parentField = required();

      public void runParent()
      {
         validateNonChanging();

         setFieldTwo(parentField.get());

         run();
      }

      public void setParentField(Object parentField)
      {
         this.parentField.set(parentField);
      }
   }

   class TieredChild implements BehaviorBuilderPattern
   {
      private final Field<Object> fieldOne = required();
      private final Field<Object> fieldTwo = requiredChanging();

      protected void run()
      {
         validateAll();

         fieldOne.get();
         fieldTwo.get();

         invalidateChanging();
      }

      public void setFieldOne(Object object)
      {
         this.fieldOne.set(object);
      }

      protected void setFieldTwo(Object object)
      {
         this.fieldTwo.set(object);
      }
   }
}
