package us.ihmc.humanoidBehaviors.tools;

import org.apache.commons.lang3.reflect.FieldUtils;
import us.ihmc.log.LogTools;

/**
 * Interface to streamline creating builders with checks for each field.
 * <p>
 * Add fields to the builder by using
 * <p>
 * private final Field<Type> name = required();
 * private final IntField intName = requiredInt();
 * private final LongField longName = requiredLong();
 * private final DoubleField doubleName = requiredDouble();
 * private final BooleanField booleanName = requiredBoolean();
 * <p>
 * In your build() function, call validate() before accessing the values using FieldBasics.get().
 *
 * @author Jesper Smith
 * @author Duncan Calvert
 */
public interface BehaviorBuilderPattern
{
   abstract class FieldBasics
   {
      private final boolean required;
      private final boolean changing;
      private boolean set = false;
      private boolean valid = false;

      private final String clickableCodeLocation;

      private FieldBasics(boolean required, boolean changing)
      {
         this.required = required;
         this.changing = changing;

         StackTraceElement[] stackTrace = new Throwable().getStackTrace();
         StackTraceElement firstStackTraceElement = stackTrace[5];
         clickableCodeLocation = "(" + firstStackTraceElement.getFileName() + ":" + firstStackTraceElement.getLineNumber() + ")";
      }

      protected void markSet()
      {
         this.set = true;
      }

      private void invalidateChanging()
      {
         if (changing)
         {
            set = false;
            valid = false;
         }
      }

      private void validateNonChanging(String name)
      {
         if (!changing && !set)
         {
            nonValidAction(name);
         }

         valid = true;
      }

      private void validateAll(String name)
      {
         if (!set)
         {
            nonValidAction(name);
         }

         valid = true;
      }

      private void nonValidAction(String name)
      {
         if (required)
         {
            throw new RuntimeException("Field not set: " + name + " " + clickableCodeLocation);
         }
         else
         {
            LogTools.info("Optional field not set: {} {}", name, clickableCodeLocation);
         }
      }

      protected void checkValid()
      {
         if (!valid)
         {
            throw new RuntimeException("Call validate() before accessing declared fields.");
         }
      }

      public boolean isSet()
      {
         return set;
      }
   }

   class Field<T> extends FieldBasics
   {
      private T value = null;

      private Field(boolean required, boolean changing, T initial)
      {
         super(required, changing);
         this.value = initial;
      }

      public T get()
      {
         checkValid();
         return value;
      }

      public void set(T value)
      {
         this.value = value;
         markSet();
      }
   }

   class DoubleField extends FieldBasics
   {
      private double value;

      private DoubleField(boolean required, boolean changing, double initial)
      {
         super(required, changing);
         this.value = initial;
      }

      public double get()
      {
         checkValid();
         return value;
      }

      public void set(double value)
      {
         this.value = value;
         markSet();
      }
   }

   class IntField extends FieldBasics
   {
      private int value;

      private IntField(boolean required, boolean changing, int initial)
      {
         super(required, changing);
         this.value = initial;
      }

      public int get()
      {
         checkValid();
         return value;
      }

      public void set(int value)
      {
         this.value = value;
         markSet();
      }
   }

   class LongField extends FieldBasics
   {
      private long value;

      private LongField(boolean required, boolean changing, long initial)
      {
         super(required, changing);
         this.value = initial;
      }

      public long get()
      {
         checkValid();
         return value;
      }

      public void set(long value)
      {
         this.value = value;
         markSet();
      }
   }

   class BooleanField extends FieldBasics
   {
      private boolean value;

      private BooleanField(boolean required, boolean changing, boolean initial)
      {
         super(required, changing);
         this.value = initial;
      }

      public boolean get()
      {
         checkValid();
         return value;
      }

      public void set(boolean value)
      {
         this.value = value;
         markSet();
      }
   }

   default <T> Field<T> required()
   {
      return new Field<T>(true, false, null);
   }

   default <T> Field<T> requiredChanging()
   {
      return new Field<T>(true, true, null);
   }

   default IntField requiredInt()
   {
      return new IntField(true, false, 0);
   }

   default DoubleField requiredDouble()
   {
      return new DoubleField(true, false, Double.NaN);
   }

   default LongField requiredLong()
   {
      return new LongField(true, false, 0);
   }

   default BooleanField requiredBoolean()
   {
      return new BooleanField(true, false, false);
   }

   default <T> Field<T> optional(T defaultValue)
   {
      return new Field<T>(false, false, defaultValue);
   }

   default IntField optionalInt(int defaultValue)
   {
      return new IntField(false, false, defaultValue);
   }

   default DoubleField optionalDouble(double defaultValue)
   {
      return new DoubleField(false, false, defaultValue);
   }

   default LongField optionalLong(long defaultValue)
   {
      return new LongField(false, false, defaultValue);
   }

   default BooleanField optionalBoolean(boolean defaultValue)
   {
      return new BooleanField(false, false, defaultValue);
   }

   default void validateAll()
   {
      for (java.lang.reflect.Field field : FieldUtils.getAllFieldsList(getClass()))
      {
         if (FieldBasics.class.isAssignableFrom(field.getType()))
         {
            try
            {
               field.setAccessible(true);

               FieldBasics member = (FieldBasics) field.get(this);
               member.validateAll(field.getName());
            }
            catch (IllegalArgumentException | IllegalAccessException e)
            {
               throw new RuntimeException("Cannot validate field " + field.getName(), e);
            }
         }
      }
   }

   default void validateNonChanging()
   {
      for (java.lang.reflect.Field field : FieldUtils.getAllFieldsList(getClass()))
      {
         if (FieldBasics.class.isAssignableFrom(field.getType()))
         {
            try
            {
               field.setAccessible(true);

               FieldBasics member = (FieldBasics) field.get(this);
               member.validateNonChanging(field.getName());
            }
            catch (IllegalArgumentException | IllegalAccessException e)
            {
               throw new RuntimeException("Cannot validate field " + field.getName(), e);
            }
         }
      }
   }

   default void invalidateChanging()
   {
      for (java.lang.reflect.Field field : FieldUtils.getAllFieldsList(getClass()))
      {
         if (FieldBasics.class.isAssignableFrom(field.getType()))
         {
            try
            {
               field.setAccessible(true);

               FieldBasics member = (FieldBasics) field.get(this);
               member.invalidateChanging();
            }
            catch (IllegalArgumentException | IllegalAccessException e)
            {
               throw new RuntimeException("Cannot validate field " + field.getName(), e);
            }
         }
      }
   }
}