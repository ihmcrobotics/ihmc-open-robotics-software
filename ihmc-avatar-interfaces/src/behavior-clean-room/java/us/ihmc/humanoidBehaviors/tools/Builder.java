package us.ihmc.humanoidBehaviors.tools;

import org.apache.commons.lang3.reflect.FieldUtils;

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
public interface Builder
{
   abstract class FieldBasics
   {
      private final boolean required;
      private boolean set = false;
      private boolean valid = false;

      private final StackTraceElement[] cause;

      private FieldBasics(boolean required)
      {
         this.required = required;

         StackTraceElement[] stackTrace = new Throwable().getStackTrace();
         if (stackTrace.length > 5)
         {
            this.cause = new StackTraceElement[stackTrace.length - 5];
            System.arraycopy(stackTrace, 5, cause, 0, cause.length);
         }
         else
         {
            // Weird state, but don't fail
            this.cause = stackTrace;
         }
      }

      protected void markSet()
      {
         this.set = true;
      }

      private void invalidate()
      {
         set = false;
         valid = false;
      }

      private void validate(Class<? extends Builder> caller, String name)
      {
         if (!set)
         {

            if (required)
            {
               RuntimeException requiredException = new RuntimeException("Field not set: " + name);
               requiredException.setStackTrace(cause);
               throw requiredException;
            }
            else
            {
               System.out.println("[INFO] (" + cause[0].getFileName() + ":" + cause[0].getLineNumber() + ") Field not set: " + name);
            }
         }

         valid = true;
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

      private Field(boolean required, T initial)
      {
         super(required);
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

      private DoubleField(boolean required, double initial)
      {
         super(required);
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

      private IntField(boolean required, int initial)
      {
         super(required);
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

      private LongField(boolean required, long initial)
      {
         super(required);
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

      private BooleanField(boolean required, boolean initial)
      {
         super(required);
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
      return new Field<T>(true, null);
   }

   default IntField requiredInt()
   {
      return new IntField(true, 0);
   }

   default DoubleField requiredDouble()
   {
      return new DoubleField(true, Double.NaN);
   }

   default LongField requiredLong()
   {
      return new LongField(true, 0);
   }

   default BooleanField requiredBoolean()
   {
      return new BooleanField(true, false);
   }

   default <T> Field<T> optional(T defaultValue)
   {
      return new Field<T>(false, defaultValue);
   }

   default IntField optionalInt(int defaultValue)
   {
      return new IntField(false, defaultValue);
   }

   default DoubleField optionalDouble(double defaultValue)
   {
      return new DoubleField(false, defaultValue);
   }

   default LongField optionalLong(long defaultValue)
   {
      return new LongField(false, defaultValue);
   }

   default BooleanField optionalBoolean(boolean defaultValue)
   {
      return new BooleanField(false, defaultValue);
   }

   default void validate()
   {
      for (java.lang.reflect.Field field : FieldUtils.getAllFieldsList(getClass()))
      {
         if (FieldBasics.class.isAssignableFrom(field.getType()))
         {
            try
            {
               field.setAccessible(true);

               FieldBasics member = (FieldBasics) field.get(this);
               member.validate(getClass(), field.getName());
            }
            catch (IllegalArgumentException | IllegalAccessException e)
            {
               throw new RuntimeException("Cannot validate field " + field.getName(), e);
            }
         }
      }
   }

   default void invalidate()
   {
      for (java.lang.reflect.Field field : FieldUtils.getAllFieldsList(getClass()))
      {
         if (FieldBasics.class.isAssignableFrom(field.getType()))
         {
            try
            {
               field.setAccessible(true);

               FieldBasics member = (FieldBasics) field.get(this);
               member.invalidate();
            }
            catch (IllegalArgumentException | IllegalAccessException e)
            {
               throw new RuntimeException("Cannot validate field " + field.getName(), e);
            }
         }
      }
   }
}