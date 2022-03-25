package us.ihmc.tools.factories;

import java.util.Objects;

public class OptionalFactoryField<T> extends FactoryField<T>
{
   public OptionalFactoryField(String fieldName)
   {
      super(fieldName);
   }

   public OptionalFactoryField(String fieldName, T defaultValue)
   {
      this(fieldName);
      setDefaultValue(defaultValue);
   }

   public void setDefaultValue(T defaultValue)
   {
      Objects.requireNonNull(defaultValue);

      if (!hasBeenSet)
      {
         set(defaultValue);
      }
   }

   public boolean hasValue()
   {
      checkNotDisposed();

      return hasBeenSet;
   }
}
