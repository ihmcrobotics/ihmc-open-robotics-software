package us.ihmc.tools.factories;

public class OptionalFactoryField<T> extends FactoryField<T>
{
   public OptionalFactoryField(String fieldName)
   {
      super(fieldName);
   }

   public boolean hasBeenSet()
   {
      return hasBeenSet;
   }
}
