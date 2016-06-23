package us.ihmc.tools.exceptions;

public class RequiredFactoryField<T> extends FactoryField<T>
{
   public RequiredFactoryField(String fieldName)
   {
      super(fieldName);
   }
}
