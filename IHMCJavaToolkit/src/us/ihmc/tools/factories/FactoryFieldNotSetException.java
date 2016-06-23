package us.ihmc.tools.factories;

@SuppressWarnings("serial")
public class FactoryFieldNotSetException extends RuntimeException
{
   public FactoryFieldNotSetException(String fieldName)
   {
      super("Field " + fieldName + " has not been set!");
   }
}
