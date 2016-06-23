package us.ihmc.tools.exceptions;

@SuppressWarnings("serial")
public class FactoryFieldNotSetException extends RuntimeException
{
   public FactoryFieldNotSetException(String fieldName)
   {
      super("Field " + fieldName + " has not been set!");
   }
}
