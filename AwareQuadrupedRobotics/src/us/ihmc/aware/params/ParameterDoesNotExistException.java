package us.ihmc.aware.params;

public class ParameterDoesNotExistException extends RuntimeException
{
   public ParameterDoesNotExistException(String paramName)
   {
      super("Parameter does not exist: " + paramName);
   }
}
