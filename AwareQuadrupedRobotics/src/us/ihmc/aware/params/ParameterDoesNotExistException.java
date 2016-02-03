package us.ihmc.aware.params;

/**
 * Thrown when the requested parameter has not been set in the map.
 */
public class ParameterDoesNotExistException extends RuntimeException
{
   public ParameterDoesNotExistException(String paramName)
   {
      super("Parameter does not exist: " + paramName);
   }
}
