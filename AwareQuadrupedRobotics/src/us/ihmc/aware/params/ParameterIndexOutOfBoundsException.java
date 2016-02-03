package us.ihmc.aware.params;

/**
 * Thrown when a parameter name exists in the map, but the requested index is larger than the parameter length.
 */
public class ParameterIndexOutOfBoundsException extends RuntimeException
{
   public ParameterIndexOutOfBoundsException(String paramName, int idx)
   {
      super("Index " + idx + " of " + paramName + " does not exist");
   }
}
