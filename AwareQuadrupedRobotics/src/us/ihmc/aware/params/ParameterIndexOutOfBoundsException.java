package us.ihmc.aware.params;

public class ParameterIndexOutOfBoundsException extends RuntimeException
{
   public ParameterIndexOutOfBoundsException(String paramName, int idx)
   {
      super("Index " + idx + " of " + paramName + " does not exist");
   }
}
