package us.ihmc.quadrupedRobotics.params;

public abstract class Parameter
{
   private final String path;

   public Parameter(String path)
   {
      this.path = path;
   }

   /**
    * Attempt to load the given line as this particular type of parameter, with no a priori knowledge of the actual type.
    *
    * @param line the line from which to load
    * @return whether or not the line matched the expected format for this parameter type
    */
   public boolean tryLoad(String line)
   {
      String[] split = line.split("=");

      if (split.length < 2)
      {
         System.err.println("Malformed configuration line: " + line);
         return false;
      }

      String targetPath = split[0];
      String targetValue = split[1];

      if (path.equals(targetPath))
      {
         if (tryLoadValue(targetValue))
         {
            return true;
         }
         else
         {
            System.err.println("Malformed configuration line: " + line);
            return false;
         }
      }

      return false;
   }

   public abstract boolean tryLoadValue(String value);

   public String dump()
   {
      return path + "=" + dumpValue();
   }

   abstract String dumpValue();

   public String getPath()
   {
      return path;
   }
}
