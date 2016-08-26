package us.ihmc.quadrupedRobotics.simulation;

import java.nio.file.Path;
import java.nio.file.Paths;

public enum QuadrupedParameterSet
{
   SIMULATION_IDEAL, SIMULATION_REAL, HARDWARE;
   
   private final Path parametersPath;
   
   private QuadrupedParameterSet()
   {
      parametersPath = Paths.get("parameters/" + name().toLowerCase() + ".param");
   }
   
   public Path getPath()
   {
      return parametersPath;
   }
   
   public static QuadrupedParameterSet fromName(String name)
   {
      if (name.equals(SIMULATION_IDEAL.name().toLowerCase()))
      {
         return SIMULATION_IDEAL;
      }
      else if (name.equals(SIMULATION_REAL.name().toLowerCase()))
      {
         return SIMULATION_REAL;
      }
      else
      {
         return HARDWARE;
      }
   }
}
