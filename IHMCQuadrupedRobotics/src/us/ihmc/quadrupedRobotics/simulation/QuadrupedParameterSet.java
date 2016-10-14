package us.ihmc.quadrupedRobotics.simulation;

public enum QuadrupedParameterSet
{
   SIMULATION_IDEAL, SIMULATION_REAL, HARDWARE;
   
   private final String parametersPath;
   
   private QuadrupedParameterSet()
   {
      parametersPath = "parameters/" + name().toLowerCase() + ".param";
   }
   
   public String getPath()
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
