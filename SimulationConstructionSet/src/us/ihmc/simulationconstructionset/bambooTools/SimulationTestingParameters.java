package us.ihmc.simulationconstructionset.bambooTools;

import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;

public class SimulationTestingParameters extends SimulationConstructionSetParameters
{
   private boolean createSCSMovies = false;
   private boolean checkNothingChangedInSimulation = false;
   private boolean keepSCSUp = false;
   
   public SimulationTestingParameters()
   {
   }
   
   public static SimulationTestingParameters createFromEnvironmentVariables()
   {
      SimulationTestingParameters parametersToReturn = new SimulationTestingParameters();
      parametersToReturn.setFromSystemProperties();
      return parametersToReturn;
   }

   public void setFromSystemProperties()
   {
      super.setFromSystemProperties();

      String property = System.getProperty("create.scs.movies");
      if (property != null)
      {
         Boolean createSCSMovies = Boolean.parseBoolean(property);
         setCreateSCSMovies(createSCSMovies);
      }

      property = System.getProperty("check.nothing.changed.in.simulation");
      if (property != null)
      {
         Boolean checkNothingChanged = Boolean.parseBoolean(property);
         setCheckNothingChangedInSimulation(checkNothingChanged);
      }
      
      property = System.getProperty("keep.scs.up");
      if (property != null)
      {
         Boolean keepSCSUp = Boolean.parseBoolean(property);
         setKeepSCSUp(keepSCSUp);
      }
      System.out.println(toString());
   }
   
   public boolean getCreateSCSMovies()
   {
      return createSCSMovies;
   }

   public void setCreateSCSMovies(boolean createSCSMovies)
   {
      this.createSCSMovies = createSCSMovies;
   }

   public boolean getCheckNothingChangedInSimulation()
   {
      return checkNothingChangedInSimulation;
   }

   public void setCheckNothingChangedInSimulation(boolean checkNothingChangedInSimulation)
   {
      this.checkNothingChangedInSimulation = checkNothingChangedInSimulation;
   }
   
   public boolean getKeepSCSUp()
   {
      return keepSCSUp;
   }

   public void setKeepSCSUp(boolean keepSCSUp)
   {
      this.keepSCSUp = keepSCSUp;
   }
   
   public String toString()
   {
      String st = super.toString();
      st += "createSCSMovies: " + createSCSMovies + "\n";    
      st += "checkNothingChangedInSimulation: " + checkNothingChangedInSimulation + "\n";    
      st += "keepSCSUp: " + keepSCSUp + "\n";    
      return st;   
   }



}
