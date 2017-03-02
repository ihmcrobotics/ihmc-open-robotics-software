package us.ihmc.simulationconstructionset.util.simulationTesting;

import java.io.File;
import java.util.ArrayList;

import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.simulationRunner.StateFileComparer;
import us.ihmc.simulationconstructionset.util.simulationRunner.VariableDifference;

public class StateFileSimulationComparer implements SimulationComparer
{
   private final double maxPercentDiff;
   private ArrayList<VariableDifference> variableDifferences;
   private ArrayList<String> exceptions = new ArrayList<String>();

   public StateFileSimulationComparer(double maxPercentDiff)
   {
      this.maxPercentDiff = maxPercentDiff;
   }

   @Override
   public boolean compare(SimulationConstructionSet scs0, SimulationConstructionSet scs1)
   {
      ArrayList<SimulationConstructionSet> simulationConstructionSets = new ArrayList<SimulationConstructionSet>();
      simulationConstructionSets.add(scs0);
      simulationConstructionSets.add(scs1);

      // write state files
      ArrayList<String> stateFileNames = new ArrayList<String>();
      for (int i = 0; i < simulationConstructionSets.size(); i++)
      {
         stateFileNames.add("scs" + i);
         simulationConstructionSets.get(i).writeState(stateFileNames.get(i));
      }

      variableDifferences = StateFileComparer.percentualCompareStateFiles(stateFileNames.get(0), stateFileNames.get(1), maxPercentDiff, exceptions);

      // remove state files
      for (String stateFileName : stateFileNames)
      {
         File file = new File(stateFileName);
         file.delete();
      }

      // return
      return variableDifferences.isEmpty();
   }
   
   public void addException(String exception)
   {
      exceptions.add(exception);
   }

   public ArrayList<VariableDifference> getVariableDifferences()
   {
      return variableDifferences;
   }
   
   @Override
   public String toString()
   {
      StringBuffer stringBuffer = new StringBuffer();
      
      if (variableDifferences == null)
      {
         return "No comparison done - sim probably crashed";
      }
      
      for (VariableDifference variableDifference : variableDifferences)
      {
         if (variableDifference == null)
         {
            System.err.println("StateFileSimulationComparer.toString() variableDifference == null. Seems odd...");
         }
         else
         {
            stringBuffer.append(variableDifference.toString());
            stringBuffer.append("\n");
         }
      }
      return stringBuffer.toString();
   }
}
