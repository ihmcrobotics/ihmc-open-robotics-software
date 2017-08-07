package us.ihmc.simulationConstructionSetTools.simulationTesting;

import java.io.File;
import java.net.InetAddress;
import java.util.ArrayList;

import javax.swing.JOptionPane;

import us.ihmc.robotics.controllers.ControllerFailureException;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.simulationconstructionset.util.simulationRunner.StateFileComparer;
import us.ihmc.simulationconstructionset.util.simulationRunner.VariableDifference;
import us.ihmc.tools.thread.ThreadTools;

public class NothingChangedVerifier
{
   private final String baseFilename;
   private final String newFilename;
   private final boolean writeNewBaseFile;
   
   private final SimulationConstructionSet scs;
   private final BlockingSimulationRunner blockingSimulationRunner;
   
   public NothingChangedVerifier(String runName, SimulationConstructionSet scs)
   {
      this.scs = scs;
      blockingSimulationRunner = new BlockingSimulationRunner(scs, 600.0);

      String pathName = "StateComparison";
      File dir = new File(pathName);
      if (!dir.exists())
      {
         dir.mkdir();
      }

      String computerName = getComputerName();
      baseFilename = pathName + "/" + runName + "_base." + computerName + ".state";
      newFilename = pathName + "/" + runName + "_new." + computerName + ".state";

      File baseFile = new File(baseFilename);

      writeNewBaseFile = !baseFile.exists();
      if (writeNewBaseFile)
      {
         JOptionPane.showMessageDialog(null, "Warning! " + baseFilename + " doesn't exist. Creating it.");
      }
      
      //TODO: Not sure if we still need this sleep or not.
      ThreadTools.sleep(1000); // give scs time to start, otherwise isRunning() will return false.
   }
   
   public boolean getWriteNewBaseFile()
   {
      return writeNewBaseFile;
   }
   
   public void runSimulation(double simTime) throws SimulationExceededMaximumTimeException, ControllerFailureException
   {
      if (simTime > 0.0)
         blockingSimulationRunner.simulateAndBlock(simTime);
   }
   
   public void runAndVerifySameResultsAsPreviously(double simTime, double maxPercentDifference, ArrayList<String> stringsToIgnore) throws SimulationExceededMaximumTimeException, ControllerFailureException
   {
      runSimulation(simTime);
      verifySameResultsAsPreviously(maxPercentDifference, stringsToIgnore);
   }
   
   public void verifySameResultsAsPreviously(double maxPercentDifference, ArrayList<String> stringsToIgnore)
   {     
      if (writeNewBaseFile)
      {
         scs.writeState(baseFilename);
      }
      else
      {
         scs.writeState(newFilename);
         ArrayList<VariableDifference> variableDifferences = StateFileComparer.percentualCompareStateFiles(baseFilename, newFilename, maxPercentDifference, stringsToIgnore);

         if (variableDifferences.isEmpty())
            System.out.println("Sims are the same!");

         else
         {
            String message = baseFilename + " and " + newFilename + " are not the same.";
            
            System.err.println(message + "Changed variables:\n" + VariableDifference.allVariableDifferencesToString(variableDifferences));
            JOptionPane.showMessageDialog(null, message);
            System.err.flush();
         }
      }
   }
   
   private static String getComputerName()
   {
      try
      {
         String computerName = InetAddress.getLocalHost().getHostName();
         return computerName;
      }
      catch (Exception e)
      {
         System.out.println("Exception caught ="+e.getMessage());
         return "unknown";
      }

   }

}
