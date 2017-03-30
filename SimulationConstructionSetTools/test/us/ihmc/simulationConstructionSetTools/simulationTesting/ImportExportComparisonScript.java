package us.ihmc.simulationConstructionSetTools.simulationTesting;

import java.io.File;

import us.ihmc.simulationconstructionset.SimulationConstructionSet;

public class ImportExportComparisonScript implements SimulationComparisonScript
{
   private final int nTicksInitial;
   private final int nTicksCompare;
   private final int nTicksFinal;

   public ImportExportComparisonScript(int nTicksInitial, int nTicksCompare, int nTicksFinal)
   {
      this.nTicksInitial = nTicksInitial;
      this.nTicksCompare = nTicksCompare;
      this.nTicksFinal = nTicksFinal;
   }

   @Override
   public void doInitialAction(SimulationConstructionSet scs0, SimulationConstructionSet scs1)
   {
      SimulationConstructionSet[] scss = {scs0, scs1};

      for (SimulationConstructionSet scs : scss)
      {
         scs.simulate(nTicksInitial);
      }
      for (SimulationConstructionSet scs : scss)
      {
         waitForSimulationToFinish(scs);
      }
   }

   @Override
   public void doFinalAction(SimulationConstructionSet scs0, SimulationConstructionSet scs1)
   {
      SimulationConstructionSet[] scss = {scs0, scs1};

      for (SimulationConstructionSet scs : scss)
      {
         scs.setInPoint();
         scs.simulate(nTicksCompare);
      }
      for (SimulationConstructionSet scs : scss)
      {
         waitForSimulationToFinish(scs);
      }

      File stateFile = new File("stateFile");
      scs0.writeState(stateFile);
      scs0.gotoInPointNow();
      scs0.simulate(1);

      scs0.writeState(stateFile);
      stateFile.delete();

      for (SimulationConstructionSet scs : scss)
      {
         scs.simulate(nTicksFinal);
      }
      for (SimulationConstructionSet scs : scss)
      {
         waitForSimulationToFinish(scs);
      }
   }

   private void waitForSimulationToFinish(SimulationConstructionSet scs)
   {
      while (scs.isSimulating())
      {
         try
         {
            Thread.sleep(10);
         }
         catch (InterruptedException e)
         {
            e.printStackTrace();
         }
      }
   }
}
