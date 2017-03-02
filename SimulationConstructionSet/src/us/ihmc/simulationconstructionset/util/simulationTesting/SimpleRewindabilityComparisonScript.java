package us.ihmc.simulationconstructionset.util.simulationTesting;

import us.ihmc.simulationconstructionset.SimulationConstructionSet;

public class SimpleRewindabilityComparisonScript implements SimulationComparisonScript
{
   private final boolean DEBUG = false;
   
   private final int nTicksInitial;
   private final int nTicksCompare;
   private final int nTicksFinal;

   public SimpleRewindabilityComparisonScript(int nTicksInitial, int nTicksCompare, int nTicksFinal)
   {
      this.nTicksInitial = nTicksInitial;
      this.nTicksCompare = nTicksCompare;
      this.nTicksFinal = nTicksFinal;
   }

   @Override
   public void doInitialAction(SimulationConstructionSet scs0, SimulationConstructionSet scs1)
   {
      if (scs0.getRecordFreq() != scs1.getRecordFreq())
      {
         throw new RuntimeException("scs0.getRecordFreq() != scs1.getRecordFreq()");
      }
      
      if (nTicksInitial % scs0.getRecordFreq() != 0)
      {
         throw new RuntimeException("nTicksInitial must be divisible by record frequency!");
      }
      
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
      if (scs0.getRecordFreq() != scs1.getRecordFreq())
      {
         throw new RuntimeException("scs0.getRecordFreq() != scs1.getRecordFreq()");
      }
      
      if (nTicksCompare % scs0.getRecordFreq() != 0)
      {
         throw new RuntimeException("nTicksCompare must be divisible by record frequency!");
      }
      
      if (nTicksFinal % scs0.getRecordFreq() != 0)
      {
         throw new RuntimeException("nTicksFinal must be divisible by record frequency!");
      }
      
      SimulationConstructionSet[] scss = {scs0, scs1};
      
      if (DEBUG) System.err.println("Setting In Points");
      for (SimulationConstructionSet scs : scss)
      {
         scs.setInPoint();
      }

      if (DEBUG) System.err.println("Ticking Simulation 0");

      scs0.simulate(nTicksCompare);
      waitForSimulationToFinish(scs0);


      int index = 0;
      for (SimulationConstructionSet scs : scss)
      {
         if (DEBUG) System.err.println("Going to InPoint And Simulating on " + index++);

         scs.gotoInPointNow();
         scs.simulate(nTicksFinal);
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
