package us.ihmc.simulationconstructionset.simulationDispatcher.client;

import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.YoVariable;
import us.ihmc.simulationconstructionset.Simulation;
import us.ihmc.simulationconstructionset.SimulationConstructor;
import us.ihmc.simulationconstructionset.SimulationDoneListener;
import us.ihmc.simulationconstructionset.UnreasonableAccelerationException;
import us.ihmc.simulationconstructionset.simulationDispatcher.interfaces.RemoteSimulationDescription;

public class DispatchedSimulationDescription implements RemoteSimulationDescription, SimulationDoneListener
{
   private static final long serialVersionUID = 5987329050837257865L;
   private DoubleYoVariable time;
   private Simulation simulation;

   private String[] inputStateVariableNames, outputStateVariableNames;
   private YoVariable<?>[] inputStateVariables, outputStateVariables;

   private SimulationConstructor constructor;
   private boolean isSimulationDone;

   public DispatchedSimulationDescription(SimulationConstructor constructor, String[] inputStateVarNames, String[] outputStateVarNames)
   {
      // System.out.println("Creating Test Description");
      this.constructor = constructor;

      this.inputStateVariableNames = inputStateVarNames;
      this.outputStateVariableNames = outputStateVarNames;

      if (inputStateVarNames != null)
         inputStateVariables = new YoVariable[inputStateVarNames.length];
      if (outputStateVarNames != null)
         outputStateVariables = new YoVariable[outputStateVarNames.length];

      this.isSimulationDone = false;
   }


   @Override
   public void createSimulation(String[] structuralParameterNames, double[] structuralParameterValues)
   {
      System.out.println("In Create Simulation");

      // waitForSimulationToFinish();

      simulation = constructor.constructSimulation(structuralParameterNames, structuralParameterValues);
      simulation.addSimulateDoneListener(this);

      time = (DoubleYoVariable) simulation.getVariable("t");

      if (inputStateVariableNames != null)
      {
         for (int i = 0; i < inputStateVariables.length; i++)
         {
            System.out.println("Getting var: " + inputStateVariableNames[i]);
            inputStateVariables[i] = simulation.getVariable(inputStateVariableNames[i]);
         }
      }

      if (outputStateVariableNames != null)
      {
         for (int i = 0; i < outputStateVariables.length; i++)
         {
            System.out.println("Getting var: " + outputStateVariableNames[i]);
            outputStateVariables[i] = simulation.getVariable(outputStateVariableNames[i]);
         }
      }
   }

   @Override
   public void destroySimulation()
   {
      System.out.println("Destroying Simulation!");
      this.constructor = null;
      this.simulation = null;
      this.time = null;
      this.outputStateVariableNames = null;
      this.outputStateVariables = null;
      this.inputStateVariableNames = null;
      this.inputStateVariables = null;
   }

/*   private void waitForSimulationToFinish()
   {
     if(!this.isSimulationDone())
     {
       System.out.println("Waiting for previous simulation to finish..."); System.out.flush();

       while(!this.isSimulationDone())
       {
         try{Thread.sleep(100);}
         catch(InterruptedException e){}
       }
       System.out.println("Done waiting for previous simulation to finish..."); System.out.flush();
     }
   }
*/


   @Override
   public void setSimulationState(Object state)
   {
      if (inputStateVariables == null)
      {
         System.out.println("Can't set state! Was given null for state var names!!");

         return;
      }

      double[] inputStateVals = (double[]) state;

//      System.out.println("Setting system state to : " + inputStateVals);

      for (int i = 0; i < inputStateVariables.length; i++)
      {
         YoVariable<?> var = inputStateVariables[i];
         if (var != null)
         {
            var.setValueFromDouble(inputStateVals[i]);
//            System.out.println(var.getName() + ": " + var.getValueAsDouble());
         }
      }
      
      constructor.doActionAfterSimulationStateInitialized(simulation);
   }

   private void printState()
   {
      for (int i = 0; i < outputStateVariables.length; i++)
      {
         YoVariable<?> var = outputStateVariables[i];
         if (var != null)
         {
            System.out.println(var.getName() + ": " + var.getValueAsDouble());
         }
      }
   }

   @Override
   public void startSimulation()
   {
      this.isSimulationDone = false;
      System.out.println("Before Running t = " + time.getDoubleValue());

      printState();

      Thread anim = new Thread("Simulation Anim")
      {
         @Override
         public void run()    // throws Exception
         {
            try
            {
               simulation.simulate(1000.0);    // At most simulate for 1000.0 seconds, even if it doesn't finish...
               isSimulationDone = true;
            }

            catch (UnreasonableAccelerationException e)
            {
               System.out.println("Simulation Crashed with Unreasonable Acceleration");    // + e.getMessage());
               isSimulationDone = true;

               // e.printStackTrace();
               // throw e;
            }
            catch (Exception e)
            {
               System.out.println("Simulation went wrong!!!\n" + e.getMessage());
               isSimulationDone = true;
            }
         }
      };

      anim.start();

      System.out.println("That Sim is now running on a new thread.");
   }

   @Override
   public boolean isSimulationDone()
   {
      return isSimulationDone;
   }

   @Override
   public Object getSimulationState()
   {
//    System.out.println("Getting Simulation State!"); System.out.flush();
      if (outputStateVariables == null)
      {
         System.out.println("Output State vars is null!!");

         return null;
      }

      double[] ret = new double[outputStateVariables.length];

      for (int i = 0; i < ret.length; i++)
      {
         YoVariable<?> var = outputStateVariables[i];
         if (var != null)
            ret[i] = outputStateVariables[i].getValueAsDouble();
         else
            ret[i] = 0.0;
      }

      return ret;
   }

   @Override
   public Object getSimulationData()
   {
      return simulation.getDataBuffer();
   }

   @Override
   public void simulationDone()
   {
      System.out.println("Simulation Done!!!");
      System.out.println("After Running t = " + time.getDoubleValue());
      printState();

      isSimulationDone = true;
   }


   @Override
   public void simulationDoneWithException(Throwable throwable)
   {
      System.out.println("Simulation Done with Exception!!!");
      System.out.println("After Running t = " + time.getDoubleValue());
      printState();

      isSimulationDone = true;
   }

}
