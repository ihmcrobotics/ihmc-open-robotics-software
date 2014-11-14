package us.ihmc.simulationconstructionset;

import java.util.ArrayList;


public class FunctionIntegrators
{
   private FunctionIntegrator[] integrators;

   public FunctionIntegrators()
   {
   }

   public void addFunctionIntegrator(FunctionIntegrator integrator)
   {
      if (integrators == null)
      {
         integrators = new FunctionIntegrator[] {integrator};

         return;
      }

      ArrayList<FunctionIntegrator> tempIntegrators = new ArrayList<FunctionIntegrator>();

      for (int i = 0; i < integrators.length; i++)
      {
         tempIntegrators.add(integrators[i]);
      }

      tempIntegrators.add(integrator);

      integrators = new FunctionIntegrator[tempIntegrators.size()];
      tempIntegrators.toArray(integrators);
   }

   public void saveTempState()
   {
      for (int i = 0; i < integrators.length; i++)
      {
         integrators[i].saveTempState();
      }
   }

   public void doDynamics(int step)
   {
      for (int i = 0; i < integrators.length; i++)
      {
         integrators[i].doDynamics(step);
      }
   }

   public void eulerIntegrate(double dt)
   {
      for (int i = 0; i < integrators.length; i++)
      {
         integrators[i].eulerIntegrate(dt);
      }
   }

   public void restoreTempState()
   {
      for (int i = 0; i < integrators.length; i++)
      {
         integrators[i].restoreTempState();
      }
   }

   public void rungeKuttaSum(double DT)
   {
      for (int i = 0; i < integrators.length; i++)
      {
         integrators[i].rungeKuttaSum(DT);
      }
   }
}
