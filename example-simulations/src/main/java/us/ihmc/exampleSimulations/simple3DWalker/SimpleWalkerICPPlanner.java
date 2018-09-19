package us.ihmc.exampleSimulations.simple3DWalker;

import java.util.ArrayList;

public class SimpleWalkerICPPlanner
{
   ArrayList<Double> CoPplan = new ArrayList();
   ArrayList<Double> ICPplan = new ArrayList<>();
   ArrayList<Double> ICPPlanOrdered = new ArrayList<>();
   double omega;
   public SimpleWalkerICPPlanner(ArrayList<Double> footstepsXvalues, double stepTime, double omega)
   {
      this.omega=omega;
      CoPplan = footstepsXvalues;

      double ICPf = CoPplan.get(CoPplan.size()-1);
      ICPplan.add(ICPf);
      for(int i=0; i<(CoPplan.size()-1);i++)
      {
         double CoPx = CoPplan.get(CoPplan.size()-2-i);
         double ICPprev = CoPx + Math.exp(-omega*stepTime)*(ICPf - CoPx);
         ICPplan.add(ICPprev);
         ICPf = ICPprev;
      }
   }

   public ArrayList<Double> getICPKnotPoints()
   {
      for(int i=0;i<ICPplan.size();i++)
      {
         double ICPcurr = ICPplan.get(ICPplan.size()-1-i);
         ICPPlanOrdered.add(ICPcurr);
      }
      return ICPPlanOrdered;
   }

   public double getICPReference(int currentStep, double timeInstate)
   {
      double ICPinit = ICPplan.get(ICPplan.size()-1-currentStep);
      double CoPinit = CoPplan.get(currentStep);
      double ICPref = Math.exp(omega*timeInstate)*(ICPinit-CoPinit)+CoPinit;
      return  ICPref;
   }
}
