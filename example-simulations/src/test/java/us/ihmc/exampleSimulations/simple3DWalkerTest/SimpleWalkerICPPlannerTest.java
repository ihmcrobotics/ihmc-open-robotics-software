package us.ihmc.exampleSimulations.simple3DWalkerTest;

import org.junit.After;
import org.junit.Assert;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.exampleSimulations.simple3DWalker.SimpleWalkerICPPlanner;
import us.ihmc.exampleSimulations.skippy.SkippySimulation.SkippyControllerMode;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.commons.thread.ThreadTools;


import java.util.ArrayList;
import java.util.List;

import static java.lang.Math.sqrt;

public class SimpleWalkerICPPlannerTest
{
   ArrayList<Double> CoPs = createEvenCoPList(0.2);
   double stepTime =0.5;
   double omega = sqrt(9.81);
   SimpleWalkerICPPlanner icpPlanner = new SimpleWalkerICPPlanner(CoPs, stepTime, omega );
   ArrayList<Double> ICPs = icpPlanner.getICPKnotPoints();

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testIfICPListIsSameSize()
   {
      Assert.assertEquals("CoP knots not equal length ICP knots", CoPs.size(),ICPs.size());
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public  void testIfICPsAreInFrontICPExceptLast()
   {
      for(int i=0; i<CoPs.size();i++)
      {
         if(i==CoPs.size()-1)
         {
            Assert.assertEquals("Last CoP should coincide with last ICP", CoPs.get(CoPs.size()-1),ICPs.get(ICPs.size()-1));
         }
         else
         {
            double currentCoP = CoPs.get(i);
            Assert.assertTrue("ICPs have to lay in front of CoPs", ICPs.get(i) >currentCoP);
         }
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testIfICPsAreBetweenAllCoPs()
   {
      for(int i =0; i<ICPs.size();i++)
      {
         Assert.assertTrue("ICPs have to lie between initial and final CoP", CoPs.get(0)<ICPs.get(i) && ICPs.get(i)<=CoPs.get(CoPs.size()-1));
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testIfICPReferenceLiesBetweenICPPoints()
   {

      for(int i=1; i<ICPs.size();i++)
      {
         double ICPrefnext = 0;
         for (int j=1;j<10;j++)
         {
            double timeInState = stepTime/j;
            double ICPref = icpPlanner.getICPReference(i-1, timeInState);
            if(j==1)
            {
               double icpNext = ICPs.get(i);
               Assert.assertEquals("Last ICP ref should coincide with next",ICPref,icpNext, 0.002);
            }
            else
            {
               Assert.assertTrue("ICP reference should be increasing between knotpoints",ICPref<ICPrefnext);
            }
            double ICPfromFormula = Math.exp(omega*timeInState)*(ICPs.get(i-1)-CoPs.get(i-1))+CoPs.get(i-1);
            Assert.assertEquals("ICP formula not correct", ICPfromFormula,ICPref,0.02);
            ICPrefnext=ICPref;
         }

      }
   }

   private ArrayList<Double> createEvenCoPList(double stepLength)
   {
      ArrayList<Double> CoPs = new ArrayList<>();
      double CoPcurr = 0;
      for(int i = 0; i<10; i++)
      {
         CoPs.add(CoPcurr);
         CoPcurr=CoPcurr+stepLength;
      }
      return CoPs;
   }
}
