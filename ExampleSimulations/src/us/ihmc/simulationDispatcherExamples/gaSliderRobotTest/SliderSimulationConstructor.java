package us.ihmc.simulationDispatcherExamples.gaSliderRobotTest;

import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.simulationconstructionset.Simulation;
import us.ihmc.simulationconstructionset.SimulationConstructor;
import us.ihmc.simulationconstructionset.SimulationDoneCriterion;


public class SliderSimulationConstructor implements SimulationConstructor, SimulationDoneCriterion
{
   private static final long serialVersionUID = -7181149359828581923L;
   
   private DoubleYoVariable t, q_joint1, q_joint2, qd_joint1, qd_joint2;

   public SliderSimulationConstructor()
   {
   }

   public Simulation constructSimulation(String[] structuralParameterNames, double[] structuralParameterValues)
   {
      SliderRobot slider = new SliderRobot(structuralParameterNames, structuralParameterValues);
      SliderController controller = new SliderController(slider);
      slider.setController(controller);

      Simulation ret = new Simulation(slider, 2);
      ret.setDT(0.0004, 25);
      ret.setSimulateDoneCriterion(this);

      t = (DoubleYoVariable)ret.getVariable("t");
      q_joint1 = (DoubleYoVariable)ret.getVariable("q_joint1");
      qd_joint1 = (DoubleYoVariable)ret.getVariable("qd_joint1");
      q_joint2 = (DoubleYoVariable)ret.getVariable("q_joint2");
      qd_joint2 = (DoubleYoVariable)ret.getVariable("qd_joint2");

      return ret;
   }

   public boolean isSimulationDone()
   {
      // if ((t.val > 10.0))
      if ((t.getDoubleValue() > 10.0) || (Math.abs(q_joint1.getDoubleValue()) > 10.0) || (Math.abs(q_joint2.getDoubleValue()) > 1.0))
      {
         return true;
      }

      else
         return false;
   }

   public void doActionAfterSimulationStateInitialized(Simulation simulation)
   {
      // TODO Auto-generated method stub
      
   }
}
