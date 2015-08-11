package us.ihmc.simulationDispatcherExamples.springFlamingoDispatcher;

import us.ihmc.simulationconstructionset.Simulation;
import us.ihmc.simulationconstructionset.SimulationConstructor;
import us.ihmc.simulationconstructionset.SimulationDoneCriterion;
import us.ihmc.simulationconstructionset.util.LinearGroundContactModel;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;

public class SpringFlamingoForDispatchSimulationConstructor implements SimulationConstructor, SimulationDoneCriterion
{
   private static final long serialVersionUID = 442059836887553232L;
   private DoubleYoVariable t;
   private int dataBufferSize = 2;
   private SimulationDoneCriterion doneCriterion = this;

   public SpringFlamingoForDispatchSimulationConstructor()
   {
   }

   public SpringFlamingoForDispatchSimulationConstructor(int dataBufferSize, SimulationDoneCriterion criterion)
   {
      this.dataBufferSize = dataBufferSize;
      this.doneCriterion = criterion;
   }

   public Simulation constructSimulation(String[] structuralParameterNames, double[] structuralParameterValues)
   {
      SpringFlamingoForDispatchRobot springFlamingo = new SpringFlamingoForDispatchRobot();
      SpringFlamingoForDispatchController controller = new SpringFlamingoForDispatchController(springFlamingo);
      springFlamingo.setController(controller);
      YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
      springFlamingo.setGroundContactModel(new LinearGroundContactModel(springFlamingo, registry));

      // springFlamingo.setGroundContactModel(new CollisionGroundContactModel(this, 0.1, 0.7));

      Simulation ret = new Simulation(springFlamingo, dataBufferSize);
      ret.setDT(0.0004, 25);
      
      if (doneCriterion != null)
      {
         ret.setSimulateDoneCriterion(doneCriterion);
      }
      
      t = (DoubleYoVariable)ret.getVariable("t");

      return ret;
   }

   public boolean isSimulationDone()
   {
      if (t.getDoubleValue() > 60.0)
         return true;
      else
         return false;
   }

   public void doActionAfterSimulationStateInitialized(Simulation simulation)
   {
      // TODO Auto-generated method stub
      
   }
}
