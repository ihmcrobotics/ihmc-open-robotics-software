package us.ihmc.simulationConstructionSetTools.robotController;

import us.ihmc.simulationconstructionset.SimulationConstructionSet;

public class TestMultiThreadedSimulation
{
   public TestMultiThreadedSimulation()
   {
      DoublePendulum doublePendulum = new DoublePendulum();
      SimulationConstructionSet scs = new SimulationConstructionSet(doublePendulum);

      DoublePendulumController controller = new DoublePendulumController(doublePendulum);
      
      MultiThreadedRobotController multiThreadedRobotController = new MultiThreadedRobotController("threadedController", doublePendulum, scs);
      multiThreadedRobotController.addController(controller, 10, false);
      multiThreadedRobotController.initialize();
      
      
      doublePendulum.setController(multiThreadedRobotController);
      
      scs.setDT(0.0001, 1);
      
      scs.startOnAThread();
   }

   public static void main(String[] args)
   {
      new TestMultiThreadedSimulation();
   }
}
