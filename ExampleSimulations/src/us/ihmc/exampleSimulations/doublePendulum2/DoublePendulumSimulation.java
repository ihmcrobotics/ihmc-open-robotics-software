package us.ihmc.exampleSimulations.doublePendulum2;

import us.ihmc.simulationconstructionset.SimulationConstructionSet;

public class DoublePendulumSimulation
{
   private SimulationConstructionSet sim;
   public DoublePendulumSimulation()
   {
     DoublePendulumRobot doublePendulum = new DoublePendulumRobot("doublePendulum");
///
   //   DoublePendulumRobot doublePendulum = new DoublePendulumRobot("doublePendulum1");
     // doublePendulum[1] = new DoublePendulumRobot("doublePendulum2");
      //doublePendulum[1].addYoVariableRegistry();
     // doublePendulum[1].setLocation(5.0, 00.0, 0.0);
     // doublePendulum[0].pin1.setLimitStops(3,5,6,10);
//      doublePendulum[0].pin1.setLimitStops(1,15,15,15);

   //  doublePendulum.setController(new DoublePendulumController(doublePendulum, "doublePendulumController"));
    //  doublePendulum[1].setController(new DoublePendulumController(doublePendulum[1],"doublePendulumController"));
      sim = new SimulationConstructionSet(doublePendulum);
//      sim = new SimulationConstructionSet(doublePendulum);
//      sim.setDT(0.0001, 100);
 //    sim.setGroundVisible(false);
//      sim.setCameraPosition(0, -40.0, 2.0);
//
      Thread myThread = new Thread(sim);
      myThread.start();
   }

   public static void main(String[] args)
   {
      new DoublePendulumSimulation();
   }
}
