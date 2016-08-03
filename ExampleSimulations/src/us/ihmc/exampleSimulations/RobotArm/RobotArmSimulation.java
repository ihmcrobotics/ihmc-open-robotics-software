package us.ihmc.exampleSimulations.RobotArm;


import us.ihmc.simulationconstructionset.SimulationConstructionSet;

public class RobotArmSimulation
{
   private SimulationConstructionSet sim;
   public RobotArmSimulation()
   {
      RobotArm robotArm = new RobotArm();

      //buildingPendulumRobot.setController(new BuildingPendulumController(buildingPendulumRobot));

      sim = new SimulationConstructionSet(robotArm);
     // sim = new SimulationConstructionSet();
      sim.setGroundVisible(false);
      sim.setDT(0.00001, 400);
      sim.setCameraPosition(0, -40.0, 2.0);

      Thread myThread = new Thread(sim);
      myThread.start();
   }

   public static void main(String[] args)
   {
      new RobotArmSimulation();
   }
}

