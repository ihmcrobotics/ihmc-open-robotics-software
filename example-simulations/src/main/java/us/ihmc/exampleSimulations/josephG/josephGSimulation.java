package us.ihmc.exampleSimulations.josephG;

import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;

public class josephGSimulation
{
   public static final double DT = 0.001;
   private SimulationConstructionSet sim;
   
   public josephGSimulation()
   {
      josephGRobot robot = new josephGRobot();
      robot.setController(new josephGController(robot));
      
      SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters();
      parameters.setDataBufferSize(32000);
      
      sim = new SimulationConstructionSet(robot, parameters);
      sim.setDT(DT, 20);
      sim.setGroundVisible(true);
      sim.setCameraPosition(0, -9.0, 0.6);
      sim.setCameraFix(0.0, 0.0, 0.70);
      
      sim.setSimulateDuration(60.0);
      
      Thread myThread = new Thread(sim);
      myThread.start();
   }
  
   public static void main(String[] args)
   {
      new josephGSimulation();
   }
}
