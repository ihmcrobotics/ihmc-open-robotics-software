package us.ihmc.exampleSimulations.simpleQuadrupedRobot;

import us.ihmc.exampleSimulations.singgleLeggedRobot.SinggleLeggedSimulation;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;

public class SimpleQuadrupedSimulation
{
   public static final double DT = 0.005;
   private SimulationConstructionSet sim;

   public SimpleQuadrupedSimulation()
   {
    SimpleQuadrupedRobot robot = new SimpleQuadrupedRobot();
    
    SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters();
    parameters.setDataBufferSize(32000);

    sim = new SimulationConstructionSet(robot, parameters);
    sim.setDT(DT, 10);
    sim.setGroundVisible(true);
    sim.setCameraPosition(0.0, -9.0, 0.6);
    sim.setCameraFix(0.0, 0.0, 0.70);
    sim.setSimulateDuration(10.0);
    sim.setFastSimulate(false);

    Thread myThread = new Thread(sim);
    myThread.start();
   }
   
   public static void main(String[] args)
   {
      new SimpleQuadrupedSimulation();
   }
   
   
}
