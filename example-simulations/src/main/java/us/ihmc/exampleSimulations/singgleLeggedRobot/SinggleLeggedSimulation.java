package us.ihmc.exampleSimulations.singgleLeggedRobot;

import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;

public class SinggleLeggedSimulation
{
   public static final double DT = 0.001;
   private SimulationConstructionSet sim;

   public SinggleLeggedSimulation()
   {
      SinggleLeggedRobot robot = new SinggleLeggedRobot();
      robot.setController(new SinggleLeggedPIDController(robot)); //TODO: Making PID Controller
//      robot.setController(new SinggleLeggedIDController(robot)); //TODO: Making ID Controller
//      robot.setController(new SinggleLeggedMPCController(robot)); //TODO: Making Model Predictive Controller

      SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters();
      parameters.setDataBufferSize(32000);

      sim = new SimulationConstructionSet(robot, parameters);
      sim.setDT(DT, 10);
      sim.setGroundVisible(true);
      sim.setCameraPosition(0.0, -9.0, 0.6);
      sim.setCameraFix(0.0, 0.0, 0.70);
      sim.setSimulateDuration(10);
      sim.setFastSimulate(false);
      
      Thread myThread = new Thread(sim);
      myThread.start();
   }

   public static void main(String[] args)
   {
      new SinggleLeggedSimulation();
   }
   
   // TODO: Make ground and Fix the base to the ground 
}
