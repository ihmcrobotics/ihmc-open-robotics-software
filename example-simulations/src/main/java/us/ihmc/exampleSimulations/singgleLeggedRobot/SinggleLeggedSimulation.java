package us.ihmc.exampleSimulations.singgleLeggedRobot;

import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;

public class SinggleLeggedSimulation
{
   public static final double DT = 0.005;
   private SimulationConstructionSet sim;

   public SinggleLeggedSimulation()
   {
      SinggleLeggedRobot robot = new SinggleLeggedRobot();
//            robot.setController(new SinggleLeggedPIDController(robot)); 
//            robot.setController(new SinggleLeggedIDController(robot)); 
      robot.setController(new SinggleLeggedMPCController(robot));

      SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters();
      parameters.setDataBufferSize(32000);

      sim = new SimulationConstructionSet(robot, parameters);
      sim.setDT(DT, 1);
      sim.setGroundVisible(true);
      sim.setCameraPosition(0.0, -9.0, 0.6);
      sim.setCameraFix(0.0, 0.0, 0.70);
      sim.setSimulateDuration(10.0);
      sim.setFastSimulate(true);

      Thread myThread = new Thread(sim);
      myThread.start();
   }

   public static void main(String[] args)
   {
      new SinggleLeggedSimulation();
   }
 
   public double getSimTime()
   {
      return sim.getTime();
   }
   
}
