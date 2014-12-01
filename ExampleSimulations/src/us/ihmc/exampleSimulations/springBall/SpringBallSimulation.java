package us.ihmc.exampleSimulations.springBall;

import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.LinearGroundContactModel;

public class SpringBallSimulation
{
   private SimulationConstructionSet sim;

   public SpringBallSimulation()
   {
      SpringBallRobot springBall = new SpringBallRobot();


      // sim.setGroundVisible(false);
      springBall.setGroundContactModel(new LinearGroundContactModel(springBall, 400.0, 10.0, 80.0, 100.0, springBall.getRobotsYoVariableRegistry()));

      SpringBallController controller = new SpringBallController(springBall, "springBallController");
      springBall.setController(controller);

      sim = new SimulationConstructionSet(springBall);

      sim.setCameraTracking(false, false, false, false);
      sim.setCameraDolly(false, false, false, false);

      // sim.setCameraPosition(1.0,1.0,0.5);
      // sim.setCameraFix(0.0,0.0,0.8);

      // sim.setCameraTrackingVars("ef_track00_x", "ef_track00_y", "ef_track00_z");

      sim.setupEntryBox("offset_spike");
      sim.setupEntryBox("amp_spike");
      sim.setupEntryBox("freq_spike");
      sim.setupEntryBox("k_spike");
      sim.setupEntryBox("b_spike");

      sim.setDT(0.002, 5);
      sim.setFastSimulate(true);
      Thread myThread = new Thread(sim);
      myThread.start();
   }

   public static void main(String[] args)
   {
	   
      new SpringBallSimulation();
   
   }
}
