package us.ihmc.exampleSimulations.simplePendulum;

import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizer;
import us.ihmc.scs2.simulation.SimulationSession;
import us.ihmc.scs2.simulation.SimulationSessionControls;

public class SimplePendulumSimulation
{

   public static final double DT = 0.001;
   public static final int BUFFER_SIZE = 12000;

   public SimplePendulumSimulation()
   {
      SimplePendulumRobotDefinition robotDefinition = new SimplePendulumRobotDefinition();

      SimulationSession simulationSession = new SimulationSession();
      simulationSession.addRobot(robotDefinition);

      SimulationSessionControls simulationSessionControls = simulationSession.getSimulationSessionControls();
      simulationSessionControls.initializeBufferSize(BUFFER_SIZE);

      // Stop simulating once the buffer has been filled up once, instead of wrapping around and overwriting old data.
      // NOTE: addExternalTerminalCondition() only takes effect for simulate() calls made through this
      // SimulationSessionControls object - the visualizer's own Play/Simulate button bypasses it entirely
      // (it sets the session mode directly through the messager, with no terminal-condition awareness).
      // addAfterPhysicsCallback() runs on every tick regardless of what triggered the run, so it works
      // whether the simulation was started from the GUI or from code.
      simulationSessionControls.addAfterPhysicsCallback(time ->
                                                        {
                                                           if (simulationSessionControls.getBufferOutPoint() >= simulationSessionControls.getBufferSize() - 1)
                                                              simulationSessionControls.pause();
                                                        });

      SessionVisualizer.startSessionVisualizer(simulationSession);
   }

   public static void main(String[] args)
   {
      new SimplePendulumSimulation();
   }
}
