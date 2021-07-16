package us.ihmc.gdx.simulation.environment;

import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.physics.ExperimentalPhysicsEngine;
import us.ihmc.scs2.simulation.SimulationSession;
import us.ihmc.scs2.simulation.physicsEngine.PhysicsEngine;
import us.ihmc.simulationToolkit.physicsEngine.ExperimentalSimulation;
import us.ihmc.simulationconstructionset.Robot;

public class GDXPhysicsSimulator
{
   private final SimulationSession simulationSession;
   //   private final ExperimentalPhysicsEngine physicsEngine = new ExperimentalPhysicsEngine();
//   private final PhysicsEngine physicsEngine = new PhysicsEngine()

   public GDXPhysicsSimulator()
   {
      simulationSession = new SimulationSession();

//      simulationSession.

      //      ExperimentalSimulation experimentalSimulation = new ExperimentalSimulation(allSimulatedRobotList.toArray(new Robot[0]),
//                                                                                 simulationConstructionSetParameters.getDataBufferSize());
//      experimentalSimulation.setGravity(new Vector3D(0.0, 0.0, -Math.abs(gravity.get())));
   }
}
