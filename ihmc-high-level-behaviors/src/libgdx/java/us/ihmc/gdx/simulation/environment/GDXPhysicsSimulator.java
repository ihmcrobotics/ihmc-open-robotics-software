package us.ihmc.gdx.simulation.environment;

import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.physics.ExperimentalPhysicsEngine;
import us.ihmc.simulationToolkit.physicsEngine.ExperimentalSimulation;
import us.ihmc.simulationconstructionset.Robot;

public class GDXPhysicsSimulator
{
   private final ExperimentalPhysicsEngine physicsEngine = new ExperimentalPhysicsEngine();

   public GDXPhysicsSimulator()
   {
//      ExperimentalSimulation experimentalSimulation = new ExperimentalSimulation(allSimulatedRobotList.toArray(new Robot[0]),
//                                                                                 simulationConstructionSetParameters.getDataBufferSize());
//      experimentalSimulation.setGravity(new Vector3D(0.0, 0.0, -Math.abs(gravity.get())));
   }
}
