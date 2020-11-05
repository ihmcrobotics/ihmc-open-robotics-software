package us.ihmc.simulationToolkit.physicsEngine;

import java.util.List;
import java.util.stream.Collectors;

import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.MultiBodySystemReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyReadOnly;
import us.ihmc.mecano.spatial.interfaces.SpatialInertiaReadOnly;
import us.ihmc.robotics.physics.ExperimentalPhysicsEngine;
import us.ihmc.robotics.physics.MultiBodySystemStateReader;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class SimulationEnergyStatistics
{
   public static void setupSimulationEnergyStatistics(Vector3DReadOnly gravity, ExperimentalPhysicsEngine physicsEngine)
   {
      YoRegistry registry = new YoRegistry(SimulationEnergyStatistics.class.getSimpleName());
      physicsEngine.getPhysicsEngineRegistry().addChild(registry);

      for (String robotName : physicsEngine.getRobotNames())
      {
         physicsEngine.addRobotPhysicsOutputStateReader(robotName, new RobotEnergyStatistics(robotName, gravity, registry));
      }
   }

   private static class RobotEnergyStatistics implements MultiBodySystemStateReader
   {
      private final YoDouble kineticEnergy;
      private final YoDouble potentialEnergy;
      private final YoDouble orbitalEnergy;
      private final Vector3DReadOnly gravity;
      private List<? extends RigidBodyReadOnly> allRigidBodies;

      public RobotEnergyStatistics(String robotName, Vector3DReadOnly gravity, YoRegistry registry)
      {
         this.gravity = gravity;
         kineticEnergy = new YoDouble(robotName + "KineticEnergy", registry);
         potentialEnergy = new YoDouble(robotName + "PotentialEnergy", registry);
         orbitalEnergy = new YoDouble(robotName + "OrbitalEnergy", registry);
      }

      @Override
      public void setMultiBodySystem(MultiBodySystemReadOnly multiBodySystem)
      {
         allRigidBodies = multiBodySystem.getRootBody().subtreeStream().filter(body -> !body.isRootBody()).collect(Collectors.toList());
      }

      @Override
      public void read()
      {
         double kinetic = 0.0;
         double potential = 0.0;

         for (RigidBodyReadOnly rigidBody : allRigidBodies)
         {
            SpatialInertiaReadOnly inertia = rigidBody.getInertia();
            MovingReferenceFrame bodyFixedFrame = rigidBody.getBodyFixedFrame();

            kinetic += inertia.computeKineticCoEnergy(bodyFixedFrame.getTwistOfFrame());
            potential += -inertia.getMass() * gravity.dot(bodyFixedFrame.getTransformToRoot().getTranslation());
         }

         kineticEnergy.set(kinetic);
         potentialEnergy.set(potential);
         orbitalEnergy.set(kinetic + potential);
      }
   }
}
