package us.ihmc.simulationconstructionset.util.perturbance;

import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;


public class ForcePerturbance implements DirectedPerturbance
{
   private final String name = "ForcePerturbance";
   private final YoVariableRegistry registry = new YoVariableRegistry(name);

   private final ForcePerturbable forcePerturbable;

   private final DoubleYoVariable disturbanceMagnitude = new DoubleYoVariable("disturbanceMagnitude", registry);
   private final DoubleYoVariable disturbanceDuration = new DoubleYoVariable("disturbanceDuration", registry);

   private final double ballVelocityMagnitude;

   public ForcePerturbance(ForcePerturbable forcePerturbable, double magnitude, double duration, double ballVelocityMagnitudeForViz, YoVariableRegistry parentRegistry)
   {
      this.forcePerturbable = forcePerturbable;
      this.disturbanceMagnitude.set(magnitude);
      this.disturbanceDuration.set(duration);
      this.ballVelocityMagnitude = ballVelocityMagnitudeForViz;

      if (parentRegistry != null)
         parentRegistry.addChild(registry);
   }

   @Override
   public void perturb(Vector3D direction)
   {
      Vector3D force = new Vector3D(direction);
      if (direction.lengthSquared() > 0.0)
      {
         force.normalize();
         force.scale(disturbanceMagnitude.getDoubleValue());
         forcePerturbable.setForcePerturbance(force, disturbanceDuration.getDoubleValue());
      }
   }

   @Override
   public double getBallVelocityMagnitude()
   {
      return ballVelocityMagnitude;
   }

   @Override
   public double getBallMass()
   {
      return impulse() / getBallVelocityMagnitude();
   }

   private double impulse()
   {
      return disturbanceMagnitude.getDoubleValue() * disturbanceDuration.getDoubleValue();
   }

   @Override
   public void doEveryTick()
   {
      forcePerturbable.resetPerturbanceForceIfNecessary();
   }
   
   @Override
   public String toString()
   {
      return name + ": Magnitude=" + disturbanceMagnitude.getDoubleValue() + ", Duration="+disturbanceDuration.getDoubleValue();
   }
}
