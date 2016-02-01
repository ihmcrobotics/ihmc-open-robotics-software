package us.ihmc.simulationconstructionset.util.perturbance;

import javax.vecmath.Vector3d;

public interface DirectedPerturbance
{
   public abstract void perturb(Vector3d direction);
   
   public abstract double getBallMass();
   
   public abstract double getBallVelocityMagnitude();

   public abstract void doEveryTick();
}
