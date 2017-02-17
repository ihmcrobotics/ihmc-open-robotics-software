package us.ihmc.simulationconstructionset.util.perturbance;

import us.ihmc.euclid.tuple3D.Vector3D;

public interface DirectedPerturbance
{
   public abstract void perturb(Vector3D direction);
   
   public abstract double getBallMass();
   
   public abstract double getBallVelocityMagnitude();

   public abstract void doEveryTick();
}
