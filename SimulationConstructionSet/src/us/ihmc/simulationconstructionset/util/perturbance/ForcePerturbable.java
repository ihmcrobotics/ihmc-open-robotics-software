package us.ihmc.simulationconstructionset.util.perturbance;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;


public interface ForcePerturbable
{
   public abstract void setForcePerturbance(Vector3d force, double duration);
   public abstract void resetPerturbanceForceIfNecessary(); // need to do it this way because of thread issues.
   public abstract Point3d getForcePerturbanceApplicationPoint();
}
