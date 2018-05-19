package us.ihmc.simulationConstructionSetTools.util.perturbance;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;


public interface ForcePerturbable
{
   public abstract void setForcePerturbance(Vector3DReadOnly force, double duration);
   public abstract void resetPerturbanceForceIfNecessary(); // need to do it this way because of thread issues.
   public abstract Point3D getForcePerturbanceApplicationPoint();
}
