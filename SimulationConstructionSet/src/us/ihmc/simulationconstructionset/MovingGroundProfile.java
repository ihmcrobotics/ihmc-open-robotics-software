package us.ihmc.simulationconstructionset;

import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.jMonkeyEngineToolkit.GroundProfile3D;

public interface MovingGroundProfile extends GroundProfile3D
{
   public abstract void velocityAt(double x, double y, double z, Vector3D normal);
}
