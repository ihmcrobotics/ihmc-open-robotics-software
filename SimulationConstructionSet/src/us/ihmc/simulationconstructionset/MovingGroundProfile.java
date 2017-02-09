package us.ihmc.simulationconstructionset;

import javax.vecmath.Vector3d;

import us.ihmc.jMonkeyEngineToolkit.GroundProfile3D;

public interface MovingGroundProfile extends GroundProfile3D
{
   public abstract void velocityAt(double x, double y, double z, Vector3d normal);
}
