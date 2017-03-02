

package us.ihmc.simulationconstructionset;

import us.ihmc.jMonkeyEngineToolkit.GroundProfile3D;


public interface GroundContactModel extends java.io.Serializable
{
   public abstract void doGroundContact();

   public abstract void setGroundProfile3D(GroundProfile3D profile);
   // public abstract void setGroundProfile(MovingGroundProfile profile);
   public abstract GroundProfile3D getGroundProfile3D();
}
