

package us.ihmc.simulationconstructionset;

import us.ihmc.jMonkeyEngineToolkit.GroundProfile3D;


public interface MovingGroundContactModel
{
   // public abstract void initGroundContact();
   public abstract void doGroundContact();

   public abstract void setGroundProfile(MovingGroundProfile profile);
   public abstract GroundProfile3D getGroundProfile3D();
}
