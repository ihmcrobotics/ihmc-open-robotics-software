package us.ihmc.simulationconstructionset.util.ground;

import us.ihmc.graphics3DAdapter.GroundProfile3D;
import us.ihmc.graphicsDescription.Graphics3DObject;


public interface TerrainObject3D extends GroundProfile3D
{
   public abstract Graphics3DObject getLinkGraphics();
}