package us.ihmc.simulationconstructionset.util.ground;

import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.jMonkeyEngineToolkit.GroundProfile3D;


public interface TerrainObject3D extends GroundProfile3D
{
   public abstract Graphics3DObject getLinkGraphics();
}