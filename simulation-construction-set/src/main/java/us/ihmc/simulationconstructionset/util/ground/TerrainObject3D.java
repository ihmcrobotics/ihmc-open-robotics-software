package us.ihmc.simulationconstructionset.util.ground;


import java.util.List;

import us.ihmc.euclid.geometry.Shape3D;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.jMonkeyEngineToolkit.GroundProfile3D;


public interface TerrainObject3D extends GroundProfile3D
{
   public abstract Graphics3DObject getLinkGraphics();
   public abstract List<? extends Shape3D> getSimpleShapes();
}