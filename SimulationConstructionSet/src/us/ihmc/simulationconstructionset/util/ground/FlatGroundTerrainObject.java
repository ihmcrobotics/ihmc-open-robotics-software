package us.ihmc.simulationconstructionset.util.ground;

import us.ihmc.graphicsDescription.Graphics3DObject;

public class FlatGroundTerrainObject extends FlatGroundProfile implements TerrainObject3D
{
   private final Graphics3DObject groundGraphics;
   
   public FlatGroundTerrainObject()
   {
      groundGraphics = new Graphics3DObject();
      groundGraphics.translate(0.0, 0.0, -0.03);
      groundGraphics.addCube(20.0, 20.0, 0.03);
   }
   
   @Override
   public Graphics3DObject getLinkGraphics()
   {
      return groundGraphics;
   }

}
