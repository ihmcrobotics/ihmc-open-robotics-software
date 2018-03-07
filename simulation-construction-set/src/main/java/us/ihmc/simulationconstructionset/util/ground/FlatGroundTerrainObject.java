package us.ihmc.simulationconstructionset.util.ground;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.euclid.geometry.Box3D;
import us.ihmc.euclid.geometry.Cylinder3D;
import us.ihmc.euclid.geometry.Shape3D;
import us.ihmc.graphicsDescription.Graphics3DObject;

public class FlatGroundTerrainObject extends FlatGroundProfile implements TerrainObject3D
{
   private static double groundSize = 20.0;
   private static double groundThickness = 0.03;
   
   private final Graphics3DObject groundGraphics;
   
   private ArrayList<Shape3D> simpleShapes = new ArrayList<>();
   
   public FlatGroundTerrainObject()
   {
      groundGraphics = new Graphics3DObject();
      groundGraphics.translate(0.0, 0.0, -groundThickness);
      groundGraphics.addCube(groundSize, groundSize, groundThickness);
      
      Box3D boxShape = new Box3D(groundSize, groundSize, groundThickness);
      boxShape.appendTranslation(0.0, 0.0, -groundThickness);  
      this.simpleShapes.add(boxShape);
   }
   
   @Override
   public Graphics3DObject getLinkGraphics()
   {
      return groundGraphics;
   }

   @Override
   public List<? extends Shape3D> getSimpleShapes()
   {
      return simpleShapes;
   }
}
