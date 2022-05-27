package us.ihmc.gdx.ui.graphics;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.Material;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.attributes.TextureAttribute;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.gdx.mesh.GDXMultiColorMeshBuilder;
import us.ihmc.gdx.ui.gizmo.DynamicGDXModel;

public class GDXArrowGraphic
{
   private DynamicGDXModel dynamicArrowModel;
   private double arrowBodyLength;
   private double arrowBodyRadius;
   private double arrowHeadRadius;
   private double arrowHeadLength;
   private double arrowLengthRatio = 0.431;
   private double arrowHeadBodyLengthRatio = 0.480;
   private double arrowHeadBodyRadiusRatio = 0.5;

   public GDXArrowGraphic(Color color)
   {
      dynamicArrowModel = new DynamicGDXModel();
      dynamicArrowModel.setMesh(meshBuilder ->
      {
         // Euclid cylinders are defined from the center, but mesh builder defines them from the bottom
         meshBuilder.addCylinder(arrowBodyLength, arrowBodyRadius, new Point3D(), color);
         meshBuilder.addCone(arrowHeadLength, arrowHeadRadius, new Point3D(0.0, 0.0, arrowBodyLength), color);
      });
      Material material = new Material();
      material.set(TextureAttribute.createDiffuse(GDXMultiColorMeshBuilder.loadPaletteTexture()));
      dynamicArrowModel.setMaterial(material);
   }

   public void update(double arrowBodyLength)
   {
      this.arrowBodyLength = arrowBodyLength;
      arrowLengthRatio = 0.15;
      arrowHeadBodyLengthRatio = 0.480;
      arrowHeadBodyRadiusRatio = 2.0;

//      arrowBodyRadius = arrowLengthRatio * this.arrowBodyLength;
      arrowBodyRadius = 0.02;
      arrowHeadRadius = arrowHeadBodyRadiusRatio * arrowBodyRadius;
//      arrowHeadLength = arrowHeadBodyLengthRatio * this.arrowBodyLength;
      arrowHeadLength = 0.08;
      dynamicArrowModel.invalidateMesh();
   }

   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      dynamicArrowModel.getOrCreateModelInstance().getRenderables(renderables, pool);
   }

   public DynamicGDXModel getDynamicModel()
   {
      return dynamicArrowModel;
   }
}
