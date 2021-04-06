package us.ihmc.jme;

import com.jme3.asset.AssetManager;
import com.jme3.material.Material;
import com.jme3.material.RenderState;
import com.jme3.math.ColorRGBA;
import com.jme3.renderer.queue.RenderQueue.Bucket;
import com.jme3.renderer.queue.RenderQueue.ShadowMode;
import com.jme3.scene.Geometry;
import com.jme3.scene.Mesh;
import com.jme3.scene.Node;

import javafx.scene.paint.Color;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.robotics.geometry.PlanarRegion;

public class PlanarRegionNode extends Node
{
   private static final float PLANAR_REGION_TRANSPARENCY = 0.5f;
   private PlanarRegion planarRegion;

   public PlanarRegionNode(int id, PlanarRegion planarRegion, ColorRGBA color, AssetManager assetManager)
   {
      this.planarRegion = planarRegion;
      String geometryName = getClass().getSimpleName() + id;

      Geometry regionGeometry = createRegionGeometry(planarRegion, geometryName, JMEColorConversions.toJavaFXColor(color));

      Material regionMaterial = new Material(assetManager, "Common/MatDefs/Light/Lighting.j3md");
      regionMaterial.setBoolean("UseMaterialColors",true);
      color.set(color.getRed(), color.getGreen(), color.getBlue(), PLANAR_REGION_TRANSPARENCY);
      regionMaterial.setColor("Ambient", color);
      regionMaterial.setColor("Specular", color);
      regionMaterial.setColor("Diffuse", color);
      regionMaterial.setFloat("Shininess", 64f);
      regionMaterial.getAdditionalRenderState().setBlendMode(RenderState.BlendMode.Alpha);
      regionGeometry.setMaterial(regionMaterial);
      regionGeometry.setShadowMode(ShadowMode.CastAndReceive);
      regionGeometry.setQueueBucket(Bucket.Translucent);
      attachChild(regionGeometry);
   }
   public PlanarRegion getPlanarRegion()
   {
      return planarRegion;
   }

   private Geometry createRegionGeometry(PlanarRegion planarRegion, String geometryName, Color color)
   {
      RigidBodyTransform transformToWorld = planarRegion.getTransformToWorldCopy();

      JMEMultiColorMeshBuilder meshBuilder = new JMEMultiColorMeshBuilder();
      meshBuilder.addMultiLine(transformToWorld, planarRegion.getConcaveHull(), 0.01, color, true);
      for (ConvexPolygon2D convexPolygon : planarRegion.getConvexPolygons())
      {
         meshBuilder.addPolygon(transformToWorld, convexPolygon, color);
      }

      Mesh regionMesh = meshBuilder.generateMesh();
      return new Geometry(geometryName, regionMesh);
   }
}
