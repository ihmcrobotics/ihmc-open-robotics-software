package us.ihmc.rdx.visualizers;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.Mesh;
import com.badlogic.gdx.graphics.Texture;
import com.badlogic.gdx.graphics.g3d.Material;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.RenderableProvider;
import com.badlogic.gdx.graphics.g3d.attributes.ColorAttribute;
import com.badlogic.gdx.graphics.g3d.attributes.TextureAttribute;
import com.badlogic.gdx.graphics.g3d.model.MeshPart;
import com.badlogic.gdx.graphics.g3d.utils.ModelBuilder;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import org.lwjgl.opengl.GL41;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.rdx.mesh.RDXMultiColorMeshBuilder;
import us.ihmc.rdx.tools.RDXModelInstance;
import us.ihmc.robotics.geometry.PlanarRegion;

import java.util.List;

/**
 * The origin of the model is the origin and plane normal.
 */
public class RDXPlanarRegionGraphic implements RenderableProvider
{
   private final RDXMultiColorMeshBuilder meshBuilder = new RDXMultiColorMeshBuilder();
   private final ModelBuilder modelBuilder = new ModelBuilder();
   private final Color color;
   private RDXModelInstance modelInstance;
   private final RigidBodyTransform identityTransform = new RigidBodyTransform();

   public RDXPlanarRegionGraphic(Color color)
   {
      this.color = color;
   }

   public void generateMesh(PlanarRegion planarRegion)
   {
      generateMesh(planarRegion.getConcaveHull());
   }

   public void generateMesh(List<Point2D> concaveHullInRegionFrame)
   {
      meshBuilder.addMultiLine(identityTransform, concaveHullInRegionFrame, 0.01, color, true);

      modelBuilder.begin();
      Material material = new Material();
      Texture paletteTexture = RDXMultiColorMeshBuilder.loadPaletteTexture();
      material.set(TextureAttribute.createDiffuse(paletteTexture));
      material.set(ColorAttribute.createDiffuse(new Color(0.7f, 0.7f, 0.7f, 1.0f)));

      Mesh mesh = meshBuilder.generateMesh();
      MeshPart meshPart = new MeshPart("xyz", mesh, 0, mesh.getNumIndices(), GL41.GL_TRIANGLES);
      modelBuilder.part(meshPart, material);

      modelInstance = new RDXModelInstance(modelBuilder.end()); // TODO: Clean up garbage and look into reusing the Model
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      if (modelInstance != null)
         modelInstance.getRenderables(renderables, pool);
   }

   public RDXModelInstance getModelInstance()
   {
      return modelInstance;
   }
}
