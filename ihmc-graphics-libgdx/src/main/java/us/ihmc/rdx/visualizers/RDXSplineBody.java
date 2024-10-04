package us.ihmc.rdx.visualizers;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.Mesh;
import com.badlogic.gdx.graphics.Texture;
import com.badlogic.gdx.graphics.g3d.*;
import com.badlogic.gdx.graphics.g3d.attributes.BlendingAttribute;
import com.badlogic.gdx.graphics.g3d.Material;
import com.badlogic.gdx.graphics.g3d.model.MeshPart;
import com.badlogic.gdx.graphics.g3d.utils.ModelBuilder;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import static com.badlogic.gdx.graphics.g3d.utils.MeshBuilder.MAX_INDEX;

import net.mgsx.gltf.scene3d.attributes.PBRColorAttribute;
import net.mgsx.gltf.scene3d.attributes.PBRTextureAttribute;
import org.lwjgl.opengl.GL41;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.log.LogTools;
import us.ihmc.rdx.mesh.RDXMultiColorMeshBuilder;

import java.util.ArrayList;

public class RDXSplineBody implements RenderableProvider
{
   private final ModelBuilder modelBuilder = new ModelBuilder();
   private final RDXMultiColorMeshBuilder meshBuilder = new RDXMultiColorMeshBuilder();
   private ModelInstance modelInstance;
   private Model lastModel;
   private final ArrayList<Point3DReadOnly> points = new ArrayList<>();
   private static final int VERTEX_BUFFER_MARGIN = MAX_INDEX - 10;

   private Color color;
   private float lineWidth = 0.1f;
   private final Material material = new Material();

   public RDXSplineBody(float lineWidth)
   {
      this.lineWidth = lineWidth;
      Texture paletteTexture = RDXMultiColorMeshBuilder.loadPaletteTexture();
      material.set(PBRTextureAttribute.createBaseColorTexture(paletteTexture));
      material.set(PBRColorAttribute.createBaseColorFactor(new com.badlogic.gdx.graphics.Color(0.7f, 0.7f, 0.7f, 1.0f)));
   }

   public RDXSplineBody(float lineWidth, float opacity)
   {
      this.lineWidth = lineWidth;
      Texture paletteTexture = RDXMultiColorMeshBuilder.loadPaletteTexture();
      material.set(PBRTextureAttribute.createBaseColorTexture(paletteTexture));
      material.set(PBRColorAttribute.createBaseColorFactor(new com.badlogic.gdx.graphics.Color(0.7f, 0.7f, 0.7f, 1.0f)));
      material.set(new BlendingAttribute(true, GL41.GL_SRC_ALPHA, GL41.GL_ONE_MINUS_SRC_ALPHA, opacity));
   }

   public void setColor(Color color)
   {
      this.color = color;
   }

   public void generateMeshes(Point3DReadOnly startPoint, Point3DReadOnly endPoint)
   {
      meshBuilder.addLine(startPoint, endPoint, lineWidth, color);
      modelBuilder.begin();
      Mesh mesh = meshBuilder.generateMesh();
      if (mesh.getNumIndices() >= VERTEX_BUFFER_MARGIN) // check we are not inserting too many vertices in the mesh
         mesh = getDecimatedMesh(10);

      MeshPart meshPart = new MeshPart("xyz", mesh, 0, mesh.getNumIndices(), GL41.GL_TRIANGLES);
      modelBuilder.part(meshPart, material);

      if (lastModel != null)
         lastModel.dispose();

      lastModel = modelBuilder.end();
      modelInstance = new ModelInstance(lastModel);

      points.add(startPoint);
   }

   private Mesh getDecimatedMesh(int skip)
   {
      LogTools.warn("Decimating mesh of spline: too many vertices");
      meshBuilder.clear();
      for (int i = skip; i < points.size(); i += skip)
         meshBuilder.addLine(points.get(i - skip), points.get(i), lineWidth, color);

      return meshBuilder.generateMesh();
   }

   public void clear()
   {
      meshBuilder.clear();
      points.clear();

      if (lastModel != null)
      {
         lastModel.dispose();
         lastModel = null;
      }
      modelInstance = null;
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      if (modelInstance != null)
      {
         modelInstance.getRenderables(renderables, pool);
      }
   }

   public void dispose()
   {
      if (lastModel != null)
         lastModel.dispose();
   }
}