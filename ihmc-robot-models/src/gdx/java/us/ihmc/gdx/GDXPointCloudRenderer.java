package us.ihmc.gdx;

import com.badlogic.gdx.graphics.*;
import com.badlogic.gdx.graphics.g3d.Material;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.RenderableProvider;
import com.badlogic.gdx.graphics.g3d.attributes.ColorAttribute;
import com.badlogic.gdx.graphics.glutils.ShaderProgram;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.tuple3D.Point3D32;

public class GDXPointCloudRenderer implements RenderableProvider
{
   private Renderable renderable;
   private float[] vertices;

   private final VertexAttributes vertexAttributes = new VertexAttributes(
         new VertexAttribute(VertexAttributes.Usage.Position, 3, ShaderProgram.POSITION_ATTRIBUTE),
         new VertexAttribute(VertexAttributes.Usage.ColorUnpacked, 4,ShaderProgram.COLOR_ATTRIBUTE)
   );
   private final int vertexSize = (short) (vertexAttributes.vertexSize / 4);
   private final int vertexPositionOffset = (short) (vertexAttributes.findByUsage(VertexAttributes.Usage.Position).offset / 4);
   private final int vertexColorOffset = (short) (vertexAttributes.findByUsage(VertexAttributes.Usage.ColorUnpacked).offset / 4);

   private RecyclingArrayList<Point3D32> pointsToRender;
   private static final Color COLOR = Color.WHITE;

   public void create(int size)
   {
      renderable = new Renderable();
      renderable.meshPart.primitiveType = GL20.GL_POINTS;
      renderable.meshPart.offset = 0;
      renderable.material = new Material(ColorAttribute.createDiffuse(Color.WHITE));

      vertices = new float[size * vertexSize];
      if (renderable.meshPart.mesh != null)
         renderable.meshPart.mesh.dispose();
      renderable.meshPart.mesh = new Mesh(false, size, 0, vertexAttributes);
   }

   public void setPointsToRender(RecyclingArrayList<Point3D32> pointsToRender)
   {
      this.pointsToRender = pointsToRender;
   }

   public void render()
   {
      if (pointsToRender != null && !pointsToRender.isEmpty())
      {
         for (int i = 0; i < pointsToRender.size(); i++)
         {
            int offset = i * vertexSize;

            Point3D32 point = pointsToRender.get(i);
            vertices[offset + vertexPositionOffset] = point.getX32();
            vertices[offset + vertexPositionOffset + 1] = point.getY32();
            vertices[offset + vertexPositionOffset + 2] = point.getZ32();

            vertices[offset + vertexColorOffset] = COLOR.r;
            vertices[offset + vertexColorOffset + 1] = COLOR.g;
            vertices[offset + vertexColorOffset + 2] = COLOR.b;
            vertices[offset + vertexColorOffset + 3] = COLOR.a;
         }

         renderable.meshPart.size = pointsToRender.size();
         renderable.meshPart.mesh.setVertices(vertices, 0, pointsToRender.size() * vertexSize);
         renderable.meshPart.update();
      }
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      renderables.add(renderable);
   }

   public void dispose()
   {
      if (renderable.meshPart.mesh != null)
         renderable.meshPart.mesh.dispose();
   }
}
