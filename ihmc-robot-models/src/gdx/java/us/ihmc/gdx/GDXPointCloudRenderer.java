package us.ihmc.gdx;

import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.graphics.*;
import com.badlogic.gdx.graphics.g3d.Material;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.attributes.ColorAttribute;
import com.badlogic.gdx.graphics.glutils.ShaderProgram;
import org.lwjgl.opengl.GL32;
import us.ihmc.euclid.tuple3D.Point3D32;

public class GDXPointCloudRenderer
{
   private Renderable renderable;
   private float[] vertices;

   private final int sizeAndRotationUsage = 1 << 9;
   private final VertexAttributes CPU_ATTRIBUTES = new VertexAttributes(
         new VertexAttribute(VertexAttributes.Usage.Position, 3, ShaderProgram.POSITION_ATTRIBUTE),
         new VertexAttribute(VertexAttributes.Usage.ColorUnpacked, 4,ShaderProgram.COLOR_ATTRIBUTE),
         new VertexAttribute(sizeAndRotationUsage, 3, "a_sizeAndRotation")
   );
   private final int CPU_VERTEX_SIZE = (short) (CPU_ATTRIBUTES.vertexSize / 4);
   private final int CPU_POSITION_OFFSET = (short) (CPU_ATTRIBUTES.findByUsage(VertexAttributes.Usage.Position).offset / 4);
   private final int CPU_COLOR_OFFSET = (short) (CPU_ATTRIBUTES.findByUsage(VertexAttributes.Usage.ColorUnpacked).offset / 4);
   private final int CPU_SIZE_AND_ROTATION_OFFSET = (short) (CPU_ATTRIBUTES.findByUsage(sizeAndRotationUsage).offset / 4);

   private Point3D32[] pointsToRender;

   public void create()
   {
      renderable = new Renderable();
      renderable.meshPart.primitiveType = GL20.GL_POINTS;
      renderable.meshPart.offset = 0;
      renderable.material = new Material(ColorAttribute.createDiffuse(Color.WHITE));

      int capacity = 480000;
      vertices = new float[capacity * CPU_VERTEX_SIZE];
      if (renderable.meshPart.mesh != null) renderable.meshPart.mesh.dispose();
      renderable.meshPart.mesh = new Mesh(false, capacity, 0, CPU_ATTRIBUTES);
   }

   public void setPointsToRender(Point3D32[] pointsToRender)
   {
      this.pointsToRender = pointsToRender;
   }

   public void render(GDX3DApplication gdx3DApplication)
   {
      Gdx.gl.glEnable(GL32.GL_VERTEX_PROGRAM_POINT_SIZE);
      Gdx.gl.glEnable(GL32.GL_POINT_SPRITE);

      if (pointsToRender != null)
      {
         for (int i = 0; i < pointsToRender.length; i++)
         {
            int offset = i * CPU_VERTEX_SIZE;

            vertices[offset + CPU_POSITION_OFFSET] = pointsToRender[i].getX32();
            vertices[offset + CPU_POSITION_OFFSET + 1] = pointsToRender[i].getY32();
            vertices[offset + CPU_POSITION_OFFSET + 2] = pointsToRender[i].getZ32();
            vertices[offset + CPU_COLOR_OFFSET] = Color.WHITE.r;
            vertices[offset + CPU_COLOR_OFFSET + 1] = Color.WHITE.g;
            vertices[offset + CPU_COLOR_OFFSET + 2] = Color.WHITE.b;
            vertices[offset + CPU_COLOR_OFFSET + 3] = Color.WHITE.a;
            vertices[offset + CPU_SIZE_AND_ROTATION_OFFSET] = 1;
            vertices[offset + CPU_SIZE_AND_ROTATION_OFFSET + 1] = 0;
            vertices[offset + CPU_SIZE_AND_ROTATION_OFFSET + 2] = 1;
         }

         renderable.meshPart.size = pointsToRender.length;
         renderable.meshPart.mesh.setVertices(vertices, 0, pointsToRender.length * CPU_VERTEX_SIZE);
         renderable.meshPart.update();

         gdx3DApplication.getModelBatch().render(renderable);
      }
   }
}
