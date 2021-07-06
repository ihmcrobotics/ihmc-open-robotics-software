package us.ihmc.gdx;

import com.badlogic.gdx.Application;
import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.graphics.*;
import com.badlogic.gdx.graphics.g3d.*;
import com.badlogic.gdx.graphics.g3d.attributes.ColorAttribute;
import com.badlogic.gdx.graphics.g3d.particles.ParticleShader;
import com.badlogic.gdx.graphics.glutils.ShaderProgram;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.log.LogTools;

import java.util.Random;

public class GDXPointCloudRenderer implements RenderableProvider
{
   private static final int SIZE_AND_ROTATION_USAGE = 1 << 9;
   private static boolean POINT_SPRITES_ENABLED = false;
   private Renderable renderable;
   private float[] vertices;

   private final VertexAttributes vertexAttributes = new VertexAttributes(
         new VertexAttribute(VertexAttributes.Usage.Position, 3, ShaderProgram.POSITION_ATTRIBUTE),
         new VertexAttribute(VertexAttributes.Usage.ColorUnpacked, 4, ShaderProgram.COLOR_ATTRIBUTE),
         new VertexAttribute(SIZE_AND_ROTATION_USAGE, 3, "a_sizeAndRotation")
   );
   private final int vertexSize = 10;

   private RecyclingArrayList<Point3D32> pointsToRender;
   private ColorProvider colorProvider;
   private float pointScale = 1.0f;

   public interface ColorProvider {
      public float getNextR();
      public float getNextG();
      public float getNextB();
   }

   private static void enablePointSprites()
   {
      Gdx.gl30.glEnable(GL30.GL_VERTEX_PROGRAM_POINT_SIZE);
      if (Gdx.app.getType() == Application.ApplicationType.Desktop)
      {
         Gdx.gl30.glEnable(0x8861); // GL_POINT_OES
      }
      POINT_SPRITES_ENABLED = true;
   }

   public void create(int size)
   {
      if (!POINT_SPRITES_ENABLED)
         enablePointSprites();

      renderable = new Renderable();
      renderable.meshPart.primitiveType = GL30.GL_POINTS;
      renderable.meshPart.offset = 0;
      renderable.material = new Material(ColorAttribute.createDiffuse(Color.WHITE));

      vertices = new float[size * vertexSize];
      if (renderable.meshPart.mesh != null)
         renderable.meshPart.mesh.dispose();
      renderable.meshPart.mesh = new Mesh(false, size, 0, vertexAttributes);

      ParticleShader.Config config = new ParticleShader.Config(ParticleShader.ParticleType.Point);
      String prefix = ParticleShader.createPrefix(renderable, config);

      ShaderProgram.pedantic = true;

      final String fragmentShader = ParticleShader.getDefaultFragmentShader().replace("gl_FragColor = texture2D(u_diffuseTexture, texCoord)* v_color", "gl_FragColor = v_color");

      ShaderProgram shader = new ShaderProgram(prefix + ParticleShader.getDefaultVertexShader(), prefix + fragmentShader);
      for (String s : shader.getLog().split("\n"))
      {
         if (s.isEmpty())
            continue;

         if (s.contains("error"))
            LogTools.error(s);
         else
            LogTools.info(s);
      }

      renderable.shader = new ParticleShader(renderable, config, shader);
      renderable.shader.init();
   }

   public void updateMesh()
   {
      updateMesh(0.0f);
   }

   public void updateMesh(float alpha)
   {
      if (pointsToRender != null && !pointsToRender.isEmpty())
      {
         for (int i = 0; i < pointsToRender.size(); i++)
         {
            int offset = i * vertexSize;

            Point3D32 point = pointsToRender.get(i);
            vertices[offset] = point.getX32();
            vertices[offset + 1] = point.getY32();
            vertices[offset + 2] = point.getZ32();

            // color [0.0f - 1.0f]
            vertices[offset + 3] = colorProvider.getNextR();
            vertices[offset + 4] = colorProvider.getNextG();
            vertices[offset + 5] = colorProvider.getNextB();
            vertices[offset + 6] = alpha; // alpha

            vertices[offset + 7] = pointScale * 0.01f; // size
            vertices[offset + 8] = 1.0f; // cosine [0-1]
            vertices[offset + 9] = 0.0f; // sine [0-1]
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

   public void setPointsToRender(RecyclingArrayList<Point3D32> pointsToRender)
   {
      setPointsToRender(pointsToRender, new ColorProvider()
      {
         private final Random rand = new Random(0);

         @Override
         public float getNextR()
         {
            return rand.nextFloat();
         }

         @Override
         public float getNextG()
         {
            return rand.nextFloat();
         }

         @Override
         public float getNextB()
         {
            return rand.nextFloat();
         }
      });
   }

   public void setPointsToRender(RecyclingArrayList<Point3D32> pointsToRender, Color color) {
      setPointsToRender(pointsToRender, new ColorProvider()
      {
         private final Random rand = new Random(0);

         @Override
         public float getNextR()
         {
            return color.r;
         }

         @Override
         public float getNextG()
         {
            return color.g;
         }

         @Override
         public float getNextB()
         {
            return color.b;
         }
      });
   }

   public void setPointsToRender(RecyclingArrayList<Point3D32> pointsToRender, ColorProvider provider)
   {
      this.pointsToRender = pointsToRender;
      this.colorProvider = provider;
   }

   public void setPointScale(float size)
   {
      this.pointScale = size;
   }
}
