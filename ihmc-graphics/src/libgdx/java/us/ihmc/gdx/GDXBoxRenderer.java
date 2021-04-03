package us.ihmc.gdx;

import com.badlogic.gdx.Application;
import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.graphics.*;
import com.badlogic.gdx.graphics.g3d.Material;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.RenderableProvider;
import com.badlogic.gdx.graphics.g3d.attributes.ColorAttribute;
import com.badlogic.gdx.graphics.g3d.particles.ParticleShader;
import com.badlogic.gdx.graphics.glutils.ShaderProgram;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import org.lwjgl.opengl.GL32;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.shape.primitives.Box3D;
import us.ihmc.log.LogTools;
import us.ihmc.euclid.geometry.BoundingBox3D;

//import us.ihmc.javaFXToolkit.shapes.JavaFXMeshBuilder;
//import us.ihmc.humanoidBehaviors.ui.graphics.JavaFXGraphicPrimitives;
import us.ihmc.robotics.geometry.GeometryTools;
import us.ihmc.gdx.mesh.GDXMultiColorMeshBuilder;




import us.ihmc.gdx.GDXModelPrimitives;

//import ihmc_msgs.GDXBoxesMessage;
//import ihmc_msgs.GDXBoxMessage;


import com.badlogic.gdx.graphics.g3d.utils.MeshBuilder;


public class GDXBoxRenderer implements RenderableProvider
{
   private static final int SIZE_AND_ROTATION_USAGE = 1 << 9;
   private static boolean BOX_ENABLED = false;
   private Renderable renderable;
   private float[] vertices;
   //   private MeshBuilder newMesh;
   private short[] indices;
   //   private ShaderProgram shader_obj = new ShaderProgram("","");

   private final VertexAttributes vertexAttributes = new VertexAttributes(
         new VertexAttribute(VertexAttributes.Usage.Position, 3, ShaderProgram.POSITION_ATTRIBUTE),
         new VertexAttribute(VertexAttributes.Usage.ColorUnpacked, 4,ShaderProgram.COLOR_ATTRIBUTE),
         new VertexAttribute(SIZE_AND_ROTATION_USAGE, 3, "a_sizeAndRotation")
   );
   private final int vertexSize = 10;
   private final int vertexPositionOffset = (short) (vertexAttributes.findByUsage(VertexAttributes.Usage.Position).offset / 4);
   private final int vertexColorOffset = (short) (vertexAttributes.findByUsage(VertexAttributes.Usage.ColorUnpacked).offset / 4);
   private final int vertexSizeAndPositionOffset = (short) (vertexAttributes.findByUsage(SIZE_AND_ROTATION_USAGE).offset / 4);

   private RecyclingArrayList<BoundingBox3D> BoxesToRender;
   private RecyclingArrayList<RecyclingArrayList<BoundingBox3D>> BoxClusters;
   private Color color = Color.RED;
   GDXModelPrimitives GDXModelPrimitives = new GDXModelPrimitives();
//   JavaFXGraphicPrimitives JavaFXGraphicPrimitives  = new JavaFXGraphicPrimitives();
//   private ModelInstance ModelInstance = new ModelInstance(GDXModelPrimitives.createCoordinateFrame(0.3));


   public void create()
   {
      if (!BOX_ENABLED)
         enableBoxes();

//      renderable.meshPart.offset = 0;
//      renderable.material = new Material(ColorAttribute.createDiffuse(color));
//
//      vertices = new float[vertexSize];
//      indices = new short[vertexSize];
//      //      newMesh = new MeshBuilder();
//      //      newMesh.begin(vertexAttributes, GL20.GL_POINTS);
//
//      if (renderable.meshPart.mesh != null)
//         renderable.meshPart.mesh.dispose();
//      renderable.meshPart.mesh = new Mesh(true, 1, 0, vertexAttributes);
//
//      ParticleShader.Config config = new ParticleShader.Config(ParticleShader.ParticleType.Point);
//      renderable.shader = new ParticleShader(renderable, config);
//      //      ((ParticleShader) renderable.shader).set(ShaderProgram.COLOR_ATTRIBUTE., Color.RED);
//      renderable.shader.init();
      GeometryTools GeometryTools = new GeometryTools();

   }

   public void setColor(Color color)
   {
      this.color = color;
   }

   //   @Override
   //   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   //   {
   //      if (enabled)
   //         renderables.add(renderable);
   //   }

   public void setBoxRender(RecyclingArrayList<BoundingBox3D> BoxesToRender)
   {
      this.BoxesToRender = BoxesToRender;
   }

//   public void updateMesh()
//   {
//      if (BoxesToRender != null && !BoxesToRender.isEmpty())
//      {
//         for (int i = 0; i < BoxesToRender.size(); i++)
//         {
//            int offset = i * vertexSize;
//
//            Point3D32 point = BoxesToRender.get(i);
//            vertices[offset] = point.getX32();
//            vertices[offset + 1] = point.getY32();
//            vertices[offset + 2] = point.getZ32();
//
//            float c1 = Color.toFloatBits(255, 0, 0, 255);
//            LogTools.info(c1);
//            // color [0.0f - 1.0f]
//            vertices[offset + 3] = 0.5f; // red (not working yet)
//            vertices[offset + 4] = 0.6f; // blue
//            vertices[offset + 5] = 0.7f; // green
//            vertices[offset + 6] = 0.0f; // alpha
//
//
//            vertices[offset + 7] = 0.11f; // size
//            vertices[offset + 8] = 1.0f; // cosine [0-1]
//            vertices[offset + 9] = 0.0f; // sine [0-1]
//
//            //            indices[i] = (short)i;
//         }
//
//         renderable.meshPart.size = BoxesToRender.size();
//         renderable.meshPart.mesh.setVertices(vertices, 0, BoxesToRender.size() * vertexSize);
//         updateCluster();
//         updateClusterMesh(Clusters);
//
//         renderable.meshPart.update();
//         //         newMesh.addMesh(vertices, indices);
//      }
//   }

   public void updateCluster(){
      //      kdtree
   }

   public void updateClusterMesh(RecyclingArrayList<RecyclingArrayList<BoundingBox3D>> BoxClusters){

   }

   public void updateMesh(float alpha)
   {
      if (BoxesToRender != null && !BoxesToRender.isEmpty())
      {
         for (int i = 0; i < BoxesToRender.size(); i++)
         {
            int offset = i * vertexSize;

            BoundingBox3D box3D = BoxesToRender.get(i);
            vertices[offset] = box3D.getMinPoint().getX32();
            vertices[offset + 1] = box3D.getMinPoint().getX32();
            vertices[offset + 2] = box3D.getMinPoint().getZ32();

            // color [0.0f - 1.0f]
            vertices[offset + 3] = 0.5f; // red (not working yet)
            vertices[offset + 4] = 0.7f; // blue
            vertices[offset + 5] = 0.5f; // green
            vertices[offset + 6] = alpha; // alpha

            vertices[offset + 7] = 0.11f; // size
            vertices[offset + 8] = 1.0f; // cosine [0-1]
            vertices[offset + 9] = 0.0f; // sine [0-1]

         }
         renderable.meshPart.size = BoxesToRender.size();
         renderable.meshPart.mesh.setVertices(vertices, 0, BoxesToRender.size() * vertexSize);
         //         Gdx.gl.glClear(GL20.GL_COLOR_BUFFER_BIT);
         //         renderable.meshPart.mesh.render(new ShaderProgram("",""),GL20.GL_TRIANGLE_STRIP, 0, 4);

         renderable.meshPart.update();

         //         MeshBuilder newMesh = new MeshBuilder();
         //         newMesh.addMesh(vertices, indices);
      }
   }

   //   public void render(){
   //      Gdx.gl.glClear(GL20.GL_COLOR_BUFFER_BIT);
   //      renderable.meshPart.mesh.render(new ShaderProgram("",""),GL20.GL_TRIANGLE_STRIP, 0, 4);
   //   }

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

   private static void enableBoxes()
   {
//      Gdx.gl.glEnable(GL20.GL_VERTEX_PROGRAM_POINT_SIZE);
//      if (Gdx.app.getType() == Application.ApplicationType.Desktop)
//      {
//         Gdx.gl.glEnable(0x8861); // GL_POINT_OES
//      }
      BOX_ENABLED = true;
   }
}
