package us.ihmc.jMonkeyEngineToolkit.yoGraphics;

import java.nio.FloatBuffer;
import java.util.concurrent.Callable;

import com.jme3.material.Material;
import com.jme3.material.RenderState.BlendMode;
import com.jme3.math.ColorRGBA;
import com.jme3.renderer.queue.RenderQueue.Bucket;
import com.jme3.renderer.queue.RenderQueue.ShadowMode;
import com.jme3.scene.Geometry;
import com.jme3.scene.Mesh;
import com.jme3.scene.Mesh.Mode;
import com.jme3.scene.Node;
import com.jme3.scene.VertexBuffer;
import com.jme3.util.BufferUtils;

import us.ihmc.euclid.transform.AffineTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.plotting.artifact.Artifact;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphic;
import us.ihmc.jMonkeyEngineToolkit.jme.JMERenderer;

public class YoGraphicPointCloud extends YoGraphic
{
   private final JMERenderer jmeRenderer;
   private Graphics3DObject graphics3dObject;
   
   private final int capacity;
   private final Point3D[] points;
   
   private final Node node;
   private final Material material;
   private final Mesh mesh;
   private final Geometry geometry;
   private final FloatBuffer pointBuffer;
   private final FloatBuffer colorBuffer;
   private final FloatBuffer sizeBuffer;

   public YoGraphicPointCloud(String name, int capacity, int spriteSizePixels, ColorRGBA color, JMERenderer jmeRenderer)
   {
      super(name);

      this.jmeRenderer = jmeRenderer;
      this.capacity = capacity;
      
      points = new Point3D[capacity];
      
      for (int i = 0; i < capacity; i++)
      {
         points[i] = new Point3D();
      }

      pointBuffer = BufferUtils.createFloatBuffer(3 * capacity);
      for(int i = 0; i < capacity; i++)
      {
         pointBuffer.put(points[i].getX32());
         pointBuffer.put(points[i].getY32());
         pointBuffer.put(points[i].getZ32());
      }
      pointBuffer.rewind();
      
      colorBuffer = BufferUtils.createFloatBuffer(4 * capacity);
      for (int i = 0; i < capacity; i++)
      {
         colorBuffer.put(color.getColorArray());
      }
      colorBuffer.rewind();
      
      int bufferSize = pointBuffer.limit() / 3;
      sizeBuffer = BufferUtils.createFloatBuffer(bufferSize);
      for (int i = 0; i < bufferSize; i++)
      {
         sizeBuffer.put(1.0f);
      }
      
      node = new Node();
      
      material = new Material(jmeRenderer.getAssetManager(), "Common/MatDefs/Misc/Particle.j3md");
      material.getAdditionalRenderState().setPointSprite(true);
      material.getAdditionalRenderState().setBlendMode(BlendMode.Off);
      material.getAdditionalRenderState().setDepthWrite(true);
      material.getAdditionalRenderState().setDepthTest(true);
      material.setBoolean("PointSprite", true);
      material.setFloat("Quadratic", spriteSizePixels);
      
      mesh = new Mesh();
      mesh.setMode(Mode.Points);
      mesh.setBuffer(VertexBuffer.Type.Position, 3, pointBuffer);
      mesh.setBuffer(VertexBuffer.Type.Color, 4, colorBuffer);
      mesh.setBuffer(VertexBuffer.Type.Size, 1, sizeBuffer);
      mesh.setStatic();
      mesh.updateBound();
      
      geometry = new Geometry("Point Cloud", mesh);
      geometry.setShadowMode(ShadowMode.CastAndReceive);
      geometry.setQueueBucket(Bucket.Opaque);
      geometry.setMaterial(material);
      geometry.updateModelBound();
      
      node.attachChild(geometry);
      node.updateModelBound();

      jmeRenderer.getZUpNode().attachChild(node);

      graphics3dObject = new Graphics3DObject();
   }
   
   public void update(Point3DReadOnly[] points, int size)
   {      
      for (int i = 0; i < capacity; i++)
      {
         if (i >= size)
         {
            this.points[i].setToNaN();
         }
         else
         {
            this.points[i].set(points[i]);
         }
      }
      
      jmeRenderer.enqueue(new Callable<Object>()
      {
         @Override
         public Object call() throws Exception
         {
            pointBuffer.limit(size * 3);
            colorBuffer.limit(size * 4);
            sizeBuffer.limit(size * 1);
            
            for (int i = 0, j = 0; i < size; i++)
            {
               pointBuffer.put(j++, points[i].getX32());
               pointBuffer.put(j++, points[i].getY32());
               pointBuffer.put(j++, points[i].getZ32());
            }
            
            mesh.setBuffer(VertexBuffer.Type.Position, 3, pointBuffer);
            mesh.setBuffer(VertexBuffer.Type.Color, 4, colorBuffer);
            mesh.setBuffer(VertexBuffer.Type.Size, 1, sizeBuffer);
            mesh.updateCounts();
            node.updateModelBound();
            
            return null;
         }
      });
   }

   @Override
   protected void computeRotationTranslation(AffineTransform transform3d)
   {
      transform3d.setIdentity();
   }

   @Override
   protected boolean containsNaN()
   {
      return false;
   }

   @Override
   public Graphics3DObject getLinkGraphics()
   {
      return graphics3dObject;
   }

   @Override
   public Artifact createArtifact()
   {
      return null;
   }
}
