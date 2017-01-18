package us.ihmc.jMonkeyEngineToolkit.jme.util;

import java.nio.FloatBuffer;
import java.util.Collection;
import java.util.Iterator;

import javax.vecmath.Point3f;
import javax.vecmath.Tuple3f;

import com.jme3.asset.AssetManager;
import com.jme3.material.Material;
import com.jme3.material.RenderState.BlendMode;
import com.jme3.math.ColorRGBA;
import com.jme3.math.Vector3f;
import com.jme3.renderer.queue.RenderQueue.Bucket;
import com.jme3.renderer.queue.RenderQueue.ShadowMode;
import com.jme3.scene.Geometry;
import com.jme3.scene.Mesh;
import com.jme3.scene.Mesh.Mode;
import com.jme3.scene.Node;
import com.jme3.scene.VertexBuffer;
import com.jme3.util.BufferUtils;

public class JMEPointCloudGenerator
{
   protected AssetManager assetManager;
   private float size = 0.01f;

   public JMEPointCloudGenerator(AssetManager assetManager)
   {
      this.assetManager = assetManager;
   }

   public void setSizeCM(float size)
   {
      this.size = size;
   }

   protected Node generatePointCloudGraphFrom(FloatBuffer pointCoordinates3d)
   {
      FloatBuffer colors = createColorBuffer(new ColorRGBA(1.0f, 1.0f, 1.0f, 1.0f), pointCoordinates3d);

      return generatePointCloudGraphFrom(pointCoordinates3d, colors);
   }

   protected Node generatePointCloudGraphFrom(FloatBuffer pointCoordinates3d, FloatBuffer colorsRGBA)
   {
      FloatBuffer sizes = createSizeBuffer(1.0f, pointCoordinates3d);

      return generatePointCloudGraphFrom(pointCoordinates3d, colorsRGBA, sizes);
   }

   protected Node generatePointCloudGraphFrom(FloatBuffer pointCoordinates3d, FloatBuffer colorsRGBA, FloatBuffer sizes)
   {
      Node result = new Node();

      Material mat = new Material(assetManager, "Common/MatDefs/Misc/Particle.j3md");
      mat.getAdditionalRenderState().setPointSprite(true);
      mat.getAdditionalRenderState().setBlendMode(BlendMode.Off);

      mat.getAdditionalRenderState().setDepthWrite(true);
      mat.getAdditionalRenderState().setDepthTest(true);
      mat.setBoolean("PointSprite", true);
      mat.setFloat("Quadratic", size * 100.0f * 4.0f);

      
//    Material mat = new Material(assetManager, "Terrain/RainbowHeightBoxTerrain.j3md");
//    Vector2f scale = new Vector2f(1, 1);
//    mat.setFloat("gridSize", 100);
//    mat.setVector2("scale", scale);
//    mat.getAdditionalRenderState().setPointSprite(true);
//    mat.setFloat("radius", 0.4f);    // brightness
//    mat.setFloat("minHeight", -0.3f);    // colors
//    mat.setFloat("maxHeight", 0.5f);    // colors
//    mat.setFloat("alpha", 1);
//    mat.setVector3("lineColor", new Vector3f(0f, 0f, 0f));
//    mat.setFloat("lineWidth", 0.025f);
//
//    mat.getAdditionalRenderState().setBlendMode(BlendMode.Alpha);

////
//      mat.setFloat("minHeight", 0.0f);    // colors
//      mat.setFloat("maxHeight", 0.15f);    // colors
//      mat.setFloat("radius", 0.5f);    // brightness

      
      Mesh m = new Mesh();
      m.setMode(Mode.Points);
      m.setBuffer(VertexBuffer.Type.Position, 3, pointCoordinates3d);
      m.setBuffer(VertexBuffer.Type.Color, 4, colorsRGBA);
      m.setBuffer(VertexBuffer.Type.Size, 1, sizes);
      m.setStatic();
      m.updateBound();

      Geometry g = new Geometry("Point Cloud", m);
      g.setShadowMode(ShadowMode.CastAndReceive);
      g.setQueueBucket(Bucket.Opaque);
      g.setMaterial(mat);
      g.updateModelBound();

      result.attachChild(g);
      result.updateModelBound();

      return result;
   }

   protected FloatBuffer createColorBuffer(ColorRGBA color, FloatBuffer points)
   {
      int bufferSize = (points.limit() / 3) * 4;
      FloatBuffer result = BufferUtils.createFloatBuffer(bufferSize);
      for (int i = 0; i < (bufferSize / 4); i++)
      {
         result.put(color.r).put(color.g).put(color.b).put(color.a);
      }

      return result;
   }

   protected FloatBuffer createSizeBuffer(float pointSize, FloatBuffer points)
   {
      int bufferSize = points.limit() / 3;
      FloatBuffer result = BufferUtils.createFloatBuffer(bufferSize);
      for (int i = 0; i < bufferSize; i++)
      {
         result.put(pointSize);
      }

      return result;
   }

   public Node generatePointCloudGraph(float[] pointCoordinates3d)
   {
      if ((pointCoordinates3d.length % 3) != 0)
         System.err.println("Number of point coordinates must be a multiple of 3!");

      FloatBuffer coords = BufferUtils.createFloatBuffer(pointCoordinates3d);

      return generatePointCloudGraph(coords, null);
   }

   public Node generatePointCloudGraph(FloatBuffer pointCoordinates3d)
   {
      FloatBuffer coords = pointCoordinates3d;

      return generatePointCloudGraph(coords, null);
   }
   
   public Node generatePointCloudGraph(Point3f[] pointCoordinates3d)
   {
      return generatePointCloudGraph(pointCoordinates3d, (ColorRGBA[]) null);
   }
   
   public Node generatePointCloudGraph(Point3f[] pointCoordinates3d, ColorRGBA[] colorsRGBA)
   {
      Vector3f[] vectorArray = new Vector3f[pointCoordinates3d.length];
      
      for (int i = 0; i < pointCoordinates3d.length; i++)
      {
         vectorArray[i] = new Vector3f(pointCoordinates3d[i].getX(), pointCoordinates3d[i].getY(), pointCoordinates3d[i].getZ());
      }

      return generatePointCloudGraph(vectorArray, colorsRGBA);
   }
   
   public Node generatePointCloudGraph(Vector3f[] pointCoordinates3d)
   {
      FloatBuffer coords = BufferUtils.createFloatBuffer(pointCoordinates3d);

      return generatePointCloudGraph(coords, null);
   }

   public <T extends Tuple3f> Node generatePointCloudGraph(Collection<T> pointCoordinates3d)
   {
      FloatBuffer coords = BufferUtils.createFloatBuffer(3 * pointCoordinates3d.size());
      Iterator<T> it = pointCoordinates3d.iterator();
      T current;
      while (it.hasNext())
      {
         current = it.next();
         coords.put(current.getX()).put(current.getY()).put(current.getZ());
      }

      coords.rewind();

      return generatePointCloudGraph(coords, null);
   }

   public Node generatePointCloudGraph(float[] pointCoordinates3d, float[] colorsRGBA)
   {
      if (colorsRGBA == null)
         return generatePointCloudGraph(pointCoordinates3d);

      if ((pointCoordinates3d.length % 3) != 0)
         throw new NumberFormatException("number of point coordinates must be a multiple of 3!");

      if ((colorsRGBA.length % 4) != 0)
         throw new NumberFormatException("number of color values must be a multiple of 4!");

      if (pointCoordinates3d.length / 3 != colorsRGBA.length / 4)
         System.err.println("There should be a color value for each point, if colors are used!");

      FloatBuffer coords = BufferUtils.createFloatBuffer(pointCoordinates3d);

      FloatBuffer colors = BufferUtils.createFloatBuffer(colorsRGBA);

      return generatePointCloudGraph(coords, colors);
   }

   public Node generatePointCloudGraph(FloatBuffer pointCoordinates3d, FloatBuffer colorsRGBA)
   {
      // now - this method calls the main generator function:
      if (colorsRGBA == null)
         return generatePointCloudGraphFrom(pointCoordinates3d);
      else
         return generatePointCloudGraphFrom(pointCoordinates3d, colorsRGBA);
   }

   public Node generatePointCloudGraph(Vector3f[] pointCoordinates3d, ColorRGBA[] colorsRGBA)
   {
      if (colorsRGBA == null)
         return generatePointCloudGraph(pointCoordinates3d);

      if (pointCoordinates3d.length != colorsRGBA.length)
         System.err.println("There should be a color value for each point, if colors are used!");

      FloatBuffer coords = BufferUtils.createFloatBuffer(pointCoordinates3d);

      FloatBuffer colors = BufferUtils.createFloatBuffer(4 * colorsRGBA.length);
      for (int i = 0; i < colorsRGBA.length; i++)
      {
         colors.put(colorsRGBA[i].r).put(colorsRGBA[i].g).put(colorsRGBA[i].b).put(colorsRGBA[i].a);
      }

      colors.rewind();

      return generatePointCloudGraph(coords, colors);
   }

   public Node generatePointCloudGraph(Point3f[] pointCloud, Collection<ColorRGBA> colorsRGBA) throws Exception
   {
      if (colorsRGBA == null)
         throw new Exception("point cloud colors must not be null!");

      if (pointCloud.length != colorsRGBA.size())
         throw new Exception("There should be a color value for each point, if colors are used!");

      FloatBuffer pointBuffer = BufferUtils.createFloatBuffer(3 * pointCloud.length);
      for(Point3f point : pointCloud)
      {
         pointBuffer.put(point.getX());
         pointBuffer.put(point.getY());
         pointBuffer.put(point.getZ());
      }
      pointBuffer.rewind();

      FloatBuffer colorBuffer = BufferUtils.createFloatBuffer(4 * colorsRGBA.size());
      for (ColorRGBA colorRGBA : colorsRGBA)
      {
         colorBuffer.put(colorRGBA.getColorArray());
      }
      colorBuffer.rewind();

      return generatePointCloudGraph(pointBuffer, colorBuffer);
   }
}
