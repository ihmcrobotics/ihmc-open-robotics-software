package us.ihmc.jMonkeyEngineToolkit.jme.util;

import com.jme3.collision.Collidable;
import com.jme3.collision.CollisionResult;
import com.jme3.collision.CollisionResults;
import com.jme3.math.ColorRGBA;
import com.jme3.math.Ray;
import com.jme3.math.Transform;
import com.jme3.renderer.queue.RenderQueue.ShadowMode;
import com.jme3.scene.Geometry;
import com.jme3.scene.Node;

import us.ihmc.jMonkeyEngineToolkit.Updatable;
import us.ihmc.jMonkeyEngineToolkit.jme.JMERenderer;

import javax.vecmath.Point3f;
import javax.vecmath.Vector3f;
import java.awt.*;
import java.util.ArrayList;
import java.util.Random;
import java.util.concurrent.Callable;
import java.util.concurrent.atomic.AtomicReference;

public class JMELidarSpriteGenerator extends Node implements Updatable
{
   private static final Point3f ORIGIN = new Point3f();
   protected JMERenderer jmeRenderer;
   protected Node thisObject = this;
   protected Geometry pointCloudGeometry;
   protected ArrayList<ColorRGBA> colors;
   protected ColorRGBA defaultColor;
   protected boolean newCloudAvailable = false;
   protected JMEPointCloudGenerator pointCloudGenerator;
   protected ArrayList<ColorRGBA> colorList = new ArrayList<ColorRGBA>();
   private Random random = new Random();

   protected final AtomicReference<Point3f[]> pointSource = new AtomicReference<>();

   public JMELidarSpriteGenerator(JMERenderer jmeRenderer)
   {
      this(jmeRenderer, null);
   }

   public JMELidarSpriteGenerator(JMERenderer jmeRenderer, ColorRGBA colorRGBA)
   {
      super("JMELidarFINALVisualizer");

      this.jmeRenderer = jmeRenderer;
      pointCloudGenerator = new JMEPointCloudGenerator(jmeRenderer.getAssetManager());

      defaultColor = colorRGBA;
   }

   public void clear()
   {
      jmeRenderer.enqueue(new Callable<Object>()
      {
         public Object call() throws Exception
         {
            thisObject.detachAllChildren();

            return null;
         }
      });
   }

   public ArrayList<ColorRGBA> generateColors(Point3f[] points)
   {
      colorList.clear();
      float distance = 3f;
      float calcDistance = 0;
      int c;

      for (Point3f current : points)
      {
         calcDistance = ORIGIN.distance(current);
         c = Color.HSBtoRGB((calcDistance % distance) / distance, 1.0f, 1.0f);

         if (defaultColor == null)
         {
            colorList.add(new ColorRGBA(((c >> 16) & 0xFF) / 256.0f, ((c >> 8) & 0xFF) / 256.0f, ((c >> 0) & 0xFF) / 256.0f, 1.0f));
         }
         else
         {
            colorList.add(defaultColor);
         }
      }

      return colorList;
   }

   public void setDefaultColor(ColorRGBA color)
   {
      defaultColor = color;
   }

   public ArrayList<ColorRGBA> generateColors(int numberOfPoints)
   {
      colorList = new ArrayList<ColorRGBA>();

      for (int i = 0; i < numberOfPoints; i++)
      {
         colorList.add(new ColorRGBA(random.nextFloat(), random.nextFloat(), random.nextFloat(), 1.0f));
      }

      return colorList;
   }

   public void setResolution(float resolution)
   {
      pointCloudGenerator.setSizeCM(resolution);
   }

   public void updatePoints(Point3f[] source)
   {
      this.pointSource.set(source);
      newCloudAvailable = true;

   }

   public void update()
   {
      // System.out.println("test2"+newCloudAvailable);

      if (newCloudAvailable)
      {
         //System.out.println("test3");

         newCloudAvailable = false;
         Point3f[] pointSource = this.pointSource.get();

         if (pointSource == null)
         {
            return;
         }

         if (this.getParent() != null)
         {
            thisObject.detachAllChildren();

            colors = generateColors(pointSource);

            try
            {
               // System.out.println("making graph");
               Node pointCloud = pointCloudGenerator.generatePointCloudGraph(pointSource, colors);

               pointCloudGeometry = (pointCloud.getChildren().size() > 0) ? (Geometry) pointCloud.getChild(0) : null;
               pointCloud.setShadowMode(ShadowMode.CastAndReceive);
               thisObject.attachChild(pointCloud);

               // System.out.println("graph made");
            }
            catch (Exception e)
            {
               e.printStackTrace();
            }
         }
      }
   }

   protected float getLidarResolution()
   {
      // Default to 2 cm, child classes should override
      return 0.02f;
   }

   /**
    * Whether or not to allow collisions with the LIDAR point cloud.
    * Default set to true, child classes can override.
    * @return whether point cloud collisions are allowed
    */
   protected boolean allowPointCloudCollisions()
   {
      return true;
   }

   @Override
   public int collideWith(Collidable other, CollisionResults results)
   {
      if (!allowPointCloudCollisions())
      {
         return super.collideWith(other, results);
      }

      Geometry geom;
      synchronized (pointSource)
      {
         geom = pointCloudGeometry;
      }

      // Perform a collision with the point cloud first, if no hit, move on
      if ((other instanceof Ray) && (geom != null))
      {
         Ray ray = (Ray) other;
         if ((ray.direction != null) && (ray.origin != null) && (ray.direction.lengthSquared() != 0))
         {
            Point3f origin = new Point3f(ray.getOrigin().x, ray.getOrigin().y, ray.getOrigin().z);
            Vector3f dir = new Vector3f(ray.getDirection().x, ray.getDirection().y, ray.getDirection().z);
            Point3f nearest = getNearestIntersection(origin, dir, getLidarResolution(), geom.getWorldTransform());
            if (nearest != null)
            {
               CollisionResult collRes = new CollisionResult(geom, new com.jme3.math.Vector3f(nearest.getX(), nearest.getY(), nearest.getZ()), origin.distance(nearest), 0);
               collRes.setContactNormal(new com.jme3.math.Vector3f(0, 0, 1));
               results.addCollision(collRes);

               return 1;
            }
            else
            {
               return 0;
            }
         }
      }

      return super.collideWith(other, results);
   }

   /**
    * Find the nearest point along the ray that is also closest to the ray origin.
    * @param origin ray origin
    * @param direction ray direction
    * @param resolution lidar resolution
    * @return closest point or null if the ray did not hit any lidar point
    */
   public Point3f getNearestIntersection(Point3f origin, Vector3f direction, double resolution, Transform pointTransform)
   {
      Point3f[] points = pointSource.get();
      direction.normalize();
      float dx, dy, dz, dot;
      double distanceToLine, distance;

      double nearestDistance = Double.POSITIVE_INFINITY;
      Point3f nearestPoint = null;

      for (Point3f p1 : points)
      {
         com.jme3.math.Vector3f p = new com.jme3.math.Vector3f(p1.getX(), p1.getY(), p1.getZ());
         pointTransform.transformVector(p, p);

         dx = origin.getX() - p.x;
         dy = origin.getY() - p.y;
         dz = origin.getZ() - p.z;

         dot = dx * direction.getX() + dy * direction.getY() + dz * direction.getZ();

         dx = dx - dot * direction.getX();
         dy = dy - dot * direction.getY();
         dz = dz - dot * direction.getZ();

         distanceToLine = (dx * dx + dy * dy + dz * dz);

         Point3f curpt = new Point3f(p.x, p.y, p.z);

         if (distanceToLine < resolution * resolution)
         {
            distance = origin.distanceSquared(curpt);

            if (distance < nearestDistance)
            {
               nearestDistance = distance;
               nearestPoint = curpt;
            }
         }
      }

      return nearestPoint;
   }

   @Override
   public void simpleUpdate(float tpf)
   {
      update();
   }
}
