package us.ihmc.jMonkeyEngineToolkit.jme;

import java.net.URL;
import java.util.ArrayList;
import java.util.logging.Level;
import java.util.logging.Logger;

import com.jme3.asset.AssetManager;
import com.jme3.scene.Node;
import com.jme3.system.JmeSystem;

import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.HeightMap;
import us.ihmc.graphicsDescription.structure.Graphics3DNode;
import us.ihmc.graphicsDescription.structure.Graphics3DNodeType;
import us.ihmc.jMonkeyEngineToolkit.jme.util.SimpleLRUCache;
import us.ihmc.robotics.geometry.Ray3d;

public class JMEGeneratedHeightMap implements HeightMap
{
   private final boolean DEBUG = false;
   
   private final static int cacheSize = 1000000;
   private final static double measurementHeight = 100.0;

   private final int resolution;
   private final double elementTestSize;

//   private final double xMin = -100.0, xMax = 100.0, yMin = -100.0, yMax = 100.0; 
   private final double xMin = 0.0, xMax = 0.0, yMin = 0.0, yMax = 0.0; // Disable rendering heightmap
   private final BoundingBox3D boundingBox = new BoundingBox3D(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

   private final JMERayCollisionAdapter jmeRayCollisionAdapter;
   private final Node rootNode = new Node("jmeGeneratedHeightMapRootNode");
   private final Node zUpNode = new Node("jmeGeneratedHeightMapZUpNode");

   private final AssetManager assetManager;
   private final JMEAssetLocator assetLocator;

   private final SimpleLRUCache<Long, GroundPoint> cache = new SimpleLRUCache<Long, GroundPoint>(cacheSize);

   private int lookups = 0;
   private int cacheHits = 0;
   
   
   private final ThreadLocal<Ray3d> tempRay = new ThreadLocal<Ray3d>()
   {
      @Override
      protected Ray3d initialValue()
      {
         return new Ray3d(new Point3D(), new Vector3D(0.0, 0.0, -1.0));
      }
   };

   public JMEGeneratedHeightMap(ArrayList<Graphics3DNode> nodes, int resolution)
   {
      Logger.getLogger("").setLevel(Level.WARNING);
      this.resolution = resolution;
      this.elementTestSize = 1.0/((double)resolution) + 1e-12;

      System.out.println("Generating heightmap tree");
      final URL resource = AssetManager.class.getResource("Desktop.cfg");
      assetManager = JmeSystem.newAssetManager(resource);
      JMERenderer.setupAssetManger(assetManager);
      assetLocator = new JMEAssetLocator(assetManager);

//      zUpNode.setLocalRotation(JMEGeometryUtils.getRotationFromJMEToZupCoordinates());
      rootNode.attachChild(zUpNode);

      for (Graphics3DNode node : nodes)
      {
         addNodesRecursively(node, zUpNode);
      }

      rootNode.updateModelBound();
      rootNode.updateLogicalState(0.0f);
      rootNode.updateGeometricState();

      System.out.println("Done generating heightmap");

      jmeRayCollisionAdapter = new JMERayCollisionAdapter(rootNode);
   }

   public GroundPoint getGroundPoint(double x, double y, double z)
   {
      lookups++;
      int xMil = (int) Math.round(x * resolution);
      int yMil = (int) Math.round(y * resolution);

      long key = ((xMil & 0xFFFFFFFFL) << 32) + (yMil & 0xFFFFFFFFL);

      GroundPoint point = cache.get(key);

      if(DEBUG)
      {
         if(lookups % 10000 == 0)
         {
            System.out.println("Cache hit percentage: " + (((double) cacheHits)/((double) lookups) * 100.0));
         }
      }
      
      if (point != null)
      {
         if (Math.abs(x - point.x) > elementTestSize || Math.abs(y - point.y) > elementTestSize)
         {
            System.err.println("Wrong cache result returned for key: " + key + ", expected: (" + x + "," + y + "), returned: (" + point.getX() + ","
                  + point.getY() + ")");
            Thread.dumpStack();
         }
         cacheHits++;

         return point;
      }

      Ray3d ray = tempRay.get();
      ray.getPoint().set(x, y, measurementHeight);
      jmeRayCollisionAdapter.setPickingGeometry(ray);
      Vector3D normal = new Vector3D();
      Point3D closestPoint = new Point3D();
      double distance = jmeRayCollisionAdapter.getPickDistance(rootNode, normal, closestPoint);

      point = new GroundPoint(x, y, measurementHeight - distance, normal, closestPoint);

      cache.put(key, point);

      return point;
   }

   public double heightAt(double x, double y, double z)
   {
      return getGroundPoint(x, y, z).getZ();
   }

   private JMEGraphics3DNode addNodesRecursively(Graphics3DNode graphics3dNode, Node parentNode)
   {
      JMEGraphics3DNode jmeNode = new JMEGraphics3DNode(graphics3dNode, assetLocator, null, null);
      Graphics3DNodeType nodeType = graphics3dNode.getNodeType();
      jmeNode.setType(nodeType);

      parentNode.attachChild(jmeNode);

      for (Graphics3DNode child : graphics3dNode.getChildrenNodes())
      {
         addNodesRecursively(child, jmeNode);
      }

      return jmeNode;
   }

   public boolean isClose(double x, double y, double z)
   {
      return true;
   }

   public void closestIntersectionTo(double x, double y, double z, Point3D intersectionToPack)
   {
      GroundPoint groundPoint = getGroundPoint(x, y, z);
      intersectionToPack.set(groundPoint.getClosestIntersection());
   }

   public void surfaceNormalAt(double x, double y, double z, Vector3D normalToPack)
   {
      GroundPoint groundPoint = getGroundPoint(x, y, z);
      normalToPack.set(groundPoint.getNormal());
   }

   public void closestIntersectionAndNormalAt(double x, double y, double z, Point3D intersectionToPack, Vector3D normalToPack)
   {
      GroundPoint groundPoint = getGroundPoint(x, y, z);
      normalToPack.set(groundPoint.getNormal());
      intersectionToPack.set(groundPoint.getClosestIntersection());
   }

   private class GroundPoint
   {
      private final double x, y, z;
      private final Vector3D normal;
      private final Point3D closestIntersection;

      public GroundPoint(double x, double y, double z, Vector3D normal, Point3D closestIntersection)
      {
         this.x = x;
         this.y = y;
         this.z = z;
         this.normal = normal;
         this.closestIntersection = closestIntersection;
      }

      public double getZ()
      {
         return z;
      }

      public Vector3D getNormal()
      {
         return normal;
      }

      public Point3D getClosestIntersection()
      {
         return closestIntersection;
      }

      public double getX()
      {
         return x;
      }

      public double getY()
      {
         return y;
      }

   }
   
   public BoundingBox3D getBoundingBox()
   {
      return boundingBox;
   }

}
