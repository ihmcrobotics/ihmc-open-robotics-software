package us.ihmc.jMonkeyEngineToolkit.jme;

import java.util.ArrayList;
import java.util.concurrent.Callable;
import java.util.concurrent.Future;

import org.apache.commons.lang3.tuple.ImmutablePair;

import com.jme3.app.Application;
import com.jme3.scene.Node;

import us.ihmc.robotics.geometry.Ray3d;
import us.ihmc.tools.TimestampProvider;

public class JMEMultiRayTracer
{
   /**
    * This is technically ray casting, not ray tracing.
    */
   private static final boolean DEBUG = false;
   private final Application application;
   private long time;
   private final JMERayCollisionAdapter rayCollisionAdapter;
   private String[] childrenToIntersect = null;
   
   {
      if (DEBUG)
         System.out.println("JMEMultiRayTracer: DEBUG is true");
   }

   public JMEMultiRayTracer(Application application, Node rootNode)
   {
      this.application = application;
      this.rayCollisionAdapter = new JMERayCollisionAdapter(rootNode);
   }

   public void setChildrenToIntersect(String[] childrenToIntersect)
   {
      this.childrenToIntersect = childrenToIntersect;
   }

   public long scan(TimestampProvider timestampProvider, ArrayList<Ray3d> rays, double[] rawRayLengths)
   {
      try
      {
         Future<ImmutablePair<Long, Node>> futureRootNode = enqueueRootNodeExtracter(timestampProvider);
         ImmutablePair<Long, Node> retVal = futureRootNode.get();
         long timestamp = retVal.getLeft();
         Node rootNode = retVal.getRight();
         
         if (childrenToIntersect != null)
         {
            ArrayList<Node> children = new ArrayList<Node>();
            Node child;
            for (int i = 0; i < childrenToIntersect.length; i++)
            {
               if ((child = (Node) rootNode.getChild(childrenToIntersect[i])) != null)
               {
                  children.add(child);
               }
            }
            rootNode.detachAllChildren();
            for (Node n : children) {
               rootNode.attachChild(n);
            }
         }

         startDebugTimer();
         getPickDistance(rays.get(0), rootNode);
         reportDebugTimeToSetupPicking(rays);
         for (int i = 0; i < rays.size(); i++)
         {
            rawRayLengths[i] = getPickDistance(rays.get(i), rootNode);
         }
         reportDebugTimeToCastAllRays();
         return timestamp;
      }
      catch (Exception e)
      {
         throw new RuntimeException(e.getMessage());
      }
   }

   public void reportDebugTimeToCastAllRays()
   {
      if (DEBUG)
         System.out.println("JMEMultiRayTracer: elapsed time outside of RenderThread has been " + (System.nanoTime() - time) * 1.0e-9 + " seconds.");
   }

   public void reportDebugTimeToSetupPicking(final ArrayList<Ray3d> rays)
   {
      if (DEBUG)
         System.out.println("JMEMultiRayTracer: elapsed time setting up the scenegraph for picking " + (System.nanoTime() - time) * 1.0e-9 + " seconds.");
      if (DEBUG)
         System.out.println("JMEMultiRayTracer: number of rays = " + rays.size());
   }

   public void startDebugTimer()
   {
      time = System.nanoTime();
   }

   public Future<ImmutablePair<Long, Node>> enqueueRootNodeExtracter(final TimestampProvider timestampProvider)
   {
      return application.enqueue(new Callable<ImmutablePair<Long, Node>>()
      {
         public ImmutablePair<Long, Node> call() throws Exception
         {
            long time = System.nanoTime();
            long timestamp = timestampProvider != null ? timestampProvider.getTimestamp() : 0;
            Node newRoot = rayCollisionAdapter.cloneRootNode();
            if (DEBUG)
            {
               System.out.println("JMEMultiRayTracer: elapsed time in RenderThread has been " + (System.nanoTime() - time) * 1.0e-9 + " seconds.");
            }
            return new ImmutablePair<Long, Node>(timestamp, newRoot);
         }
      });
   }

   private double getPickDistance(Ray3d ray3d, Node rootNode)
   {
      rayCollisionAdapter.setPickingGeometry(ray3d);
      double pickDistance = rayCollisionAdapter.getPickDistance(rootNode);

      return pickDistance;
   }

}
