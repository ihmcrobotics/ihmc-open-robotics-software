package us.ihmc.jMonkeyEngineToolkit.jme.lidar;

import java.util.ArrayList;
import java.util.Arrays;

import us.ihmc.robotics.lidar.LidarScan;
import us.ihmc.robotics.lidar.LidarScanParameters;
import us.ihmc.jMonkeyEngineToolkit.Graphics3DWorld;
import us.ihmc.jMonkeyEngineToolkit.jme.JMEGraphics3DAdapter;
import us.ihmc.jMonkeyEngineToolkit.jme.util.JMEDataTypeUtils;
import us.ihmc.jMonkeyEngineToolkit.jme.util.JMEGeometryUtils;
import us.ihmc.robotics.geometry.RigidBodyTransform;

import com.jme3.collision.CollisionResult;
import com.jme3.collision.CollisionResults;
import com.jme3.math.Ray;

public class RayTracingLidar
{
   private Graphics3DWorld world;
   private int scansPerSweep;
   private double fieldOfView;
   private double minRange;
   private double maxRange;
   private final int sensorId;

   private ArrayList<String> collisionNodeNames = new ArrayList<String>();

   public RayTracingLidar(Graphics3DWorld world, int scansPerSweep, double fieldOfView, double minRange, double maxRange, int sensorId)
   {
      this.world = world;
      this.scansPerSweep = scansPerSweep;
      this.fieldOfView = fieldOfView;
      this.minRange = minRange;
      this.maxRange = maxRange;
      this.sensorId = sensorId;
   }

   public void addCollisionNodes(String... collisionNodeNames)
   {
      for (String collisionNodeName : collisionNodeNames)
      {
         this.collisionNodeNames.add(collisionNodeName);
      }
   }

   public LidarScan scan(RigidBodyTransform lidarTransform)
   {
      float[] unitRanges = new float[scansPerSweep];
      Arrays.fill(unitRanges, 1.0f);
      LidarScan unitScan = new LidarScan(new LidarScanParameters(scansPerSweep, (float) -fieldOfView / 2, (float) fieldOfView / 2, 0, (float) minRange,
                              (float) maxRange, 0), new RigidBodyTransform(lidarTransform), new RigidBodyTransform(lidarTransform), new float[scansPerSweep], sensorId);

      Ray ray;
      float[] ranges = new float[scansPerSweep];
      CollisionResults masterResults;
      CollisionResults results;
      for (int i = 0; i < scansPerSweep; i++)
      {
         ray = JMEGeometryUtils.transformRayFromZupToJMECoordinate(JMEDataTypeUtils.ray3dToJMERay(unitScan.getRay(i)));

         masterResults = new CollisionResults();
         for (String collisionNodeName : collisionNodeNames)
         {
            results = new CollisionResults();
            ((JMEGraphics3DAdapter) world.getGraphics3DAdapter()).getRenderer().getZUpNode().getChild(collisionNodeName).collideWith(ray, results);

            for (CollisionResult result : results)
            {
               masterResults.addCollision(result);
            }
         }

         CollisionResult closestCollision = masterResults.getClosestCollision();
         if (closestCollision != null)
         {
            ranges[i] = closestCollision.getDistance();
         }
         else
         {
            ranges[i] = 0.0f;
         }
      }

      return new LidarScan(new LidarScanParameters(scansPerSweep, (float) -fieldOfView / 2, (float) fieldOfView / 2, 0, (float) minRange, (float) maxRange, 0),
                           new RigidBodyTransform(lidarTransform), new RigidBodyTransform(lidarTransform), ranges, sensorId);
   }
}
