package us.ihmc.jMonkeyEngineToolkit.jme.lidar;

import java.util.ArrayList;

import com.jme3.collision.CollisionResult;
import com.jme3.collision.CollisionResults;
import com.jme3.math.Ray;

import us.ihmc.euclid.geometry.Line3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.jMonkeyEngineToolkit.Graphics3DWorld;
import us.ihmc.jMonkeyEngineToolkit.jme.JMEGraphics3DAdapter;
import us.ihmc.jMonkeyEngineToolkit.jme.util.JMEDataTypeUtils;
import us.ihmc.jMonkeyEngineToolkit.jme.util.JMEGeometryUtils;

public class RayTracingLidar
{
   private Graphics3DWorld world;
   private final LidarTestParameters params;
   private final int sensorId;

   private ArrayList<String> collisionNodeNames = new ArrayList<String>();

   public RayTracingLidar(Graphics3DWorld world, LidarTestParameters params, int sensorId)
   {
      this.world = world;
      this.params = params;
      this.sensorId = sensorId;
   }

   public void addCollisionNodes(String... collisionNodeNames)
   {
      for (String collisionNodeName : collisionNodeNames)
      {
         this.collisionNodeNames.add(collisionNodeName);
      }
   }

   public LidarTestScan scan(RigidBodyTransform lidarTransform)
   {
      Ray ray;
      int scansPerSweep = params.getScansPerSweep();
      float[] ranges = new float[scansPerSweep];
      CollisionResults masterResults;
      CollisionResults results;
      for (int i = 0; i < scansPerSweep; i++)
      {
         ray = JMEGeometryUtils.transformRayFromZupToJMECoordinate(JMEDataTypeUtils.ray3dToJMERay(getRay(params, lidarTransform, i)));

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

      return new LidarTestScan(params, lidarTransform, lidarTransform, ranges, sensorId);
   }

   public static Line3D getRay(LidarTestParameters params, RigidBodyTransform currentTransform, int index)
   {
      Line3D ray = new Line3D(0.0, 0.0, 0.0, 1.0, 0.0, 0.0);
      RigidBodyTransform transform = new RigidBodyTransform(currentTransform);
      transform.multiply(getSweepTransform(params, index));
      ray.applyTransform(transform);
      return ray;
   }

   public static RigidBodyTransform getSweepTransform(LidarTestParameters params, int i)
   {
      if (i >= params.getScansPerSweep() * params.getScanHeight())
      {
         throw new IndexOutOfBoundsException("Index " + i + " greater than or equal to pointsPerSweep " + params.getScansPerSweep() + " * scanHeight " + params.getScanHeight());
      }

      double yawPerIndex = (params.getLidarSweepEndAngle() - params.getLidarSweepStartAngle()) / (params.getScansPerSweep() - 1);
      double pitchPerIndex = (params.getLidarPitchMaxAngle() - params.getLidarPitchMinAngle()) / (params.getScanHeight() - 1);

      RigidBodyTransform sweepTransform = new RigidBodyTransform();
      if (params.getScansPerSweep() > 1)
      {
         //sweepTransform.rotZ(params.sweepYawMin + yawPerIndex * i);
         sweepTransform.setRotationYawAndZeroTranslation(params.getLidarSweepStartAngle() + yawPerIndex * (i % params.getScansPerSweep()));
      }
      if (params.getScanHeight() > 1)
      {
         sweepTransform.setRotationPitchAndZeroTranslation(params.getLidarPitchMinAngle() + pitchPerIndex * (i / params.getScansPerSweep()));
      }

      return sweepTransform;
   }
}
