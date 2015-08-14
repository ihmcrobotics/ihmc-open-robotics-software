package us.ihmc.robotics.lidar;

import com.esotericsoftware.kryo.serializers.FieldSerializer.Optional;
import org.apache.commons.lang3.ArrayUtils;
import us.ihmc.robotics.geometry.LineSegment3d;
import us.ihmc.robotics.geometry.Ray3d;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.kinematics.TransformInterpolationCalculator;

import javax.vecmath.Point3d;
import javax.vecmath.Point3f;
import javax.vecmath.Vector3d;
import java.util.ArrayList;
import java.util.Arrays;

public class LidarScan 
{
   public LidarScanParameters params;
   public RigidBodyTransform worldTransformStart;
   public RigidBodyTransform worldTransformEnd;
   public RigidBodyTransform localTransformStart;
   public RigidBodyTransform localTransformEnd;
   public RigidBodyTransform averageTransform;
   
   @Optional("TranformationInterpolationCalculator")
   private final TransformInterpolationCalculator transformInterpolationCalculator = new TransformInterpolationCalculator();
   
   public float[] ranges;
   public int sensorId;

   public LidarScan()
   {
   }
   
   public LidarScan(LidarScanParameters params, float[] ranges, int sensorId)
   {
      this.params = params;
      this.ranges = ranges;
      this.sensorId = sensorId;
   }

   public LidarScan(LidarScanParameters params, RigidBodyTransform start, RigidBodyTransform end, float[] ranges, int sensorId)
   {
      this.params = params;
      setWorldTransforms(start, end);
      this.ranges = ranges;
      this.sensorId = sensorId;
   }
   
   public LidarScan(LidarScanParameters params, RigidBodyTransform start, RigidBodyTransform end, float[] ranges)
   {
      this.params = params;
      setWorldTransforms(start, end);
      this.ranges = ranges;
      this.sensorId = 0;
   }

   public ArrayList<Point3d> getAllPoints()
   {
      ArrayList<Point3d> points = new ArrayList<Point3d>();
      for (int i = 0; i < ranges.length; i++)
      {
         points.add(getPoint(i, ranges[i]));
      }
      return points;
   }
   
   public ArrayList<Point3f> getAllPoints3f()
   {
      ArrayList<Point3f> points = new ArrayList<Point3f>();
      for (int i = 0; i < ranges.length; i++)
      {
         points.add(getPoint3f(i, ranges[i]));
      }

      return points;
   }

   public int size()
   {
      return ranges.length;
   }

   public Point3d getPoint(int index)
   {
      return getPoint(index, ranges[index]);
   }
   
   public Point3f getPoint3f(int index)
   {
      return getPoint3f(index, ranges[index]);
   }

   public LineSegment3d getLineSegment(int index)
   {
      return getLineSegment(index, ranges[index]);
   }

   /**
   *
   * @param index of the point with respect to {@link #getPoint(int)}
   * @return
   */   public float getRange(int i)
   {
      return ranges[i];
   }

   public float[] getRanges()
   {
      return ranges;
   }

   public LidarScan getCopy()
   {
      return new LidarScan(getParams(), ranges.clone(),sensorId);
   }

   public int getSensorId()
   {
      return sensorId;
   }

   public LidarScan flipNew()
   {
      float[] flippedRanges = Arrays.copyOf(ranges, ranges.length);
      
      ArrayUtils.reverse(flippedRanges);
      
      return new LidarScan(getParams(), getEndTransform(), getStartTransform(), flippedRanges, sensorId);
   }
   
   public LidarScanParameters getParams()
   {
      return params;
   }
   
   /* SETTERS */

   public void setWorldTransforms(RigidBodyTransform start, RigidBodyTransform end)
   {
      this.worldTransformStart = start;
      this.worldTransformEnd = end;
      this.averageTransform = new RigidBodyTransform();
      transformInterpolationCalculator.computeInterpolation(worldTransformStart, worldTransformEnd, averageTransform, .5);
   }
   
   public LidarScanParameters getScanParameters()
   {
      return params;
   }

   public RigidBodyTransform getStartTransform()
   {
      return worldTransformStart;
   }

   public RigidBodyTransform getEndTransform()
   {
      return worldTransformEnd;
   }

   public RigidBodyTransform getAverageTransform()
   {
      return averageTransform;
   }

   public Ray3d getRay(int index)
   {
      LineSegment3d unitSegment = getLineSegment(index, 1.0f);

      return new Ray3d(unitSegment.getPointA(), unitSegment.getDirection());
   }
   

   /* PRIVATE/PROTECTED FUNCTIONS */
   protected Point3d getPoint(int index, float range)
   {
      Point3d p = new Point3d(range, 0.0, 0.0);
      RigidBodyTransform transform = new RigidBodyTransform();
      packInterpolatedTransform(index, transform);
      transform.multiply(getSweepTransform(index));
      transform.transform(p);

      return p;
   }
   
   protected Point3f getPoint3f(int index, float range)
   {
      Point3f p = new Point3f(range, 0.0f, 0.0f);
      RigidBodyTransform transform = new RigidBodyTransform();
      packInterpolatedTransform(index, transform);
      transform.multiply(getSweepTransform(index));
      transform.transform(p);

      return p;
   }

   protected LineSegment3d getLineSegment(int index, float range)
   {
      Vector3d origin = new Vector3d();
      RigidBodyTransform transform = new RigidBodyTransform();
      packInterpolatedTransform(index, transform);
      transform.get(origin);

      return new LineSegment3d(new Point3d(origin), getPoint(index, range));
   }

   public void packInterpolatedTransform(int index, RigidBodyTransform target)
   {
      transformInterpolationCalculator.computeInterpolation(worldTransformStart, worldTransformEnd, target, index / (double) (params.pointsPerSweep - 1));
   }

   public RigidBodyTransform getSweepTransform(int i)
   {
      if (i >= params.pointsPerSweep)
      {
         throw new RuntimeException("Index " + i + " greater than or equal to pointsPerSweep " + params.pointsPerSweep);
      }

      double yawPerIndex = (params.sweepYawMax - params.sweepYawMin) / (params.pointsPerSweep - 1);

      if (Double.isNaN(yawPerIndex))
      {
         throw new RuntimeException("bad sweep parameters");
      }

      RigidBodyTransform sweepTransform = new RigidBodyTransform();
      sweepTransform.rotZ(params.sweepYawMin + yawPerIndex * i);

      return sweepTransform;
   }

   public long getScanTimeStamp() {
	   return params.getTimestamp();
   }

   public long getScanEndTime()
   {
      return params.getTimestamp() + params.getScanTimeNanos();
   }
}
