package us.ihmc.robotics.lidar;

import java.util.ArrayList;
import java.util.Arrays;

import org.apache.commons.lang3.ArrayUtils;

import com.esotericsoftware.kryo.serializers.FieldSerializer.Optional;

import us.ihmc.euclid.geometry.LineSegment3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.geometry.Ray3d;
import us.ihmc.robotics.kinematics.TransformInterpolationCalculator;

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

   public ArrayList<Point3D> getAllPoints()
   {
      ArrayList<Point3D> points = new ArrayList<Point3D>();
      for (int i = 0; i < ranges.length; i++)
      {
         points.add(getPoint(i, ranges[i]));
      }

      return points;
   }

   public ArrayList<Point3D32> getAllPoints3f()
   {
      ArrayList<Point3D32> points = new ArrayList<Point3D32>();
      for (int i = 0; i < ranges.length; i++)
      {
         points.add(getPoint3f(i, ranges[i]));
      }

      return points;
   }

   public Point3D[] toPointArray()
   {
      Point3D[] points = new Point3D[ranges.length];
      for (int i = 0; i < ranges.length; i++)
      {
         points[i] = getPoint(i);
      }
      return points;
   }

   public int size()
   {
      return ranges.length;
   }

   public Point3D getPoint(int index)
   {
      return getPoint(index, ranges[index]);
   }

   public Point3D32 getPoint3f(int index)
   {
      return getPoint3f(index, ranges[index]);
   }

   public LineSegment3D getLineSegment(int index)
   {
      return getLineSegment(index, ranges[index]);
   }

   /**
    * @param index of the point with respect to {@link #getPoint(int)}
    * @return
    */
   public float getRange(int i)
   {
      return ranges[i];
   }

   public float[] getRanges()
   {
      return ranges;
   }

   public LidarScan getCopy()
   {
      return new LidarScan(getParams(), ranges.clone(), sensorId);
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
      LineSegment3D unitSegment = getLineSegment(index, 1.0f);

      return new Ray3d(unitSegment.getFirstEndpoint(), unitSegment.getDirection(true));
   }

   /* PRIVATE/PROTECTED FUNCTIONS */
   protected Point3D getPoint(int index, float range)
   {
      Point3D p = new Point3D(range, 0.0, 0.0);
      RigidBodyTransform transform = new RigidBodyTransform();
      getInterpolatedTransform(index, transform);
      transform.multiply(getSweepTransform(index));
      transform.transform(p);

      return p;
   }

   protected Point3D32 getPoint3f(int index, float range)
   {
      Point3D32 p = new Point3D32(range, 0.0f, 0.0f);
      RigidBodyTransform transform = new RigidBodyTransform();
      getInterpolatedTransform(index, transform);
      transform.multiply(getSweepTransform(index));
      transform.transform(p);

      return p;
   }

   protected LineSegment3D getLineSegment(int index, float range)
   {
      Vector3D origin = new Vector3D();
      RigidBodyTransform transform = new RigidBodyTransform();
      getInterpolatedTransform(index, transform);
      transform.getTranslation(origin);

      return new LineSegment3D(new Point3D(origin), getPoint(index, range));
   }

   public void getInterpolatedTransform(int index, RigidBodyTransform target)
   {
      transformInterpolationCalculator.computeInterpolation(worldTransformStart, worldTransformEnd, target, index / (double) (params.pointsPerSweep - 1));
   }

   public RigidBodyTransform getSweepTransform(int i)
   {
      if (i >= params.pointsPerSweep * params.scanHeight)
      {
         throw new IndexOutOfBoundsException("Index " + i + " greater than or equal to pointsPerSweep " + params.pointsPerSweep + " * scanHeight "
               + params.scanHeight);
      }

      double yawPerIndex = (params.sweepYawMax - params.sweepYawMin) / (params.pointsPerSweep - 1);
      double pitchPerIndex = (params.heightPitchMax - params.heightPitchMin) / (params.scanHeight - 1);

      RigidBodyTransform sweepTransform = new RigidBodyTransform();
      if (params.pointsPerSweep > 1)
      {
         //sweepTransform.rotZ(params.sweepYawMin + yawPerIndex * i);
         sweepTransform.setRotationYawAndZeroTranslation(params.sweepYawMin + yawPerIndex * (i % params.pointsPerSweep));
      }
      if (params.scanHeight > 1)
      {
         sweepTransform.setRotationPitchAndZeroTranslation(params.heightPitchMin + pitchPerIndex * (i / params.pointsPerSweep));
      }

      return sweepTransform;
   }

   public long getScanTimeStamp()
   {
      return params.getTimestamp();
   }

   public long getScanEndTime()
   {
      return params.getTimestamp() + params.getScanTimeNanos();
   }
}
