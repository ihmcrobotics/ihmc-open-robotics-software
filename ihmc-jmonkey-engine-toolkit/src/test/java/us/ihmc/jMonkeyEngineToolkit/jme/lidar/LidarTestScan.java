package us.ihmc.jMonkeyEngineToolkit.jme.lidar;

import java.util.ArrayList;
import java.util.Arrays;

import org.apache.commons.lang3.ArrayUtils;

import us.ihmc.euclid.geometry.Line3D;
import us.ihmc.euclid.geometry.LineSegment3D;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.transform.QuaternionBasedTransform;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.euclid.tuple3D.Vector3D;

public class LidarTestScan
{
   public LidarTestParameters params;
   public RigidBodyTransform worldTransformStart;
   public RigidBodyTransform worldTransformEnd;
   public RigidBodyTransform localTransformStart;
   public RigidBodyTransform localTransformEnd;
   public RigidBodyTransform averageTransform;

   public float[] ranges;
   public int sensorId;

   public LidarTestScan()
   {
   }

   public LidarTestScan(LidarTestParameters params, float[] ranges, int sensorId)
   {
      this.params = params;
      this.ranges = ranges;
      this.sensorId = sensorId;
   }

   public LidarTestScan(LidarTestParameters params, RigidBodyTransform start, RigidBodyTransform end, float[] ranges, int sensorId)
   {
      this.params = params;
      setWorldTransforms(start, end);
      this.ranges = ranges;
      this.sensorId = sensorId;
   }

   public LidarTestScan(LidarTestParameters params, RigidBodyTransform start, RigidBodyTransform end, float[] ranges)
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
    *
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

   public LidarTestScan getCopy()
   {
      return new LidarTestScan(getParams(), ranges.clone(), sensorId);
   }

   public int getSensorId()
   {
      return sensorId;
   }

   public LidarTestScan flipNew()
   {
      float[] flippedRanges = Arrays.copyOf(ranges, ranges.length);

      ArrayUtils.reverse(flippedRanges);

      return new LidarTestScan(getParams(), getEndTransform(), getStartTransform(), flippedRanges, sensorId);
   }

   public LidarTestParameters getParams()
   {
      return params;
   }

   /* SETTERS */

   public void setWorldTransforms(RigidBodyTransform start, RigidBodyTransform end)
   {
      this.worldTransformStart = start;
      this.worldTransformEnd = end;
      QuaternionBasedTransform start2 = new QuaternionBasedTransform(start);
      QuaternionBasedTransform end2 = new QuaternionBasedTransform(end);
      QuaternionBasedTransform interpolated = new QuaternionBasedTransform();
      interpolated.interpolate(start2, end2, 0.5);
      this.averageTransform = new RigidBodyTransform(interpolated);
   }

   public LidarTestParameters getScanParameters()
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

   public Line3D getRay(int index)
   {
      LineSegment3D unitSegment = getLineSegment(index, 1.0f);

      return new Line3D(unitSegment.getFirstEndpoint(), unitSegment.getDirection(true));
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
      QuaternionBasedTransform start2 = new QuaternionBasedTransform(worldTransformStart);
      QuaternionBasedTransform end2 = new QuaternionBasedTransform(worldTransformEnd);
      QuaternionBasedTransform interpolated = new QuaternionBasedTransform();
      interpolated.interpolate(start2, end2, index / (double) (params.getScansPerSweep() - 1));
      target.set(interpolated);
   }

   public RigidBodyTransform getSweepTransform(int i)
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

   public boolean epsilonEquals(LidarTestScan other, double transformEpsilon, float rangesEpsilon)
   {
      if (!this.worldTransformStart.epsilonEquals(other.worldTransformStart, transformEpsilon))
         return false;
      if (!this.worldTransformEnd.epsilonEquals(other.worldTransformEnd, transformEpsilon))
         return false;

      for (int i = 0; i < this.size() && i < other.size(); i++)
      {
         double value = this.getRange(i);
         double value1 = other.getRange(i);
         if ((value > 0 && value < Double.POSITIVE_INFINITY) && (value1 > 0 && value1 < Double.POSITIVE_INFINITY))
         {
            if (!EuclidCoreTools.epsilonEquals(ranges[i], other.ranges[i], rangesEpsilon))
               return false;
         }
      }
      return true;
   }
}
