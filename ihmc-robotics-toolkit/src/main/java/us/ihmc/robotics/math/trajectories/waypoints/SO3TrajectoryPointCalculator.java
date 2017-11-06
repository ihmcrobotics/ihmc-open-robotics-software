package us.ihmc.robotics.math.trajectories.waypoints;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.rotationConversion.YawPitchRollConversion;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.lists.RecyclingArrayList;
import us.ihmc.robotics.math.trajectories.HermiteCurveBasedOrientationTrajectoryGenerator;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class SO3TrajectoryPointCalculator
{
   public double firstTrajectoryPointTime;

   public List<Quaternion> trajectoryPointsOrientation = new ArrayList<>();
   public List<Double> trajectoryPointsTime = new ArrayList<>();

   public List<Vector3D> trajectoryPointsAngularVelocity = new ArrayList<>();

   public SO3TrajectoryPointCalculator()
   {

   }

   public void clear()
   {
      trajectoryPointsOrientation.clear();
      trajectoryPointsTime.clear();
   }

   public void setFirstTrajectoryPointTime(double firstTrajectoryPointTime)
   {
      this.firstTrajectoryPointTime = firstTrajectoryPointTime;
   }

   public void appendTrajectoryPointOrientation(double time, Quaternion quaternion)
   {
      trajectoryPointsOrientation.add(quaternion);
      trajectoryPointsTime.add(time);
   }

   public void compute()
   {
      EuclideanTrajectoryPointCalculator euclideanTrajectoryPointCalculator = new EuclideanTrajectoryPointCalculator();

      int numberOfTrajectoryPoints = trajectoryPointsOrientation.size();
      for (int i = 0; i < numberOfTrajectoryPoints; i++)
      {
         Vector3D orientation = new Vector3D();
         // this conversion does not provide over 90 degree for pitch angle.
         //YawPitchRollConversion.convertQuaternionToYawPitchRoll(trajectoryPointsOrientation.get(i), orientation);
         convertQuaternionToYawPitchRoll(trajectoryPointsOrientation.get(i), orientation);

         double time = firstTrajectoryPointTime + trajectoryPointsTime.get(i);
         euclideanTrajectoryPointCalculator.appendTrajectoryPoint(time, new Point3D(orientation));         
      }

      euclideanTrajectoryPointCalculator.computeTrajectoryPointVelocities(false);

      RecyclingArrayList<FrameEuclideanTrajectoryPoint> trajectoryPoints = euclideanTrajectoryPointCalculator.getTrajectoryPoints();

      trajectoryPointsAngularVelocity.clear();
      for (int i = 0; i < numberOfTrajectoryPoints; i++)
      {
         Vector3D angularVelocityYawPitchRoll = new Vector3D();
         trajectoryPoints.get(i).getLinearVelocity(angularVelocityYawPitchRoll);
         trajectoryPointsAngularVelocity.add(angularVelocityYawPitchRoll);
      }
   }

   public List<Vector3D> getTrajectoryPointsAngularVelocity()
   {
      return trajectoryPointsAngularVelocity;
   }

   public Vector3D getAngularVelocity(double time)
   {
      Vector3D angularVelocity = new Vector3D();

      /*
       * For debug in SO3TrajectoryPointCalculatorVisualizer
       */

      return angularVelocity;
   }

   public Quaternion getOrientation(double time)
   {
      int arrayIndex = 0;

      int numberOfTrajectoryPoints = trajectoryPointsTime.size();
      for (int i = 0; i < numberOfTrajectoryPoints - 1; i++)
      {
         if (trajectoryPointsTime.get(i) <= time && time <= trajectoryPointsTime.get(i + 1))
            arrayIndex = i;
      }

      double localStartTime = trajectoryPointsTime.get(arrayIndex);
      double localEndTime = trajectoryPointsTime.get(arrayIndex + 1);

      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

      FrameOrientation initialOrientation = new FrameOrientation(worldFrame, trajectoryPointsOrientation.get(arrayIndex));
      FrameVector3D initialAngularVelocity = new FrameVector3D(worldFrame, trajectoryPointsAngularVelocity.get(arrayIndex));
      FrameOrientation finalOrientation = new FrameOrientation(worldFrame, trajectoryPointsOrientation.get(arrayIndex + 1));
      FrameVector3D finalAngularVelocity = new FrameVector3D(worldFrame, trajectoryPointsAngularVelocity.get(arrayIndex + 1));

      YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
      HermiteCurveBasedOrientationTrajectoryGenerator traj1 = new HermiteCurveBasedOrientationTrajectoryGenerator("traj1", worldFrame, registry);
      traj1.setInitialConditions(initialOrientation, initialAngularVelocity);
      traj1.setFinalConditions(finalOrientation, finalAngularVelocity);
      traj1.setTrajectoryTime(localEndTime - localStartTime);
      traj1.setNumberOfRevolutions(0);
      traj1.initialize();

      traj1.compute(time - localStartTime);
      FrameOrientation interpolatedOrientation = new FrameOrientation(worldFrame);
      traj1.getOrientation(interpolatedOrientation);

      Quaternion orientation = new Quaternion(interpolatedOrientation.getQuaternion());

      return orientation;
   }

   public Vector3D getOrientationYawPitchRoll(double time)
   {
      Vector3D angularVelocity = new Vector3D();
      YawPitchRollConversion.convertQuaternionToYawPitchRoll(getOrientation(time), angularVelocity);

      return angularVelocity;
   }
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   /**
    * temporary
    */
   
   private static final double EPS = 1.0e-12;
   
   public static final void convertQuaternionToYawPitchRoll(QuaternionReadOnly quaternion, Vector3DBasics eulerAnglesToPack)
   {
      if (quaternion.containsNaN())
      {
         eulerAnglesToPack.setToNaN();
         return;
      }

      double qx = quaternion.getX();
      double qy = quaternion.getY();
      double qz = quaternion.getZ();
      double qs = quaternion.getS();

      double norm = quaternion.norm();
      if (norm < EPS)
      {
         eulerAnglesToPack.setToZero();
         return;
      }

      norm = 1.0 / norm;
      qx *= norm;
      qy *= norm;
      qz *= norm;
      qs *= norm;

      double pitch = computePitchFromQuaternionImpl(qx, qy, qz, qs);
      eulerAnglesToPack.setY(pitch);
      if (Double.isNaN(pitch))
      {
         eulerAnglesToPack.setToNaN();
      }
      else
      {
         eulerAnglesToPack.setZ(computeYawFromQuaternionImpl(qx, qy, qz, qs));
         eulerAnglesToPack.setX(computeRollFromQuaternionImpl(qx, qy, qz, qs));
      }
   }
   
   static double computeRollFromQuaternionImpl(double qx, double qy, double qz, double qs)
   {
      return Math.atan2(2.0 * (qy * qz + qx * qs), 1.0 - 2.0 * (qx * qx + qy * qy));
   }
   
   static double computePitchFromQuaternionImpl(double qx, double qy, double qz, double qs)
   {
      double pitchArgument = 2.0 * (qs * qy - qx * qz);

      double pitch = Math.asin(pitchArgument);
      return pitch;
   }
   
   static double computeYawFromQuaternionImpl(double qx, double qy, double qz, double qs)
   {
      return Math.atan2(2.0 * (qx * qy + qz * qs), 1.0 - 2.0 * (qy * qy + qz * qz));
   }
}
