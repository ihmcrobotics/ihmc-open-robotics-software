package us.ihmc.robotics.math.trajectories.waypoints;

import java.text.DecimalFormat;
import java.text.NumberFormat;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Tuple3d;
import javax.vecmath.Tuple4d;
import javax.vecmath.Vector3d;

import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public abstract class WaypointToStringTools
{
   public static String waypointToString(FrameEuclideanWaypoint frameEuclideanWaypoint)
   {
      SimpleEuclideanWaypoint simpleWaypoint = frameEuclideanWaypoint.getSimpleWaypoint();
      if (simpleWaypoint.getNumberFormat() == null)
         simpleWaypoint.setNumberFormat(createNumberFormat());
      Point3d position = simpleWaypoint.getPosition();
      Vector3d linearVelocity = simpleWaypoint.getLinearVelocity();
      ReferenceFrame referenceFrame = frameEuclideanWaypoint.getReferenceFrame();
      return waypointToString(position, linearVelocity, referenceFrame, simpleWaypoint.getNumberFormat());
   }

   public static String waypointToString(FrameSO3Waypoint frameSO3Waypoint)
   {
      SimpleSO3Waypoint simpleWaypoint = frameSO3Waypoint.getSimpleWaypoint();
      if (simpleWaypoint.getNumberFormat() == null)
         simpleWaypoint.setNumberFormat(createNumberFormat());
      Quat4d orientation = simpleWaypoint.getOrientation();
      Vector3d angularVelocity = simpleWaypoint.getAngularVelocity();
      ReferenceFrame referenceFrame = frameSO3Waypoint.getReferenceFrame();
      return waypointToString(orientation, angularVelocity, referenceFrame, simpleWaypoint.getNumberFormat());
   }

   public static String waypointToString(FrameSE3Waypoint frameSE3Waypoint)
   {
      SimpleSE3Waypoint simpleWaypoint = frameSE3Waypoint.getSimpleWaypoint();
      if (simpleWaypoint.getNumberFormat() == null)
         simpleWaypoint.setNumberFormat(createNumberFormat());
      SimpleEuclideanWaypoint euclideanWaypoint = simpleWaypoint.getEuclideanWaypoint();
      Point3d position = euclideanWaypoint.getPosition();
      SimpleSO3Waypoint so3Waypoint = simpleWaypoint.getSO3Waypoint();
      Quat4d orientation = so3Waypoint.getOrientation();
      Vector3d linearVelocity = euclideanWaypoint.getLinearVelocity();
      Vector3d angularVelocity = so3Waypoint.getAngularVelocity();
      ReferenceFrame referenceFrame = frameSE3Waypoint.getReferenceFrame();
      return waypointToString(position, orientation, linearVelocity, angularVelocity, referenceFrame, simpleWaypoint.getNumberFormat());
   }

   public static String waypointToString(SimpleEuclideanWaypoint simpleEuclideanWaypoint)
   {
      if (simpleEuclideanWaypoint.getNumberFormat() == null)
         simpleEuclideanWaypoint.setNumberFormat(createNumberFormat());
      Point3d position = simpleEuclideanWaypoint.getPosition();
      Vector3d linearVelocity = simpleEuclideanWaypoint.getLinearVelocity();
      return waypointToString(position, linearVelocity, simpleEuclideanWaypoint.getNumberFormat());
   }

   public static String waypointToString(SimpleSO3Waypoint simpleSO3Waypoint)
   {
      if (simpleSO3Waypoint.getNumberFormat() == null)
         simpleSO3Waypoint.setNumberFormat(createNumberFormat());
      Quat4d orientation = simpleSO3Waypoint.getOrientation();
      Vector3d angularVelocity = simpleSO3Waypoint.getAngularVelocity();
      return waypointToString(orientation, angularVelocity, simpleSO3Waypoint.getNumberFormat());
   }

   public static String waypointToString(SimpleSE3Waypoint simpleSE3Waypoint)
   {
      if (simpleSE3Waypoint.getNumberFormat() == null)
         simpleSE3Waypoint.setNumberFormat(createNumberFormat());
      Point3d position = simpleSE3Waypoint.getEuclideanWaypoint().getPosition();
      Quat4d orientation = simpleSE3Waypoint.getSO3Waypoint().getOrientation();
      Vector3d linearVelocity = simpleSE3Waypoint.getEuclideanWaypoint().getLinearVelocity();
      Vector3d angularVelocity = simpleSE3Waypoint.getSO3Waypoint().getAngularVelocity();
      return waypointToString(position, orientation, linearVelocity, angularVelocity, simpleSE3Waypoint.getNumberFormat());
   }

   private static DecimalFormat createNumberFormat()
   {
      return new DecimalFormat(" 0.00;-0.00");
   }

   public static String waypointToString(Point3d position, Vector3d linearVelocity, NumberFormat format)
   {
      return waypointToString(position, linearVelocity, null, format);
   }

   public static String waypointToString(Point3d position, Vector3d linearVelocity, ReferenceFrame referenceFrame, NumberFormat format)
   {
      String referenceFrameToString = referenceFrame == null ? "" : ", " + referenceFrame.getName();
      return "Euclidean waypoint: " + "[" + positionToString(position, format) + ", " + linearVelocityToString(linearVelocity, format) + referenceFrameToString
            + "].";
   }

   public static String waypointToString(Quat4d orientation, Vector3d angularVelocity, NumberFormat format)
   {
      return waypointToString(orientation, angularVelocity, null, format);
   }

   public static String waypointToString(Quat4d orientation, Vector3d angularVelocity, ReferenceFrame referenceFrame, NumberFormat format)
   {
      String referenceFrameToString = referenceFrame == null ? "" : ", " + referenceFrame.getName();
      return "SO3 waypoint: " + "[" + orientationToString(orientation, format) + ", " + angularVelocityToString(angularVelocity, format)
            + referenceFrameToString + "].";
   }

   public static String waypointToString(Point3d position, Quat4d orientation, Vector3d linearVelocity, Vector3d angularVelocity, NumberFormat format)
   {
      return waypointToString(position, orientation, linearVelocity, angularVelocity, null, format);
   }

   public static String waypointToString(Point3d position, Quat4d orientation, Vector3d linearVelocity, Vector3d angularVelocity, ReferenceFrame referenceFrame,
         NumberFormat format)
   {
      String referenceFrameToString = referenceFrame == null ? "" : ", " + referenceFrame.getName();
      return "SE3 waypoint: " + "[" + positionToString(position, format) + ", " + orientationToString(orientation, format) + ", "
            + linearVelocityToString(linearVelocity, format) + ", " + angularVelocityToString(angularVelocity, format) + referenceFrameToString + "].";
   }

   public static String positionToString(Point3d position, NumberFormat format)
   {
      return "position = " + tuple3dToString(position, format);
   }

   public static String orientationToString(Quat4d orientation, NumberFormat format)
   {
      return "orientation = " + tuple4dToString(orientation, format);
   }

   public static String linearVelocityToString(Vector3d linearVelocity, NumberFormat format)
   {
      return "linearVelocity = " + tuple3dToString(linearVelocity, format);
   }

   public static String angularVelocityToString(Vector3d angularVelocity, NumberFormat format)
   {
      return "angular velocity = " + tuple3dToString(angularVelocity, format);
   }

   public static String tuple3dToString(Tuple3d tuple, NumberFormat format)
   {
      String xToString = format.format(tuple.getX());
      String yToString = format.format(tuple.getY());
      String zToString = format.format(tuple.getZ());
      return "(" + xToString + ", " + yToString + ", " + zToString + ")";
   }

   public static String tuple4dToString(Tuple4d tuple, NumberFormat format)
   {
      String xToString = format.format(tuple.getX());
      String yToString = format.format(tuple.getY());
      String zToString = format.format(tuple.getZ());
      String sToString = format.format(tuple.getW());
      return "(" + xToString + ", " + yToString + ", " + zToString + ", " + sToString + ")";
   }
}
