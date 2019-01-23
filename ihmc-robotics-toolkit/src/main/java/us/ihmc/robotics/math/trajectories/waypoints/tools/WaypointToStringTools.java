package us.ihmc.robotics.math.trajectories.waypoints.tools;

import java.text.DecimalFormat;
import java.text.NumberFormat;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.Tuple4DReadOnly;
import us.ihmc.robotics.geometry.interfaces.EuclideanWaypointInterface;
import us.ihmc.robotics.geometry.interfaces.FrameEuclideanWaypointInterface;
import us.ihmc.robotics.geometry.interfaces.FrameSE3WaypointInterface;
import us.ihmc.robotics.geometry.interfaces.FrameSO3WaypointInterface;
import us.ihmc.robotics.geometry.interfaces.SE3WaypointInterface;
import us.ihmc.robotics.geometry.interfaces.SO3WaypointInterface;

public class WaypointToStringTools
{
   static final NumberFormat format = new DecimalFormat(" 0.000;-0.000");
   static final NumberFormat timeFormat = new DecimalFormat(" 0.00;-0.00");

   public static String waypointToString(FrameEuclideanWaypointInterface frameEuclideanWaypoint)
   {
      Point3DReadOnly position = frameEuclideanWaypoint.getPosition();
      Vector3DReadOnly linearVelocity = frameEuclideanWaypoint.getLinearVelocity();
      ReferenceFrame referenceFrame = frameEuclideanWaypoint.getReferenceFrame();
      return waypointToString(position, linearVelocity, referenceFrame, format);
   }

   public static String waypointToString(FrameSO3WaypointInterface frameSO3Waypoint)
   {
      QuaternionReadOnly orientation = frameSO3Waypoint.getOrientation();
      Vector3DReadOnly angularVelocity = frameSO3Waypoint.getAngularVelocity();
      ReferenceFrame referenceFrame = frameSO3Waypoint.getReferenceFrame();
      return waypointToString(orientation, angularVelocity, referenceFrame, format);
   }

   public static String waypointToString(FrameSE3WaypointInterface frameSE3Waypoint)
   {
      Point3DReadOnly position = frameSE3Waypoint.getPosition();
      QuaternionReadOnly orientation = frameSE3Waypoint.getOrientation();
      Vector3DReadOnly linearVelocity = frameSE3Waypoint.getLinearVelocity();
      Vector3DReadOnly angularVelocity = frameSE3Waypoint.getAngularVelocity();
      ReferenceFrame referenceFrame = frameSE3Waypoint.getReferenceFrame();
      return waypointToString(position, orientation, linearVelocity, angularVelocity, referenceFrame, format);
   }

   public static String waypointToString(EuclideanWaypointInterface simpleEuclideanWaypoint)
   {
      Point3DReadOnly position = simpleEuclideanWaypoint.getPosition();
      Vector3DReadOnly linearVelocity = simpleEuclideanWaypoint.getLinearVelocity();
      return waypointToString(position, linearVelocity, format);
   }

   public static String waypointToString(SO3WaypointInterface simpleSO3Waypoint)
   {
      QuaternionReadOnly orientation = simpleSO3Waypoint.getOrientation();
      Vector3DReadOnly angularVelocity = simpleSO3Waypoint.getAngularVelocity();
      return waypointToString(orientation, angularVelocity, format);
   }

   public static String waypointToString(SE3WaypointInterface simpleSE3Waypoint)
   {
      Point3DReadOnly position = simpleSE3Waypoint.getPosition();
      QuaternionReadOnly orientation = simpleSE3Waypoint.getOrientation();
      Vector3DReadOnly linearVelocity = simpleSE3Waypoint.getLinearVelocity();
      Vector3DReadOnly angularVelocity = simpleSE3Waypoint.getAngularVelocity();
      return waypointToString(position, orientation, linearVelocity, angularVelocity, format);
   }

   public static String waypointToString(Point3DReadOnly position, Vector3DReadOnly linearVelocity, NumberFormat format)
   {
      return waypointToString(position, linearVelocity, null, format);
   }

   public static String waypointToString(Point3DReadOnly position, Vector3DReadOnly linearVelocity, ReferenceFrame referenceFrame, NumberFormat format)
   {
      String referenceFrameToString = referenceFrame == null ? "" : ", " + referenceFrame.getName();
      return "Euclidean waypoint: " + "[" + positionToString(position, format) + ", " + linearVelocityToString(linearVelocity, format) + referenceFrameToString
            + "]";
   }

   public static String waypointToString(QuaternionReadOnly orientation, Vector3DReadOnly angularVelocity, NumberFormat format)
   {
      return waypointToString(orientation, angularVelocity, null, format);
   }

   public static String waypointToString(QuaternionReadOnly orientation, Vector3DReadOnly angularVelocity, ReferenceFrame referenceFrame, NumberFormat format)
   {
      String referenceFrameToString = referenceFrame == null ? "" : ", " + referenceFrame.getName();
      return "SO3 waypoint: " + "[" + orientationToString(orientation, format) + ", " + angularVelocityToString(angularVelocity, format)
            + referenceFrameToString + "]";
   }

   public static String waypointToString(Point3DReadOnly position, QuaternionReadOnly orientation, Vector3DReadOnly linearVelocity, Vector3DReadOnly angularVelocity, NumberFormat format)
   {
      return waypointToString(position, orientation, linearVelocity, angularVelocity, null, format);
   }

   public static String waypointToString(Point3DReadOnly position, QuaternionReadOnly orientation, Vector3DReadOnly linearVelocity, Vector3DReadOnly angularVelocity, ReferenceFrame referenceFrame,
         NumberFormat format)
   {
      String referenceFrameToString = referenceFrame == null ? "" : ", " + referenceFrame.getName();
      return "SE3 waypoint: " + "[" + positionToString(position, format) + ", " + orientationToString(orientation, format) + ", "
            + linearVelocityToString(linearVelocity, format) + ", " + angularVelocityToString(angularVelocity, format) + referenceFrameToString + "]";
   }

   public static String positionToString(Point3DReadOnly position, NumberFormat format)
   {
      return "position = " + tuple3dToString(position, format);
   }

   public static String orientationToString(QuaternionReadOnly orientation, NumberFormat format)
   {
      return "orientation = " + tuple4dToString(orientation, format);
   }

   public static String linearVelocityToString(Vector3DReadOnly linearVelocity, NumberFormat format)
   {
      return "linearVelocity = " + tuple3dToString(linearVelocity, format);
   }

   public static String angularVelocityToString(Vector3DReadOnly angularVelocity, NumberFormat format)
   {
      return "angular velocity = " + tuple3dToString(angularVelocity, format);
   }

   public static String tuple3dToString(Tuple3DReadOnly tuple, NumberFormat format)
   {
      String xToString = format.format(tuple.getX());
      String yToString = format.format(tuple.getY());
      String zToString = format.format(tuple.getZ());
      return "(" + xToString + ", " + yToString + ", " + zToString + ")";
   }

   public static String tuple4dToString(Tuple4DReadOnly tuple, NumberFormat format)
   {
      String xToString = format.format(tuple.getX());
      String yToString = format.format(tuple.getY());
      String zToString = format.format(tuple.getZ());
      String sToString = format.format(tuple.getS());
      return "(" + xToString + ", " + yToString + ", " + zToString + ", " + sToString + ")";
   }

   public static String format(double number)
   {
      return format.format(number);
   }

   public static String formatTime(double number)
   {
      return timeFormat.format(number);
   }
}
