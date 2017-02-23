package us.ihmc.robotics.math.trajectories.waypoints;

import java.text.DecimalFormat;
import java.text.NumberFormat;

import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.Tuple4DReadOnly;
import us.ihmc.robotics.geometry.frameObjects.FrameEuclideanWaypoint;
import us.ihmc.robotics.geometry.frameObjects.FrameSE3Waypoint;
import us.ihmc.robotics.geometry.frameObjects.FrameSO3Waypoint;
import us.ihmc.robotics.geometry.transformables.EuclideanWaypoint;
import us.ihmc.robotics.geometry.transformables.SE3Waypoint;
import us.ihmc.robotics.geometry.transformables.SO3Waypoint;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public abstract class WaypointToStringTools
{
   public static String waypointToString(FrameEuclideanWaypoint frameEuclideanWaypoint)
   {
      EuclideanWaypoint simpleWaypoint = frameEuclideanWaypoint.getGeometryObject();
      if (simpleWaypoint.getNumberFormat() == null)
         simpleWaypoint.setNumberFormat(createNumberFormat());
      Point3DReadOnly position = simpleWaypoint.getPosition();
      Vector3DReadOnly linearVelocity = simpleWaypoint.getLinearVelocity();
      ReferenceFrame referenceFrame = frameEuclideanWaypoint.getReferenceFrame();
      return waypointToString(position, linearVelocity, referenceFrame, simpleWaypoint.getNumberFormat());
   }

   public static String waypointToString(FrameSO3Waypoint frameSO3Waypoint)
   {
      SO3Waypoint simpleWaypoint = frameSO3Waypoint.getGeometryObject();
      if (simpleWaypoint.getNumberFormat() == null)
         simpleWaypoint.setNumberFormat(createNumberFormat());
      QuaternionReadOnly orientation = simpleWaypoint.getOrientation();
      Vector3DReadOnly angularVelocity = simpleWaypoint.getAngularVelocity();
      ReferenceFrame referenceFrame = frameSO3Waypoint.getReferenceFrame();
      return waypointToString(orientation, angularVelocity, referenceFrame, simpleWaypoint.getNumberFormat());
   }

   public static String waypointToString(FrameSE3Waypoint frameSE3Waypoint)
   {
      SE3Waypoint simpleWaypoint = frameSE3Waypoint.getGeometryObject();
      if (simpleWaypoint.getNumberFormat() == null)
         simpleWaypoint.setNumberFormat(createNumberFormat());
      EuclideanWaypoint euclideanWaypoint = simpleWaypoint.getEuclideanWaypoint();
      Point3DReadOnly position = euclideanWaypoint.getPosition();
      SO3Waypoint so3Waypoint = simpleWaypoint.getSO3Waypoint();
      QuaternionReadOnly orientation = so3Waypoint.getOrientation();
      Vector3DReadOnly linearVelocity = euclideanWaypoint.getLinearVelocity();
      Vector3DReadOnly angularVelocity = so3Waypoint.getAngularVelocity();
      ReferenceFrame referenceFrame = frameSE3Waypoint.getReferenceFrame();
      return waypointToString(position, orientation, linearVelocity, angularVelocity, referenceFrame, simpleWaypoint.getNumberFormat());
   }

   public static String waypointToString(EuclideanWaypoint simpleEuclideanWaypoint)
   {
      if (simpleEuclideanWaypoint.getNumberFormat() == null)
         simpleEuclideanWaypoint.setNumberFormat(createNumberFormat());
      Point3DReadOnly position = simpleEuclideanWaypoint.getPosition();
      Vector3DReadOnly linearVelocity = simpleEuclideanWaypoint.getLinearVelocity();
      return waypointToString(position, linearVelocity, simpleEuclideanWaypoint.getNumberFormat());
   }

   public static String waypointToString(SO3Waypoint simpleSO3Waypoint)
   {
      if (simpleSO3Waypoint.getNumberFormat() == null)
         simpleSO3Waypoint.setNumberFormat(createNumberFormat());
      QuaternionReadOnly orientation = simpleSO3Waypoint.getOrientation();
      Vector3DReadOnly angularVelocity = simpleSO3Waypoint.getAngularVelocity();
      return waypointToString(orientation, angularVelocity, simpleSO3Waypoint.getNumberFormat());
   }

   public static String waypointToString(SE3Waypoint simpleSE3Waypoint)
   {
      if (simpleSE3Waypoint.getNumberFormat() == null)
         simpleSE3Waypoint.setNumberFormat(createNumberFormat());
      Point3DReadOnly position = simpleSE3Waypoint.getEuclideanWaypoint().getPosition();
      QuaternionReadOnly orientation = simpleSE3Waypoint.getSO3Waypoint().getOrientation();
      Vector3DReadOnly linearVelocity = simpleSE3Waypoint.getEuclideanWaypoint().getLinearVelocity();
      Vector3DReadOnly angularVelocity = simpleSE3Waypoint.getSO3Waypoint().getAngularVelocity();
      return waypointToString(position, orientation, linearVelocity, angularVelocity, simpleSE3Waypoint.getNumberFormat());
   }

   private static DecimalFormat createNumberFormat()
   {
      return new DecimalFormat(" 0.00;-0.00");
   }

   public static String waypointToString(Point3DReadOnly position, Vector3DReadOnly linearVelocity, NumberFormat format)
   {
      return waypointToString(position, linearVelocity, null, format);
   }

   public static String waypointToString(Point3DReadOnly position, Vector3DReadOnly linearVelocity, ReferenceFrame referenceFrame, NumberFormat format)
   {
      String referenceFrameToString = referenceFrame == null ? "" : ", " + referenceFrame.getName();
      return "Euclidean waypoint: " + "[" + positionToString(position, format) + ", " + linearVelocityToString(linearVelocity, format) + referenceFrameToString
            + "].";
   }

   public static String waypointToString(QuaternionReadOnly orientation, Vector3DReadOnly angularVelocity, NumberFormat format)
   {
      return waypointToString(orientation, angularVelocity, null, format);
   }

   public static String waypointToString(QuaternionReadOnly orientation, Vector3DReadOnly angularVelocity, ReferenceFrame referenceFrame, NumberFormat format)
   {
      String referenceFrameToString = referenceFrame == null ? "" : ", " + referenceFrame.getName();
      return "SO3 waypoint: " + "[" + orientationToString(orientation, format) + ", " + angularVelocityToString(angularVelocity, format)
            + referenceFrameToString + "].";
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
            + linearVelocityToString(linearVelocity, format) + ", " + angularVelocityToString(angularVelocity, format) + referenceFrameToString + "].";
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
}
