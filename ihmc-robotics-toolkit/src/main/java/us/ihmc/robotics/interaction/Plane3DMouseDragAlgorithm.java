package us.ihmc.robotics.interaction;

import us.ihmc.euclid.geometry.Plane3D;
import us.ihmc.euclid.geometry.interfaces.Line3DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

public class Plane3DMouseDragAlgorithm
{
   private final Plane3D dragPlane = new Plane3D();
   private final Point3D dragPoint = new Point3D();
   private final Vector3D dragVector = new Vector3D();

   public Vector3DReadOnly calculate(Line3DReadOnly pickRay, Point3DReadOnly pointOnPlane, Vector3DReadOnly planeNormal)
   {
      dragPlane.set(pointOnPlane, planeNormal);

      dragPlane.intersectionWith(pickRay, dragPoint);
      dragVector.sub(dragPoint, pointOnPlane);

      return dragVector;
   }
}
