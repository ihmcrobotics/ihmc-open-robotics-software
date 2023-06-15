package us.ihmc.robotics.interaction;

import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.geometry.Line3D;
import us.ihmc.euclid.geometry.Plane3D;
import us.ihmc.euclid.geometry.interfaces.Line3DReadOnly;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

public class Line3DMouseDragAlgorithm
{
   private final Line3D dragLine = new Line3D();
   private final Plane3D perpendicularPlane = new Plane3D();
   private final Point3D closestPointToPickRay = new Point3D();
   private final Vector3D axisMoveVector = new Vector3D();

   public Vector3DReadOnly calculate(Line3DReadOnly pickRay,
                                     Point3D pointOnLine,
                                     RotationMatrix lineOrientationInObjectFrame,
                                     RigidBodyTransformReadOnly objectTransform)
   {
      dragLine.getPoint().set(pointOnLine);

      dragLine.getDirection().set(Axis3D.Z);
      lineOrientationInObjectFrame.transform(dragLine.getDirection());
      objectTransform.getRotation().transform(dragLine.getDirection());

      perpendicularPlane.set(dragLine.getPoint(), dragLine.getDirection());

      pickRay.closestPointsWith(dragLine, null, closestPointToPickRay);
      double distanceToMove = perpendicularPlane.signedDistance(closestPointToPickRay);
      axisMoveVector.set(dragLine.getDirection());
      axisMoveVector.scale(distanceToMove);
      return axisMoveVector;
   }
}
