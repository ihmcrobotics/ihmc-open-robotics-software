package us.ihmc.gdx.ui.gizmo;

import us.ihmc.euclid.geometry.interfaces.Line3DReadOnly;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;

public class BoxRayIntersection
{

   private final RigidBodyTransform boxToWorldTransform = new RigidBodyTransform();
   private final ReferenceFrame boxFrame = ReferenceFrameTools.constructFrameWithChangingTransformToParent("boxFrame",
                                                                                                           ReferenceFrame.getWorldFrame(),
                                                                                                           boxToWorldTransform);
   private final FramePoint3D rayOrigin = new FramePoint3D();
   private final FrameVector3D rayDirection = new FrameVector3D();
   private final Point3D boundingBoxMin = new Point3D();
   private final Point3D boundingBoxMax = new Point3D();
   private Point3DBasics firstIntersectionToPack;
   private Point3DBasics secondIntersectionToPack;

   public boolean intersect(double sizeX, double sizeY, double sizeZ, Pose3DReadOnly poseInWorld, Line3DReadOnly pickRay)
   {
      boxToWorldTransform.set(poseInWorld.getOrientation(), poseInWorld.getPosition());
      boxFrame.update();

      rayOrigin.setIncludingFrame(ReferenceFrame.getWorldFrame(), pickRay.getPoint());
      rayDirection.setIncludingFrame(ReferenceFrame.getWorldFrame(), pickRay.getDirection());

      rayOrigin.changeFrame(boxFrame);
      rayDirection.changeFrame(boxFrame);

      boundingBoxMin.set(-sizeX / 2.0, -sizeY / 2.0, -sizeZ / 2.0);
      boundingBoxMax.set(sizeX / 2.0, sizeY / 2.0, sizeZ / 2.0);

      int numberOfIntersections = EuclidGeometryTools.intersectionBetweenRay3DAndBoundingBox3D(boundingBoxMin,
                                                                                               boundingBoxMax,
                                                                                               rayOrigin,
                                                                                               rayDirection,
                                                                                               firstIntersectionToPack,
                                                                                               secondIntersectionToPack);
      return numberOfIntersections == 2;
   }

   public Point3DBasics getFirstIntersectionToPack()
   {
      return firstIntersectionToPack;
   }

   public Point3DBasics getSecondIntersectionToPack()
   {
      return secondIntersectionToPack;
   }
}
