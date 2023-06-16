package us.ihmc.robotics.interaction;

import us.ihmc.euclid.geometry.interfaces.Line3DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.robotics.referenceFrames.ReferenceFrameMissingTools;

public class BoxRayIntersection
{
   private final RigidBodyTransform boxToWorldTransform = new RigidBodyTransform();
   private final ReferenceFrame boxFrame = ReferenceFrameMissingTools.constructFrameWithChangingTransformToParent(ReferenceFrame.getWorldFrame(),
                                                                                                                  boxToWorldTransform);
   private final FramePoint3D rayOrigin = new FramePoint3D();
   private final FrameVector3D rayDirection = new FrameVector3D();
   private final Point3D boundingBoxMin = new Point3D();
   private final Point3D boundingBoxMax = new Point3D();
   private final FramePoint3D firstIntersectionToPack = new FramePoint3D();
   private final FramePoint3D secondIntersectionToPack = new FramePoint3D();
   private boolean intersects = false;

   public boolean intersect(double sizeX,
                            double sizeY,
                            double sizeZ,
                            RigidBodyTransform boxCenterToWorld,
                            Line3DReadOnly pickRay)
   {
      boxToWorldTransform.set(boxCenterToWorld);
      boxFrame.update();

      rayOrigin.setIncludingFrame(ReferenceFrame.getWorldFrame(), pickRay.getPoint());
      rayDirection.setIncludingFrame(ReferenceFrame.getWorldFrame(), pickRay.getDirection());

      rayOrigin.changeFrame(boxFrame);
      rayDirection.changeFrame(boxFrame);

      boundingBoxMin.set(-sizeX / 2.0, -sizeY / 2.0, -sizeZ / 2.0);
      boundingBoxMax.set(sizeX / 2.0, sizeY / 2.0, sizeZ / 2.0);

      firstIntersectionToPack.setToZero(boxFrame);
      secondIntersectionToPack.setToZero(boxFrame);
      int numberOfIntersections = EuclidGeometryTools.intersectionBetweenRay3DAndBoundingBox3D(boundingBoxMin,
                                                                                               boundingBoxMax,
                                                                                               rayOrigin,
                                                                                               rayDirection,
                                                                                               firstIntersectionToPack,
                                                                                               secondIntersectionToPack);
      firstIntersectionToPack.changeFrame(ReferenceFrame.getWorldFrame());
      secondIntersectionToPack.changeFrame(ReferenceFrame.getWorldFrame());
      intersects = numberOfIntersections == 2;
      return intersects;
   }

   public Point3DReadOnly getFirstIntersectionToPack()
   {
      return firstIntersectionToPack;
   }

   public Point3DReadOnly getSecondIntersectionToPack()
   {
      return secondIntersectionToPack;
   }

   public boolean getIntersects()
   {
      return intersects;
   }
}
