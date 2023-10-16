package us.ihmc.robotics.interaction;

import us.ihmc.euclid.geometry.interfaces.Line3DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.shape.primitives.Ellipsoid3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.robotics.referenceFrames.MutableReferenceFrame;

public class EllipsoidRayIntersection
{
   private final Ellipsoid3D ellipsoid = new Ellipsoid3D();
   private final FramePoint3D firstIntersectionToPack = new FramePoint3D();
   private final FramePoint3D secondIntersectionToPack = new FramePoint3D();
   private final FramePose3D tempFramePose = new FramePose3D();
   private final MutableReferenceFrame ellipsoidFrame = new MutableReferenceFrame(ReferenceFrame.getWorldFrame());
   private final FramePoint3D pickRayFrameOrigin = new FramePoint3D();
   private final FrameVector3D pickRayFrameDirection = new FrameVector3D();
   private boolean intersects = false;

   public void update(double radiusX,
                      double radiusY,
                      double radiusZ,
                      Point3DReadOnly positionToParent,
                      Orientation3DReadOnly orientationToParent,
                      ReferenceFrame parentFrame)
   {
      ellipsoid.setToZero();
      ellipsoid.setRadiusX(radiusX);
      ellipsoid.setRadiusY(radiusY);
      ellipsoid.setRadiusZ(radiusZ);

      tempFramePose.setIncludingFrame(parentFrame, positionToParent, orientationToParent);
      tempFramePose.changeFrame(ReferenceFrame.getWorldFrame());
      tempFramePose.get(ellipsoidFrame.getTransformToParent());
      ellipsoidFrame.getReferenceFrame().update();
   }

   public boolean intersect(Line3DReadOnly pickRay)
   {
      pickRayFrameOrigin.setIncludingFrame(ReferenceFrame.getWorldFrame(), pickRay.getPoint());
      pickRayFrameDirection.setIncludingFrame(ReferenceFrame.getWorldFrame(), pickRay.getDirection());

      pickRayFrameOrigin.changeFrame(ellipsoidFrame.getReferenceFrame());
      pickRayFrameDirection.changeFrame(ellipsoidFrame.getReferenceFrame());

      firstIntersectionToPack.setToZero(ellipsoidFrame.getReferenceFrame());
      secondIntersectionToPack.setToZero(ellipsoidFrame.getReferenceFrame());

      int numberOfIntersections = EuclidGeometryTools.intersectionBetweenRay3DAndEllipsoid3D(ellipsoid.getRadiusX(),
                                                                                             ellipsoid.getRadiusY(),
                                                                                             ellipsoid.getRadiusZ(),
                                                                                             pickRayFrameOrigin,
                                                                                             pickRayFrameDirection,
                                                                                             firstIntersectionToPack,
                                                                                             secondIntersectionToPack);

      intersects = numberOfIntersections == 2;

      if (intersects)
      {
         firstIntersectionToPack.changeFrame(ReferenceFrame.getWorldFrame());
         secondIntersectionToPack.changeFrame(ReferenceFrame.getWorldFrame());
      }

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
