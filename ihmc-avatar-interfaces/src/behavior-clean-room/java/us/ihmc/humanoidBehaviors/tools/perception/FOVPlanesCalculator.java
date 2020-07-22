package us.ihmc.humanoidBehaviors.tools.perception;

import us.ihmc.euclid.geometry.Plane3D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;

public class FOVPlanesCalculator
{
   private final ReferenceFrame cameraFrame;

   private final double verticalFOV;
   private final double horizontalFOV;

   private final FramePose3D tempFramePose3D = new FramePose3D();
   private final FramePoint3D tempFramePoint3D = new FramePoint3D();
   private final Vector3D tempNormal = new Vector3D();

   private final Plane3D planeTop;
   private final Plane3D planeBottom;
   private final Plane3D planeLeft;
   private final Plane3D planeRight;

   public FOVPlanesCalculator(double verticalFOV, double horizontalFOV, ReferenceFrame cameraFrame)
   {
      this.verticalFOV = verticalFOV;
      this.horizontalFOV = horizontalFOV;
      this.cameraFrame = cameraFrame;

      planeTop = new Plane3D();
      planeBottom = new Plane3D();
      planeLeft = new Plane3D();
      planeRight = new Plane3D();
   }

   /**
    * Must call update first.
    */
   public boolean isPointInView(Point3DReadOnly point)
   {
      return planeTop.isOnOrAbove(point) && planeBottom.isOnOrAbove(point) && planeLeft.isOnOrAbove(point) && planeRight.isOnOrAbove(point);
   }

   public void update()
   {
      updatePlaneToFrameWithParameters(planeTop, -verticalFOV / 2.0, 0.0, 0.0, -1.0);
      updatePlaneToFrameWithParameters(planeBottom, verticalFOV / 2.0, 0.0, 0.0, 1.0);
      updatePlaneToFrameWithParameters(planeLeft, 0.0, -horizontalFOV / 2.0, 1.0, 0.0);
      updatePlaneToFrameWithParameters(planeRight, 0.0, horizontalFOV / 2.0, -1.0, 0.0);
   }

   private void updatePlaneToFrameWithParameters(Plane3D plane, double pitch, double yaw, double yFace, double zFace)
   {
      tempFramePoint3D.setToZero(cameraFrame);
      tempFramePoint3D.changeFrame(ReferenceFrame.getWorldFrame());
      plane.getPoint().set(tempFramePoint3D);

      tempFramePose3D.setToZero();
      tempFramePose3D.setReferenceFrame(cameraFrame);
      tempFramePose3D.appendPitchRotation(Math.toRadians(pitch));
      tempFramePose3D.appendYawRotation(Math.toRadians(yaw)); // <-- this is wrong
      tempFramePose3D.changeFrame(ReferenceFrame.getWorldFrame());
      tempNormal.set(0.0, yFace, zFace);
      tempFramePose3D.getOrientation().transform(tempNormal);
      plane.getNormal().set(tempNormal);
   }

   public Plane3D getTopPlane()
   {
      return planeTop;
   }

   public Plane3D getBottomPlane()
   {
      return planeBottom;
   }

   public Plane3D getLeftPlane()
   {
      return planeLeft;
   }

   public Plane3D getRightPlane()
   {
      return planeRight;
   }
}
