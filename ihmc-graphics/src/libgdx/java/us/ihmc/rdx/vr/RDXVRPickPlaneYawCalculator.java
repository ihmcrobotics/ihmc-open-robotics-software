package us.ihmc.rdx.vr;

import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.geometry.Plane3D;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.referenceFrame.FrameLine3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.referenceFrames.MutableReferenceFrame;

/**
 * This class is used for pointing the VR controllers at the ground
 * and dragging stuff around, controlling the XY translation and yaw.
 */
public class RDXVRPickPlaneYawCalculator
{
   private final FrameVector3D controllerZAxisVector = new FrameVector3D();
   private final FrameLine3D pickRay = new FrameLine3D();
   private final FramePose3D xyPlanePose = new FramePose3D();
   private final Plane3D plane = new Plane3D();
   private final Point3D planeRayIntesection = new Point3D();
   private final Point3D controllerZAxisProjectedToPlanePoint = new Point3D();
   private final Vector3D orientationDeterminationVector = new Vector3D();
   private final FramePose3D yawPose = new FramePose3D();
   private final MutableReferenceFrame yawReferenceFrame = new MutableReferenceFrame();

   public FramePose3DReadOnly calculate(ReferenceFrame controllerPickFrame, ReferenceFrame xyPlaneFrame)
   {
      controllerZAxisVector.setIncludingFrame(controllerPickFrame, Axis3D.Z);
      controllerZAxisVector.changeFrame(ReferenceFrame.getWorldFrame());

      pickRay.setToZero(controllerPickFrame);
      pickRay.getDirection().set(Axis3D.X);
      pickRay.changeFrame(ReferenceFrame.getWorldFrame());

      xyPlanePose.setToZero(xyPlaneFrame);
      xyPlanePose.changeFrame(ReferenceFrame.getWorldFrame());

      plane.getNormal().set(Axis3D.Z);
      plane.getPoint().set(xyPlanePose.getPosition());

      plane.intersectionWith(pickRay, planeRayIntesection);

      controllerZAxisProjectedToPlanePoint.set(planeRayIntesection);
      controllerZAxisProjectedToPlanePoint.add(controllerZAxisVector);
      plane.orthogonalProjection(controllerZAxisProjectedToPlanePoint);

      orientationDeterminationVector.sub(controllerZAxisProjectedToPlanePoint, planeRayIntesection);

      yawPose.getPosition().set(planeRayIntesection);
      EuclidGeometryTools.orientation3DFromFirstToSecondVector3D(Axis3D.X, orientationDeterminationVector, yawPose.getOrientation());

      yawPose.get(yawReferenceFrame.getTransformToParent());
      yawReferenceFrame.getReferenceFrame().update();

      return yawPose;
   }

   public ReferenceFrame getYawReferenceFrame()
   {
      return yawReferenceFrame.getReferenceFrame();
   }
}
