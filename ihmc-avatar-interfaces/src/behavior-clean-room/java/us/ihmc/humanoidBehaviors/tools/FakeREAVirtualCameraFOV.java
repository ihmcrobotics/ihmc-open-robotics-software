package us.ihmc.humanoidBehaviors.tools;

import us.ihmc.euclid.geometry.Plane3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.pathPlanning.visibilityGraphs.tools.PlanarRegionsListCutTool;
import us.ihmc.robotics.geometry.PlanarRegionsList;

public class FakeREAVirtualCameraFOV
{
   private final ReferenceFrame cameraFrame;

   private final double verticalFOV;
   private final double horizontalFOV;
   private final FramePose3D cameraPose3D;

   private final FramePose3D tempFramePose3D = new FramePose3D();
   private final Vector3D tempNormal = new Vector3D();

   private final Plane3D planeTop;
   private final Plane3D planeBottom;
   private final Plane3D planeLeft;
   private final Plane3D planeRight;


   public FakeREAVirtualCameraFOV(double verticalFOV, double horizontalFOV, ReferenceFrame cameraFrame)
   {
      this.verticalFOV = verticalFOV;
      this.horizontalFOV = horizontalFOV;
      cameraPose3D = new FramePose3D(cameraFrame);
      this.cameraFrame = cameraFrame;

      planeTop = new Plane3D();
      planeBottom = new Plane3D();
      planeLeft = new Plane3D();
      planeRight = new Plane3D();
   }

   public PlanarRegionsList filterMapToVisible(PlanarRegionsList map)
   {
      cameraPose3D.changeFrame(ReferenceFrame.getWorldFrame());

      calculatePlane(planeTop, -verticalFOV / 2.0, 0.0, 0.0, -1.0);
      calculatePlane(planeBottom, verticalFOV / 2.0, 0.0, 0.0, 1.0);
      calculatePlane(planeLeft, 0.0, -horizontalFOV / 2.0, -1.0, 0.0);
      calculatePlane(planeRight, 0.0, horizontalFOV / 2.0, 1.0, 0.0);

      PlanarRegionsList visibleMap = PlanarRegionsListCutTool.cutByPlane(planeTop, map);
      visibleMap = PlanarRegionsListCutTool.cutByPlane(planeBottom, visibleMap);
      visibleMap = PlanarRegionsListCutTool.cutByPlane(planeLeft, visibleMap);
      visibleMap = PlanarRegionsListCutTool.cutByPlane(planeRight, visibleMap);

      return visibleMap;
   }

   private void calculatePlane(Plane3D plane, double pitch, double yaw, double yFace, double zFace)
   {
      tempFramePose3D.setToZero();
      tempFramePose3D.setReferenceFrame(cameraFrame);
      tempFramePose3D.appendPitchRotation(pitch);
      tempFramePose3D.appendYawRotation(yaw);
      tempFramePose3D.changeFrame(ReferenceFrame.getWorldFrame());
      plane.setPoint(tempFramePose3D.getPosition());
      tempNormal.set(0.0, yFace, zFace);
      tempFramePose3D.getOrientation().transform(tempNormal);
      this.planeTop.setNormal(tempNormal);
   }
}
