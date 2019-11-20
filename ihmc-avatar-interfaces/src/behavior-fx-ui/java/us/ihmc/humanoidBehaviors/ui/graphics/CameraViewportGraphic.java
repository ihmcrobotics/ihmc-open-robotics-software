package us.ihmc.humanoidBehaviors.ui.graphics;

import javafx.scene.Group;
import us.ihmc.euclid.geometry.Plane3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.javaFXToolkit.shapes.JavaFXMeshBuilder;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;

public class CameraViewportGraphic extends Group
{
   private static final double SIZE = 0.1;
   private final PoseReferenceFrame cameraFrame;
   private final double verticalFOV;
   private final double horizontalFOV;

   FramePose3D tempFramePose3D = new FramePose3D();

   public CameraViewportGraphic(PoseReferenceFrame cameraFrame, double verticalFOV, double horizontalFOV)
   {

      this.cameraFrame = cameraFrame;
      this.verticalFOV = verticalFOV;
      this.horizontalFOV = horizontalFOV;
   }
   // import a model of a camera
   // what model types can be loaded?

   // just make some triangles


   public void update()
   {
      JavaFXMeshBuilder meshBuilder = new JavaFXMeshBuilder();

      tempFramePose3D.setToZero();
      tempFramePose3D.setReferenceFrame(cameraFrame);

      Point3D commonPoint = new Point3D();

      // create 4 triangles
      // add origin point to each
      // add two more points to each

      // create vector in x forward
      Vector3D straightForward = new Vector3D();

      updatePlaneToFrameWithParameters(meshBuilder, -verticalFOV / 2.0, 0.0, 0.0, -1.0);
      updatePlaneToFrameWithParameters(meshBuilder, verticalFOV / 2.0, 0.0, 0.0, 1.0);
      updatePlaneToFrameWithParameters(meshBuilder, 0.0, -horizontalFOV / 2.0, 1.0, 0.0);
      updatePlaneToFrameWithParameters(meshBuilder, 0.0, horizontalFOV / 2.0, -1.0, 0.0);




   }

   private void updatePlaneToFrameWithParameters(JavaFXMeshBuilder meshBuilder, double pitch, double yaw, double yFace, double zFace)
   {
      tempFramePose3D.setToZero();
      tempFramePose3D.setReferenceFrame(cameraFrame);
      tempFramePose3D.appendPitchRotation(pitch);
      tempFramePose3D.appendYawRotation(yaw);
      tempFramePose3D.changeFrame(ReferenceFrame.getWorldFrame());

//      meshBuilder.addPolygon();

      // do it again

   }
}
