package us.ihmc.humanoidBehaviors.ui.graphics;

import javafx.scene.Group;
import javafx.scene.paint.Color;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.Mesh;
import javafx.scene.shape.MeshView;
import us.ihmc.euclid.geometry.Plane3D;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.javaFXToolkit.shapes.JavaFXMeshBuilder;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;

import java.util.ArrayList;
import java.util.List;

public class CameraViewportGraphic extends Group
{
   private static final double SIZE = 0.5;
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


      List<Point3D> points = new ArrayList<>();


      Point3D commonPoint = new Point3D();
      Point3D upperLeftPoint = new Point3D();
      Point3D upperRightPoint = new Point3D();
      Point3D lowerLeftPoint = new Point3D();
      Point3D lowerRightPoint = new Point3D();


      // draw wireframe version instead

      tempFramePose3D.setToZero();
      tempFramePose3D.setReferenceFrame(cameraFrame);
      tempFramePose3D.changeFrame(ReferenceFrame.getWorldFrame());
      commonPoint.set(tempFramePose3D.getPosition());

      PoseReferenceFrame tempFrame;
      PoseReferenceFrame tempFrame2;
      FramePoint3D tempFramePoint;
      Pose3D pose3D;
      Pose3D pose3D2;

//      tempFrame = new PoseReferenceFrame("tempFrame", cameraFrame);
//      tempFrame2 = new PoseReferenceFrame("tempFrame2", tempFrame);
//      pose3D = new Pose3D();
//      pose3D.setOrientationYawPitchRoll(horizontalFOV / 2.0, verticalFOV / 2.0, 0.0);
//      pose3D.appendPitchRotation(verticalFOV / 2.0);
//      pose3D2 = new Pose3D();
//      pose3D2.appendYawRotation(horizontalFOV / 2.0);
//      tempFrame.setPoseAndUpdate(pose3D);
      tempFramePoint = new FramePoint3D(cameraFrame);
      tempFramePoint.setX(SIZE * Math.cos(verticalFOV / 2.0));
      tempFramePoint.setY(SIZE * Math.sin(horizontalFOV / 2.0));
      tempFramePoint.setZ(SIZE * Math.sin(verticalFOV / 2.0));
      tempFramePoint.changeFrame(ReferenceFrame.getWorldFrame());
      upperLeftPoint.set(tempFramePoint);

//      tempFrame = new PoseReferenceFrame("tempFrame", cameraFrame);
//      pose3D = new Pose3D();
//      pose3D.setOrientationYawPitchRoll(-horizontalFOV / 2.0, verticalFOV / 2.0, 0.0);
////      pose3D.appendPitchRotation(verticalFOV / 2.0);
////      pose3D.appendYawRotation(-horizontalFOV / 2.0);
//      tempFrame.setPoseAndUpdate(pose3D);
//      tempFramePoint = new FramePoint3D(tempFrame);
//      tempFramePoint.setX(SIZE);
      tempFramePoint = new FramePoint3D(cameraFrame);
      tempFramePoint.setX(SIZE * Math.cos(verticalFOV / 2.0));
      tempFramePoint.setY(SIZE * Math.sin(-horizontalFOV / 2.0));
      tempFramePoint.setZ(SIZE * Math.sin(verticalFOV / 2.0));
      tempFramePoint.changeFrame(ReferenceFrame.getWorldFrame());
      upperRightPoint.set(tempFramePoint);

//      tempFrame = new PoseReferenceFrame("tempFrame", cameraFrame);
//      pose3D = new Pose3D();
//      pose3D.setOrientationYawPitchRoll(horizontalFOV / 2.0, -verticalFOV / 2.0, 0.0);
////      pose3D.appendPitchRotation(-verticalFOV / 2.0);
////      pose3D.appendYawRotation(horizontalFOV / 2.0);
//      tempFrame.setPoseAndUpdate(pose3D);
//      tempFramePoint = new FramePoint3D(tempFrame);
//      tempFramePoint.setX(SIZE);
      tempFramePoint = new FramePoint3D(cameraFrame);
      tempFramePoint.setX(SIZE * Math.cos(-verticalFOV / 2.0));
      tempFramePoint.setY(SIZE * Math.sin(horizontalFOV / 2.0));
      tempFramePoint.setZ(SIZE * Math.sin(-verticalFOV / 2.0));
      tempFramePoint.changeFrame(ReferenceFrame.getWorldFrame());
      lowerLeftPoint.set(tempFramePoint);

//      tempFrame = new PoseReferenceFrame("tempFrame", cameraFrame);
//      pose3D = new Pose3D();
//      pose3D.setOrientationYawPitchRoll(-horizontalFOV / 2.0, -verticalFOV / 2.0, 0.0);
////      pose3D.appendPitchRotation(-verticalFOV / 2.0);
////      pose3D.appendYawRotation(-horizontalFOV / 2.0);
//      tempFrame.setPoseAndUpdate(pose3D);
//      tempFramePoint = new FramePoint3D(tempFrame);
//      tempFramePoint.setX(SIZE);
      tempFramePoint = new FramePoint3D(cameraFrame);
      tempFramePoint.setX(SIZE * Math.cos(-verticalFOV / 2.0));
      tempFramePoint.setY(SIZE * Math.sin(-horizontalFOV / 2.0));
      tempFramePoint.setZ(SIZE * Math.sin(-verticalFOV / 2.0));
      tempFramePoint.changeFrame(ReferenceFrame.getWorldFrame());
      lowerRightPoint.set(tempFramePoint);


      points.add(commonPoint);
      points.add(upperLeftPoint);
      points.add(upperRightPoint);
      points.add(commonPoint);
      points.add(upperRightPoint);
      points.add(lowerRightPoint);
      points.add(commonPoint);
      points.add(lowerRightPoint);
      points.add(lowerLeftPoint);
      points.add(commonPoint);
      points.add(lowerLeftPoint);
      points.add(upperLeftPoint);


//      // create 4 triangles
//      // add origin point to each
//      // add two more points to each
//
//      // create vector in x forward
//      Vector3D straightForward = new Vector3D();
//
//      updatePlaneToFrameWithParameters(points, -verticalFOV / 2.0, 0.0, 0.0, -1.0);
//      updatePlaneToFrameWithParameters(points, verticalFOV / 2.0, 0.0, 0.0, 1.0);
//      updatePlaneToFrameWithParameters(points, 0.0, -horizontalFOV / 2.0, 1.0, 0.0);
//      updatePlaneToFrameWithParameters(points, 0.0, horizontalFOV / 2.0, -1.0, 0.0);

      meshBuilder.addMultiLine(points, 0.01, false);

      Mesh mesh = meshBuilder.generateMesh();

      MeshView meshView = new MeshView(mesh);
      meshView.setMaterial(new PhongMaterial(Color.BLUEVIOLET));

      getChildren().add(meshView);
   }

   private void updatePlaneToFrameWithParameters(List<Point3D> points, double pitch, double yaw, double yFace, double zFace)
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
