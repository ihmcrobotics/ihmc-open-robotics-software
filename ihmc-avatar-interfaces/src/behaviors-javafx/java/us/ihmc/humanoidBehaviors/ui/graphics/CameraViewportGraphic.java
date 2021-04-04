package us.ihmc.humanoidBehaviors.ui.graphics;

import javafx.scene.Group;
import javafx.scene.paint.Color;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.Mesh;
import javafx.scene.shape.MeshView;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
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
   private FramePoint3D tempFramePoint = new FramePoint3D();
   private final List<Point3D> points = new ArrayList<>();
   private final Point3D commonPoint = new Point3D();
   private final Point3D upperLeftPoint = new Point3D();
   private final Point3D upperRightPoint = new Point3D();
   private final Point3D lowerLeftPoint = new Point3D();
   private final Point3D lowerRightPoint = new Point3D();

   public CameraViewportGraphic(PoseReferenceFrame cameraFrame, double verticalFOV, double horizontalFOV)
   {
      this.cameraFrame = cameraFrame;
      this.verticalFOV = verticalFOV;
      this.horizontalFOV = horizontalFOV;
   }

   public void update()
   {
      JavaFXMeshBuilder meshBuilder = new JavaFXMeshBuilder();

      tempFramePose3D.setToZero();
      tempFramePose3D.setReferenceFrame(cameraFrame);
      tempFramePose3D.changeFrame(ReferenceFrame.getWorldFrame());
      commonPoint.set(tempFramePose3D.getPosition());

      updatePoint(upperLeftPoint, horizontalFOV, verticalFOV);
      updatePoint(upperRightPoint, -horizontalFOV, verticalFOV);
      updatePoint(lowerLeftPoint, horizontalFOV, -verticalFOV);
      updatePoint(lowerRightPoint, -horizontalFOV, -verticalFOV);

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

      meshBuilder.addMultiLine(points, 0.01, false);

      Mesh mesh = meshBuilder.generateMesh();

      MeshView meshView = new MeshView(mesh);
      meshView.setMaterial(new PhongMaterial(Color.BLUEVIOLET));

      getChildren().add(meshView);
   }

   private void updatePoint(Point3D upperLeftPoint, double horizontal, double vertical)
   {
      tempFramePoint.setToZero(cameraFrame);
      tempFramePoint.setX(SIZE);
      tempFramePoint.setY(SIZE * Math.asin(Math.toRadians(horizontal / 2.0)));
      tempFramePoint.setZ(SIZE * Math.asin(Math.toRadians(vertical / 2.0)));
      tempFramePoint.changeFrame(ReferenceFrame.getWorldFrame());
      upperLeftPoint.set(tempFramePoint);
   }
}
