package us.ihmc.humanoidBehaviors.ui.graphics;

import javafx.application.Platform;
import javafx.scene.paint.Color;
import javafx.scene.shape.DrawMode;
import javafx.scene.transform.Rotate;
import javafx.scene.transform.Scale;
import javafx.scene.transform.Translate;
import org.fxyz3d.shapes.primitives.Text3DMesh;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.axisAngle.interfaces.AxisAngleReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.yoVariables.parameters.DoubleParameter;

import java.util.function.DoubleConsumer;

public class LabelGraphic
{
   public static DoubleParameter textMeshScale;
   public static DoubleParameter textOffsetX;
   public static DoubleParameter textOffsetY;
   public static DoubleParameter textOffsetZ;
   public static DoubleParameter textRotationAxisX;
   public static DoubleParameter textRotationAxisY;
   public static DoubleParameter textRotationAxisZ;
   public static DoubleParameter textRotationAngle;

   private final Text3DMesh textMesh;
   private double currentScale;
   private Point3D currentPositionOffset;
   private AxisAngle currentOrientationOffset;

   public LabelGraphic(String text, double height, Color color)
   {
      textMesh = new Text3DMesh(text, height);
      textMesh.setTextureModeNone(color);
      textMesh.setDrawMode(DrawMode.FILL);

      currentScale = textMeshScale.getValue();
      currentPositionOffset = new Point3D(textOffsetX.getValue(), textOffsetY.getValue(), textOffsetZ.getValue());
      currentOrientationOffset = new AxisAngle(textRotationAxisX.getValue(), textRotationAxisY.getValue(), textRotationAxisZ.getValue(), textRotationAngle.getValue());
      addParameterListener(textMeshScale, value -> currentScale = value);
      addParameterListener(textOffsetX, value -> currentPositionOffset.setX(value));
      addParameterListener(textOffsetY, value -> currentPositionOffset.setX(value));
      addParameterListener(textOffsetZ, value -> currentPositionOffset.setZ(value));
      addParameterListener(textRotationAxisX, value -> currentOrientationOffset.setX(value));
      addParameterListener(textRotationAxisY, value -> currentOrientationOffset.setY(value));
      addParameterListener(textRotationAxisZ, value -> currentOrientationOffset.setZ(value));
      addParameterListener(textRotationAngle, value -> currentOrientationOffset.setAngle(value));

      setPoseInternal();
   }

   public void setPosition(Point3D position)
   {
//      currentPosition.set(position);
      setPoseInternal();
//      textMesh.setTranslateX(position.getX());
//      textMesh.setTranslateY(position.getY());
//      textMesh.setTranslateZ(position.getZ());
   }

   public void setOrientation(AxisAngleReadOnly axisAngle)
   {
//      currentOrientation.set(axisAngle);
      setPoseInternal();

      //      textMesh.rotateProperty().set(axisAngle.getAngle());
//      textMesh.getMeshes().forEach(mesh ->
//                                   {
//                                      mesh.setRotationAxis(new javafx.geometry.Point3D(axisAngle.getX(), axisAngle.getY(), axisAngle.getZ()));
//                                      mesh.setRotate(axisAngle.getAngle());
//                                   });
//      textMesh.setRotationAxis(new javafx.geometry.Point3D(axisAngle.getX(), axisAngle.getY(), axisAngle.getZ()));
//      textMesh.setRotate(axisAngle.getAngle());
//      textMesh.setNodeOrientation(NodeOrientation.RIGHT_TO_LEFT);
   }

   private void setPoseInternal()
   {
      textMesh.getTransforms().clear();
      textMesh.getTransforms().add(new Scale(textMeshScale.getValue(), currentScale));
      textMesh.getTransforms().add(new Rotate(currentOrientationOffset.getAngle(),
                                              currentOrientationOffset.getX(),
                                              currentOrientationOffset.getY(),
                                              currentOrientationOffset.getZ()));
      textMesh.getTransforms().add(new Translate(currentPositionOffset.getX(), currentPositionOffset.getY(), currentPositionOffset.getZ()));
   }

   public Text3DMesh getMesh()
   {
      return textMesh;
   }

   private void addParameterListener(DoubleParameter parameter, DoubleConsumer runnable)
   {
      parameter.addParameterChangedListener(changedParameter ->
      {
         double value = ((DoubleParameter) changedParameter).getValue();
         Platform.runLater(() ->
         {
            runnable.accept(value);
            setPoseInternal();
         });
      });
   }
}
