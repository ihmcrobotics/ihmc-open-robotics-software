package us.ihmc.javafx.graphics;

import javafx.application.Platform;
import javafx.scene.Node;
import javafx.scene.paint.Color;
import javafx.scene.shape.DrawMode;
import javafx.scene.transform.Scale;
import org.fxyz3d.shapes.primitives.Text3DMesh;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DBasics;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;
import us.ihmc.javaFXVisualizers.JavaFXGraphicTools;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

import java.util.function.DoubleConsumer;

public class LabelGraphic
{
   public static boolean TUNING_MODE = false;

   public static TunedDouble textMeshHeight = new TunedDouble(0.0005);
   public static TunedDouble textMeshScale = new TunedDouble(0.03);
   public static TunedDouble textOffsetX = new TunedDouble(0.06);
   public static TunedDouble textOffsetY = new TunedDouble(0.0);
   public static TunedDouble textOffsetYaw = new TunedDouble(3 * Math.PI / 2);
   public static TunedDouble textOffsetPitch = new TunedDouble(0.0);
   public static TunedDouble textOffsetRoll = new TunedDouble(Math.PI);

   private static class TunedDouble
   {
      final double primitive;
      DoubleParameter parameter;

      public TunedDouble(double initialValue)
      {
         primitive = initialValue;
      }

      double getValue()
      {
         return TUNING_MODE ? parameter.getValue() : primitive;
      }
   }

   public static void initializeYoVariables(YoVariableRegistry registry)
   {
      if (TUNING_MODE)
      {
         textMeshHeight  .parameter = new DoubleParameter("textMeshHeight",  registry, textMeshHeight  .primitive);
         textMeshScale   .parameter = new DoubleParameter("textMeshScale",   registry, textMeshScale   .primitive);
         textOffsetX     .parameter = new DoubleParameter("textOffsetX",     registry, textOffsetX     .primitive);
         textOffsetY     .parameter = new DoubleParameter("textOffsetY",     registry, textOffsetY     .primitive);
         textOffsetYaw   .parameter = new DoubleParameter("textOffsetYaw",   registry, textOffsetYaw   .primitive);
         textOffsetPitch .parameter = new DoubleParameter("textOffsetPitch", registry, textOffsetPitch .primitive);
         textOffsetRoll  .parameter = new DoubleParameter("textOffsetRoll",  registry, textOffsetRoll  .primitive);
      }
   }
   
   private final Text3DMesh textMesh;
   private double currentHeight;
   private double currentScale;
   private Point3D currentPositionOffset;
   private YawPitchRoll currentOrientationOffset;
   private final FramePose3D pose = new FramePose3D();
   private final FramePose3D offestPose = new FramePose3D();

   /**
    * height - specified in meters, the "thickness" of the text in 3D; the distance from the front face to the back face
    * offsets x, y, and z - specified in meters
    *
    * @param text
    */
   public LabelGraphic(String text)
   {
      textMesh = new Text3DMesh(text, textMeshHeight.getValue());
      textMesh.setTextureModeNone(Color.BLACK);
      textMesh.setDrawMode(DrawMode.FILL);

      currentScale = textMeshScale.getValue();
      currentPositionOffset = new Point3D(textOffsetX.getValue(), textOffsetY.getValue(), -textMeshHeight.getValue());
      currentOrientationOffset = new YawPitchRoll(textOffsetYaw.getValue(), textOffsetPitch.getValue(), textOffsetRoll.getValue());
      if (TUNING_MODE)
      {
         addParameterListener(textMeshHeight.parameter, value ->
         {
            currentHeight = value;
            currentPositionOffset.setZ(-value);
         });
         addParameterListener(textMeshScale.parameter, value -> currentScale = value);
         addParameterListener(textOffsetX.parameter, value -> currentPositionOffset.setX(value));
         addParameterListener(textOffsetY.parameter, value -> currentPositionOffset.setY(value));
         addParameterListener(textOffsetYaw.parameter, value -> currentOrientationOffset.setYaw(value));
         addParameterListener(textOffsetPitch.parameter, value -> currentOrientationOffset.setPitch(value));
         addParameterListener(textOffsetRoll.parameter, value -> currentOrientationOffset.setRoll(value));
      }

      update();
   }

   public FramePose3DBasics getPose()
   {
      return pose;
   }

   public void update()
   {
      offestPose.set(pose);
      offestPose.appendRotation(currentOrientationOffset);
      offestPose.appendTranslation(currentPositionOffset);

      JavaFXGraphicTools.setNodeTransformFromPose(textMesh, offestPose, currentScale);

      textMesh.getTransforms().add(new Scale(textMeshScale.getValue(), currentScale));

      textMesh.setHeight(currentHeight);
      textMesh.setTextureModeNone(Color.BLACK);
   }

   public Node getNode()
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
            update();
         });
      });
   }
}
