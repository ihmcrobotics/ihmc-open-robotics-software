package us.ihmc.humanoidBehaviors.ui.graphics;

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
import us.ihmc.humanoidBehaviors.ui.tools.JavaFXGraphicTools;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

import java.util.function.DoubleConsumer;

public class LabelGraphic
{
   public static DoubleParameter textMeshHeight;
   public static DoubleParameter textMeshScale;
   public static DoubleParameter textOffsetX;
   public static DoubleParameter textOffsetY;
   public static DoubleParameter textOffsetYaw;
   public static DoubleParameter textOffsetPitch;
   public static DoubleParameter textOffsetRoll;

   public static void initializeYoVariables(YoVariableRegistry registry)
   {
      textMeshHeight = new DoubleParameter("textMeshHeight", registry, 0.01);
      textMeshScale = new DoubleParameter("textMeshScale", registry, 0.03);
      textOffsetX = new DoubleParameter("textOffsetX", registry, 0.06);
      textOffsetY = new DoubleParameter("textOffsetY", registry, 0.0);
      textOffsetYaw = new DoubleParameter("textOffsetYaw", registry, 3 * Math.PI / 2);
      textOffsetPitch = new DoubleParameter("textOffsetPitch", registry, 0.0);
      textOffsetRoll = new DoubleParameter("textOffsetRoll", registry, Math.PI);
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
      addParameterListener(textMeshHeight, value ->
      {
         currentHeight = value;
         currentPositionOffset.setZ(-value);
      });
      addParameterListener(textMeshScale, value -> currentScale = value);
      addParameterListener(textOffsetX, value -> currentPositionOffset.setX(value));
      addParameterListener(textOffsetY, value -> currentPositionOffset.setY(value));
      addParameterListener(textOffsetYaw, value -> currentOrientationOffset.setYaw(value));
      addParameterListener(textOffsetPitch, value -> currentOrientationOffset.setPitch(value));
      addParameterListener(textOffsetRoll, value -> currentOrientationOffset.setRoll(value));

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
