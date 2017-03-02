package us.ihmc.javaFXToolkit.cameraControllers;

import javafx.event.EventHandler;
import javafx.scene.Node;
import javafx.scene.input.MouseEvent;
import javafx.scene.transform.Rotate;
import us.ihmc.robotics.MathTools;

/**
 * Use {@link FocusBasedCameraMouseEventHandler} instead.
 * @deprecated
 * @author Sylvain Bertrand
 */
public class SimpleCameraMouseEventHandler implements EventHandler<MouseEvent>
{
   private final Rotate xRotate = new Rotate(0.0, 0.0, 0.0, 0.0, Rotate.X_AXIS);
   private final Rotate yRotate = new Rotate(0.0, 0.0, 0.0, 0.0, Rotate.Y_AXIS);

   private double mouseOldX = 0.0;
   private double mouseOldY = 0.0;
   private double rotateModifier = 10.0;

   private double cameraLimitY = 90.0;

   public SimpleCameraMouseEventHandler(Node cameraNode)
   {
      cameraNode.getTransforms().addAll(xRotate, yRotate);
   }

   @Override
   public void handle(MouseEvent event)
   {
      if (event.getEventType() == MouseEvent.MOUSE_PRESSED || event.getEventType() == MouseEvent.MOUSE_DRAGGED)
      {
         // Acquire the new mouse coordinates from the recent event
         double mouseNewX = event.getSceneX();
         double mouseNewY = event.getSceneY();

         if (event.getEventType() == MouseEvent.MOUSE_DRAGGED)
         {
            // Calculate the rotation change of the camera pitch
            double pitchRotate = xRotate.getAngle() + (mouseNewY - mouseOldY) / rotateModifier;
            pitchRotate = MathTools.clamp(pitchRotate, cameraLimitY);

            xRotate.setAngle(pitchRotate);

            double yawRotate = yRotate.getAngle() - (mouseNewX - mouseOldX) / rotateModifier;
            yRotate.setAngle(yawRotate);
         }

         mouseOldX = mouseNewX;
         mouseOldY = mouseNewY;
      }
   }
}
