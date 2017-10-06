package us.ihmc.javaFXToolkit.cameraControllers;

import javafx.event.EventHandler;
import javafx.scene.PerspectiveCamera;
import javafx.scene.input.KeyCode;
import javafx.scene.input.KeyEvent;
import javafx.scene.transform.Translate;

/**
 * Use {@link FocusBasedCameraMouseEventHandler} instead.
 * @deprecated
 * @author Sylvain Bertrand
 */
public class SimpleCameraKeyboardEventHandler implements EventHandler<KeyEvent>
{
   private final Translate translate = new Translate(0.0, 0.0, 0.0);

   private double cameraQuantity = 0.1;
   private double cameraModifier = 2.0 * cameraQuantity;

   private final PerspectiveCamera cameraNode;

   public SimpleCameraKeyboardEventHandler(PerspectiveCamera cameraNode)
   {
      this.cameraNode = cameraNode;
      cameraNode.getTransforms().add(translate);
   }

   @Override
   public void handle(KeyEvent event)
   {
      double change = cameraQuantity;
      //Add shift modifier to simulate running speed
      if (event.isShiftDown())
         change = cameraModifier;

      KeyCode keyCode = event.getCode();

      if (keyCode == KeyCode.W)
         cameraNode.setTranslateX(cameraNode.getTranslateX() + change);
      if (keyCode == KeyCode.S)
         cameraNode.setTranslateX(cameraNode.getTranslateX() - change);

      if (keyCode == KeyCode.D)
         cameraNode.setTranslateY(cameraNode.getTranslateY() + change);
      if (keyCode == KeyCode.A)
         cameraNode.setTranslateY(cameraNode.getTranslateY() - change);
   }
}
