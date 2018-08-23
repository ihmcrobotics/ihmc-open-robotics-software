package us.ihmc.avatar.joystickBasedJavaFXController;

import javafx.fxml.FXML;
import javafx.scene.Parent;
import us.ihmc.javaFXToolkit.messager.JavaFXMessager;

public class GraspingPaneController
{
   @FXML
   private Parent graspingObjectPane;

   @FXML
   private GraspingObjectPaneController graspingObjectPaneController;

   @FXML
   private GraspingFingerPaneController graspingFingerPaneController;

   public GraspingPaneController()
   {

   }

   public void initialize(JavaFXMessager messager)
   {
      graspingObjectPaneController.initialize(messager);
      graspingFingerPaneController.initialize(messager);
   }
}