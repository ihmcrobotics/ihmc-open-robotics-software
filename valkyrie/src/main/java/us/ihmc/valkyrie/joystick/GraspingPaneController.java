package us.ihmc.valkyrie.joystick;

import javafx.fxml.FXML;
import javafx.scene.Parent;
import us.ihmc.messager.javafx.JavaFXMessager;

public class GraspingPaneController
{
   @FXML
   private Parent graspingObjectPane;

   @FXML
   private GraspingFingerPaneController graspingFingerPaneController;

   public GraspingPaneController()
   {

   }

   public void initialize(JavaFXMessager messager)
   {
      graspingFingerPaneController.initialize(messager);
   }
}