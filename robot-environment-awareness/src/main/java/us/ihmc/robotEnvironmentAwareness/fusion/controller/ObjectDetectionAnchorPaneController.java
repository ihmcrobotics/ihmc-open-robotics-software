package us.ihmc.robotEnvironmentAwareness.fusion.controller;

import java.util.ArrayList;
import java.util.List;

import javafx.fxml.FXML;
import javafx.scene.control.Button;
import javafx.scene.control.CheckBox;
import javafx.scene.control.TextField;
import us.ihmc.javaFXToolkit.messager.JavaFXMessager;
import us.ihmc.javaFXToolkit.messager.SharedMemoryJavaFXMessager;
import us.ihmc.robotEnvironmentAwareness.communication.LidarImageFusionAPI;
import us.ihmc.robotEnvironmentAwareness.fusion.objectDetection.ObjectType;

public class ObjectDetectionAnchorPaneController
{
   private JavaFXMessager messager;

   private static final String defaultHostAddress = "10.100.3.157";

   @FXML
   private TextField serverADD;

   @FXML
   private Button btnConnect;

   @FXML
   private Button btnObjectDetection;

   @FXML
   private Button btnClear;

   @FXML
   private CheckBox cboxDoor;

   @FXML
   private CheckBox cboxDoorHandle;

   @FXML
   private CheckBox cboxCup;

   @FXML
   private CheckBox cboxhuman;

   public void initialize(SharedMemoryJavaFXMessager messager)
   {
      this.messager = messager;
      serverADD.setText(defaultHostAddress);
      cboxDoor.setSelected(true);
      cboxDoorHandle.setSelected(true);
   }

   @FXML
   public void connect()
   {
      messager.submitMessage(LidarImageFusionAPI.ObjectDetectionModuleAddress, serverADD.getText());
      messager.submitMessage(LidarImageFusionAPI.RequestSocketConnection, true);
   }

   @FXML
   public void clear()
   {
      messager.submitMessage(LidarImageFusionAPI.ClearObjects, true);
   }

   @FXML
   public void requestObjectDetection()
   {
      List<ObjectType> selectedTypes = new ArrayList<ObjectType>();
      if (cboxDoor.isSelected())
         selectedTypes.add(ObjectType.Door);
      if (cboxDoorHandle.isSelected())
         selectedTypes.add(ObjectType.DoorHandle);
      if (cboxCup.isSelected())
         selectedTypes.add(ObjectType.Cup);
      if (cboxhuman.isSelected())
         selectedTypes.add(ObjectType.Human);

      messager.submitMessage(LidarImageFusionAPI.SelectedObjecTypes, selectedTypes);
      messager.submitMessage(LidarImageFusionAPI.RequestObjectDetection, true);
   }
}