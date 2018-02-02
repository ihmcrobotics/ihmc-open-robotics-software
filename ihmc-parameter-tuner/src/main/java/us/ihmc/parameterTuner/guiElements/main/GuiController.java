package us.ihmc.parameterTuner.guiElements.main;

import java.util.HashMap;
import java.util.List;

import javafx.collections.ObservableList;
import javafx.event.ActionEvent;
import javafx.event.EventHandler;
import javafx.fxml.FXML;
import javafx.scene.Node;
import javafx.scene.control.CheckBox;
import javafx.scene.control.ScrollPane;
import javafx.scene.control.SelectionMode;
import javafx.scene.control.TextField;
import javafx.scene.control.TreeItem;
import javafx.scene.input.ClipboardContent;
import javafx.scene.input.DragEvent;
import javafx.scene.input.Dragboard;
import javafx.scene.input.KeyCode;
import javafx.scene.input.KeyEvent;
import javafx.scene.input.MouseEvent;
import javafx.scene.input.TransferMode;
import javafx.scene.layout.StackPane;
import javafx.scene.layout.VBox;
import us.ihmc.commons.PrintTools;
import us.ihmc.parameterTuner.guiElements.GuiParameter;
import us.ihmc.parameterTuner.guiElements.GuiRegistry;
import us.ihmc.parameterTuner.guiElements.tree.ParameterTree;
import us.ihmc.parameterTuner.guiElements.tree.ParameterTreeParameter;
import us.ihmc.parameterTuner.guiElements.tree.ParameterTreeValue;
import us.ihmc.parameterTuner.guiElements.tuners.TuningBoxManager;

public class GuiController
{
   @FXML
   private TextField searchFieldParameters;
   @FXML
   private TextField searchFieldNamespaces;
   @FXML
   private CheckBox hideNamespaces;
   @FXML
   private ParameterTree tree;
   @FXML
   private ScrollPane scrollPane;
   @FXML
   private VBox tuningBox;
   @FXML
   private StackPane inputPane;

   private final HashMap<String, GuiParameter> parameterMap = new HashMap<>();
   private ChangeCollector changeCollector;
   private TuningBoxManager tuningBoxManager;

   public void initialize()
   {
      searchFieldParameters.textProperty().addListener(observable -> updateTree());
      searchFieldNamespaces.textProperty().addListener(observable -> updateTree());
      tuningBoxManager = new TuningBoxManager(tuningBox);

      tree.getSelectionModel().setSelectionMode(SelectionMode.MULTIPLE);
      tree.setOnMouseClicked(new EventHandler<MouseEvent>()
      {
         @Override
         public void handle(MouseEvent mouseEvent)
         {
            if (mouseEvent.getClickCount() >= 2)
            {
               addSelectedParametersToTuner();
            }
         }

      });
      tree.setOnKeyPressed(new EventHandler<KeyEvent>()
      {
         @Override
         public void handle(KeyEvent event)
         {
            if (event.getCode() == KeyCode.ENTER)
            {
               addSelectedParametersToTuner();
            }
         }
      });
      tree.setOnDragDetected(new EventHandler<MouseEvent>()
      {
         @Override
         public void handle(MouseEvent event)
         {
            Dragboard dragboard = tree.startDragAndDrop(TransferMode.MOVE);
            ClipboardContent clipboardContent = new ClipboardContent();
            clipboardContent.putString("");
            dragboard.setContent(clipboardContent);
         }
      });
      scrollPane.setOnDragOver(new EventHandler<DragEvent>()
      {
         @Override
         public void handle(DragEvent event)
         {
            event.acceptTransferModes(TransferMode.MOVE);
         }
      });
      scrollPane.setOnDragDropped(new EventHandler<DragEvent>()
      {
         @Override
         public void handle(DragEvent event)
         {
            addSelectedParametersToTuner();
         }
      });
   }

   private void addSelectedParametersToTuner()
   {
      ObservableList<TreeItem<ParameterTreeValue>> selectedItems = tree.getSelectionModel().getSelectedItems();
      for (TreeItem<ParameterTreeValue> selectedItem : selectedItems)
      {
         if (selectedItem != null && !selectedItem.getValue().isRegistry())
         {
            GuiParameter parameter = ((ParameterTreeParameter) selectedItem.getValue()).getParameter();
            tuningBoxManager.handleNewParameter(parameter);
         }
      }
   }

   @FXML
   protected void handleNamespaceButton(ActionEvent event)
   {
      updateTree();
      searchFieldNamespaces.setDisable(hideNamespaces.isSelected());
   }

   private void updateTree()
   {
      tree.filterRegistries(hideNamespaces.isSelected(), searchFieldParameters.getText(), searchFieldNamespaces.getText());
   }

   public void addInputNode(Node node)
   {
      if (node != null)
      {
         inputPane.getChildren().add(node);
      }
   }

   public void setRegistry(GuiRegistry fullRegistry)
   {
      tree.setRegistries(fullRegistry);
      updateTree();
      tuningBoxManager.clearAllParameters();

      changeCollector = new ChangeCollector();
      parameterMap.clear();
      List<GuiParameter> allParameters = fullRegistry.getAllParameters();
      allParameters.stream().forEach(parameter -> {
         parameter.addChangedListener(changeCollector);
         parameterMap.put(parameter.getUniqueName(), parameter);
      });
   }

   public List<GuiParameter> pollChangedParameters()
   {
      if (changeCollector == null)
      {
         return null;
      }

      return changeCollector.getChangedParametersAndClear();
   }

   public void updateParameters(List<GuiParameter> externallyChangesParameters)
   {
      if (changeCollector == null)
      {
         return;
      }

      changeCollector.stopRecording();
      externallyChangesParameters.stream().forEach(externalParameter -> {
         GuiParameter localParameter = parameterMap.get(externalParameter.getUniqueName());
         if (localParameter == null)
         {
            PrintTools.warn("Did not find " + externalParameter.getName() + " skipping...");
         }
         else
         {
            localParameter.setValue(externalParameter.getCurrentValue());
         }
      });
      changeCollector.startRecording();
   }
}
