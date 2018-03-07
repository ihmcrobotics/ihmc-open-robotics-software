package us.ihmc.parameterTuner.guiElements.main;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Optional;

import javafx.collections.ObservableList;
import javafx.event.EventHandler;
import javafx.fxml.FXML;
import javafx.scene.Node;
import javafx.scene.control.CheckBox;
import javafx.scene.control.ChoiceBox;
import javafx.scene.control.ContextMenu;
import javafx.scene.control.MenuItem;
import javafx.scene.control.SelectionMode;
import javafx.scene.control.SeparatorMenuItem;
import javafx.scene.control.Tab;
import javafx.scene.control.TabPane;
import javafx.scene.control.TextField;
import javafx.scene.control.TextInputDialog;
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
import us.ihmc.parameterTuner.guiElements.GuiParameterStatus;
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
   private StackPane treePane;
   @FXML
   private TabPane tabPane;
   @FXML
   private VBox tuningBox;
   @FXML
   private StackPane inputPane;
   @FXML
   private ChoiceBox<GuiParameterStatus> statusFilter;

   private final HashMap<String, GuiParameter> parameterMap = new HashMap<>();
   private ChangeCollector changeCollector;
   private TuningBoxManager tuningBoxManager;

   private final ParameterTree tree = new ParameterTree();

   public void initialize()
   {
      searchFieldParameters.textProperty().addListener(observable -> updateTree());
      searchFieldNamespaces.textProperty().addListener(observable -> updateTree());
      tuningBoxManager = new TuningBoxManager(tuningBox);

      statusFilter.getItems().addAll(GuiParameterStatus.values());
      statusFilter.getSelectionModel().select(GuiParameterStatus.ANY);
      statusFilter.getSelectionModel().selectedItemProperty().addListener((observable, oldValue, newValue) -> updateTree());

      treePane.getChildren().add(tree);
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
      tabPane.setOnDragOver(new EventHandler<DragEvent>()
      {
         @Override
         public void handle(DragEvent event)
         {
            event.acceptTransferModes(TransferMode.MOVE);
         }
      });
      tabPane.setOnDragDropped(new EventHandler<DragEvent>()
      {
         @Override
         public void handle(DragEvent event)
         {
            addSelectedParametersToTuner();
         }
      });

      MenuItem closeTab = new MenuItem("Close Open Tab");
      closeTab.setDisable(tabPane.getTabs().size() == 0);
      closeTab.setOnAction(event -> {
         Tab tab = tabPane.getSelectionModel().getSelectedItem();
         tabPane.getTabs().remove(tab);
         closeTab.setDisable(tabPane.getTabs().size() == 0);
      });
      MenuItem newTab = new MenuItem("New Tab");
      newTab.setOnAction(event -> {
         TextInputDialog dialog = new TextInputDialog();
         dialog.setTitle("Create Tab");
         dialog.setHeaderText("Enter a name for the new tab.");
         dialog.setContentText("Tab Name:");
         Optional<String> result = dialog.showAndWait();
         if (result.isPresent())
         {
            Tab tab = new Tab(result.get());
            tabPane.getTabs().add(tab);
            tabPane.getSelectionModel().select(tab);
            closeTab.setDisable(tabPane.getTabs().size() == 0);
         }
      });
      MenuItem renameTab = new MenuItem("Rename Open Tab");
      renameTab.setOnAction(event -> {
         TextInputDialog dialog = new TextInputDialog();
         dialog.setTitle("Rename Tab");
         dialog.setHeaderText("Enter a new name for the current tab.");
         dialog.setContentText("New Tab Name:");
         Optional<String> result = dialog.showAndWait();
         if (result.isPresent())
         {
            tabPane.getSelectionModel().getSelectedItem().setText(result.get());
         }
      });
      MenuItem saveTab = new MenuItem("Save Open Tab");
      saveTab.setOnAction(event -> {
         // TODO
      });
      ContextMenu tabContextMenu = new ContextMenu();
      tabContextMenu.getItems().add(saveTab);
      tabContextMenu.getItems().add(new SeparatorMenuItem());
      tabContextMenu.getItems().add(newTab);
      tabContextMenu.getItems().add(renameTab);
      tabContextMenu.getItems().add(closeTab);
      tabPane.setContextMenu(tabContextMenu);
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
   private void updateTree()
   {
      tree.filterRegistries(hideNamespaces.isSelected(), statusFilter.getValue(), searchFieldParameters.getText(), searchFieldNamespaces.getText());
      searchFieldNamespaces.setDisable(hideNamespaces.isSelected());
   }

   public void addInputNode(Node node)
   {
      if (node != null)
      {
         inputPane.getChildren().add(node);
      }
   }

   public void setRegistries(List<GuiRegistry> registries)
   {
      tuningBoxManager.setRegistries(registries);
      tree.setRegistries(registries, tuningBoxManager.getTunerMap());
      updateTree();

      changeCollector = new ChangeCollector();
      parameterMap.clear();
      List<GuiParameter> allParameters = new ArrayList<>();
      registries.stream().forEach(registry -> allParameters.addAll(registry.getAllParameters()));
      allParameters.stream().forEach(parameter -> {
         parameter.addChangedListener(changeCollector);
         parameter.addStatusUpdater();
         parameter.saveStateForReset();
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
            if (!localParameter.getCurrentValue().equals(externalParameter.getCurrentValue()))
            {
               localParameter.setValueAndStatus(externalParameter);
               localParameter.saveStateForReset();
            }
         }
      });
      changeCollector.startRecording();
   }
}
