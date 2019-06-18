package us.ihmc.parameterTuner.guiElements.main;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import javafx.beans.property.BooleanProperty;
import javafx.beans.property.SimpleBooleanProperty;
import javafx.collections.ObservableList;
import javafx.event.EventHandler;
import javafx.fxml.FXML;
import javafx.scene.Node;
import javafx.scene.control.CheckBox;
import javafx.scene.control.ChoiceBox;
import javafx.scene.control.SelectionMode;
import javafx.scene.control.TabPane;
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
import us.ihmc.parameterTuner.ParameterTuningTools;
import us.ihmc.parameterTuner.guiElements.GuiParameter;
import us.ihmc.parameterTuner.guiElements.GuiParameterStatus;
import us.ihmc.parameterTuner.guiElements.GuiRegistry;
import us.ihmc.parameterTuner.guiElements.tabs.TuningTabManager;
import us.ihmc.parameterTuner.guiElements.tree.ParameterTree;
import us.ihmc.parameterTuner.guiElements.tree.ParameterTreeParameter;
import us.ihmc.parameterTuner.guiElements.tree.ParameterTreeValue;
import us.ihmc.parameterTuner.guiElements.tuners.Tuner;

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
   private StackPane inputPane;
   @FXML
   private ChoiceBox<GuiParameterStatus> statusFilter;

   private final HashMap<String, GuiParameter> parameterMap = new HashMap<>();
   private ChangeCollector changeCollector;
   private TuningTabManager tuningTabManager;

   private final ParameterTree tree = new ParameterTree();

   private final List<GuiRegistry> allRegistries = new ArrayList<>();
   private final BooleanProperty rootRegistriesChanged = new SimpleBooleanProperty();

   private final ParameterFileLoader loader = new ParameterFileLoader();

   public void initialize()
   {
      searchFieldParameters.textProperty().addListener(observable -> updateTree());
      searchFieldNamespaces.textProperty().addListener(observable -> updateTree());

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
            if (event.getGestureSource() == tree)
            {
               addSelectedParametersToTuner();
            }
         }
      });

      tuningTabManager = new TuningTabManager(tabPane);
   }

   private void addSelectedParametersToTuner()
   {
      ObservableList<TreeItem<ParameterTreeValue>> selectedItems = tree.getSelectionModel().getSelectedItems();
      for (TreeItem<ParameterTreeValue> selectedItem : selectedItems)
      {
         if (selectedItem != null && !selectedItem.getValue().isRegistry())
         {
            GuiParameter parameter = ((ParameterTreeParameter) selectedItem.getValue()).getParameter();
            tuningTabManager.handleNewParameter(parameter.getUniqueName());
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
      Map<String, Tuner> tunerMap = ParameterTuningTools.createTunerMap(registries);
      tuningTabManager.setTunerMap(tunerMap);
      tree.setRegistries(registries, tunerMap);
      loader.setTunerMap(tunerMap);
      updateTree();

      changeCollector = new ChangeCollector();
      parameterMap.clear();
      List<GuiParameter> allParameters = new ArrayList<>();
      registries.forEach(registry -> allParameters.addAll(registry.getAllParameters()));
      allParameters.forEach(parameter -> {
         parameter.addChangedListener(changeCollector);
         parameter.addStatusUpdater();
         parameter.saveStateForReset();
         parameterMap.put(parameter.getUniqueName(), parameter);
      });

      allRegistries.clear();
      registries.stream().forEach(registry -> {
         allRegistries.add(registry);
         allRegistries.addAll(registry.getAllRegistries());
      });
      allRegistries.forEach(registry -> registry.isRoot().addListener((observable, oldValue, newValue) -> rootRegistriesChanged.set(true)));
      rootRegistriesChanged.set(true);
   }

   public boolean areRootRegistriesChanged()
   {
      return rootRegistriesChanged.getValue();
   }

   public List<String> pollRootRegistryNames()
   {
      rootRegistriesChanged.set(false);
      List<String> rootRegistries = new ArrayList<>();
      allRegistries.stream().filter(registry -> registry.isRoot().get()).forEach(registry -> rootRegistries.add(registry.getUniqueName()));
      return rootRegistries;
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
         String uniqueName = externalParameter.getUniqueName();
         GuiParameter localParameter = parameterMap.get(uniqueName);
         if (changeCollector.isPending(uniqueName))
         {
            changeCollector.parameterWasUpdated(uniqueName, externalParameter.getCurrentValue());
         }
         else if (!localParameter.getCurrentValue().equals(externalParameter.getCurrentValue()))
         {
            localParameter.setValueAndStatus(externalParameter);
            localParameter.saveStateForReset();
         }
      });
      changeCollector.startRecording();
   }

   public void close()
   {
      tuningTabManager.close();
   }

   @FXML
   public void loadFromFile()
   {
      loader.load(tree.getScene());
   }
}
