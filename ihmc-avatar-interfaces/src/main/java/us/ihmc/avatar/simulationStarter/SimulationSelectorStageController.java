package us.ihmc.avatar.simulationStarter;

import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.util.ArrayList;
import java.util.EnumMap;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Properties;

import javafx.beans.value.ChangeListener;
import javafx.beans.value.ObservableValue;
import javafx.collections.FXCollections;
import javafx.collections.ObservableList;
import javafx.fxml.FXML;
import javafx.scene.control.Button;
import javafx.scene.control.CheckBox;
import javafx.scene.control.ComboBox;
import javafx.scene.image.Image;
import javafx.scene.input.KeyCode;
import javafx.scene.input.KeyEvent;
import javafx.scene.layout.GridPane;
import javafx.stage.Stage;
import us.ihmc.avatar.simulationStarter.DRCSimulationTools.Modules;

public class SimulationSelectorStageController
{
   private static final String STARTING_LOCATION_PROPERTY_NAME = "startingLocation";

   @FXML
   private Stage stage;
   @FXML
   private GridPane checkBoxesPane;
   @FXML
   private ComboBox<String> startingLocationComboBox;
   @FXML
   private Button okButton, cancelButton;

   private final List<Modules> modulesToStart = new ArrayList<>();

   public <T> T showAndWait(@SuppressWarnings("unchecked") T... possibleStartingLocations)
   {
      String configFile = System.getProperty("user.home") + "/.ihmc/drcSimulationDefaultOptions.config";
      Properties properties = new Properties();
      try
      {
         FileInputStream lastConfigInputStream = new FileInputStream(configFile);
         properties.load(lastConfigInputStream);
         lastConfigInputStream.close();
      }
      catch (IOException e)
      {
         // No config file, whatever.
      }

      int nCols = 3;

      int row = 0;
      int col = 0;

      EnumMap<Modules, CheckBox> moduleCheckBoxes = new EnumMap<>(Modules.class);

      for (Modules module : Modules.values())
      {
         boolean enabled;
         if (module.isAlwaysEnabled()) // So Simulation, operator and spectator interfaces, and behavior visualizer can never get disabled
            enabled = true;
         else
            enabled = Boolean.parseBoolean(properties.getProperty(module.getPropertyNameForEnable(), Boolean.toString(module.getDefaultValueForEnable())));
         boolean selected = Boolean.parseBoolean(properties.getProperty(module.getPropertyNameForSelected(),
                                                                        Boolean.toString(module.getDefaultValueForSelected())));
         CheckBox checkBox = new CheckBox(module.getName());
         checkBox.setSelected(selected);
         checkBox.setDisable(!enabled);
         checkBoxesPane.add(checkBox, col, row);
         moduleCheckBoxes.put(module, checkBox);

         col++;
         if (col >= nCols)
         {
            col = 0;
            row++;
         }
      }

      ChangeListener<Boolean> networkProcessorCheckBoxChangeListener = new ChangeListener<Boolean>()
      {
         @Override
         public void changed(ObservableValue<? extends Boolean> observable, Boolean oldValue, Boolean newValue)
         {
            boolean isNetworkProcessorSelected = moduleCheckBoxes.get(Modules.NETWORK_PROCESSOR).isSelected();
            boolean isNetworkProcessorDisabled = moduleCheckBoxes.get(Modules.NETWORK_PROCESSOR).isDisabled();
            moduleCheckBoxes.get(Modules.BEHAVIOR_MODULE).setDisable(!isNetworkProcessorSelected || isNetworkProcessorDisabled);
            moduleCheckBoxes.get(Modules.SENSOR_MODULE).setDisable(!isNetworkProcessorSelected || isNetworkProcessorDisabled);
            moduleCheckBoxes.get(Modules.ZERO_POSE_PRODUCER).setDisable(!isNetworkProcessorSelected || isNetworkProcessorDisabled);
            moduleCheckBoxes.get(Modules.ROS_MODULE).setDisable(!isNetworkProcessorSelected || isNetworkProcessorDisabled);
            moduleCheckBoxes.get(Modules.REA_MODULE).setDisable(!isNetworkProcessorSelected || isNetworkProcessorDisabled);
            moduleCheckBoxes.get(Modules.REA_UI).setDisable(!isNetworkProcessorSelected || isNetworkProcessorDisabled);
            moduleCheckBoxes.get(Modules.DIRECTIONAL_CONTROL_TOOLBOX).setDisable(!isNetworkProcessorSelected || isNetworkProcessorDisabled);
         }
      };

      ChangeListener<Boolean> simulationCheckBoxChangeListener = new ChangeListener<Boolean>()
      {
         @Override
         public void changed(ObservableValue<? extends Boolean> observable, Boolean oldValue, Boolean newValue)
         {
            boolean isSimulationSelected = moduleCheckBoxes.get(Modules.SIMULATION).isSelected();
            moduleCheckBoxes.get(Modules.NETWORK_PROCESSOR).setDisable(!isSimulationSelected);
         }
      };
      moduleCheckBoxes.get(Modules.NETWORK_PROCESSOR).selectedProperty().addListener(networkProcessorCheckBoxChangeListener);
      moduleCheckBoxes.get(Modules.SIMULATION).selectedProperty().addListener(simulationCheckBoxChangeListener);

      // Call the listeners
      simulationCheckBoxChangeListener.changed(null, null, null);
      networkProcessorCheckBoxChangeListener.changed(null, null, null);

      Map<String, T> possibleStartingLocationMap = new HashMap<>();

      if (possibleStartingLocations != null && possibleStartingLocations.length > 0)
      {
         ObservableList<String> comboBoxItems = FXCollections.observableArrayList();

         for (T possibleStartingLocation : possibleStartingLocations)
         {
            possibleStartingLocationMap.put(possibleStartingLocation.toString(), possibleStartingLocation);
            comboBoxItems.add(possibleStartingLocation.toString());
         }

         startingLocationComboBox.setItems(comboBoxItems);

         startingLocationComboBox.getSelectionModel().select(properties.getProperty(STARTING_LOCATION_PROPERTY_NAME, possibleStartingLocations[0].toString()));
      }
      else
      {
         startingLocationComboBox.setDisable(true);

      }

      stage.getIcons().add(new Image(DRCSimulationTools.class.getClassLoader().getResourceAsStream("running-man-32x32-Launch.png")));

      okButton.setOnAction(e -> stage.close());
      cancelButton.setOnAction(e -> System.exit(-1));

      stage.addEventFilter(KeyEvent.KEY_RELEASED, e ->
      {
         if (e.getCode() == KeyCode.ESCAPE)
            cancelButton.fire();
         else if (e.getCode() == KeyCode.ENTER)
            okButton.fire();
      });

      stage.sizeToScene();
      stage.showAndWait();

      properties = new Properties();
      for (Modules module : Modules.values())
      {
         boolean selected = moduleCheckBoxes.get(module).isSelected();
         boolean enabled = !moduleCheckBoxes.get(module).isDisable();
         if (selected && enabled)
            modulesToStart.add(module);

         properties.setProperty(module.getPropertyNameForEnable(), String.valueOf(enabled));
         properties.setProperty(module.getPropertyNameForSelected(), String.valueOf(selected));
      }

      if (!startingLocationComboBox.isDisabled() && startingLocationComboBox.getSelectionModel().getSelectedItem() != null)
      {
         properties.setProperty(STARTING_LOCATION_PROPERTY_NAME, startingLocationComboBox.getSelectionModel().getSelectedItem());
      }

      FileOutputStream newConfigOutputStream;
      try
      {
         newConfigOutputStream = new FileOutputStream(configFile);
         properties.store(newConfigOutputStream, "Default configuration for the graphic selector of the DRCSimulationTools");
         newConfigOutputStream.close();
      }
      catch (IOException e)
      {
      }

      if (startingLocationComboBox.isDisabled())
         return null;
      else
         return possibleStartingLocationMap.get(startingLocationComboBox.getSelectionModel().getSelectedItem());
   }

   public List<Modules> getModulesToStart()
   {
      return modulesToStart;
   }
}
