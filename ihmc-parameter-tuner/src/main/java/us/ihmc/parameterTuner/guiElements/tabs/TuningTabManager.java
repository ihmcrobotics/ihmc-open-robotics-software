package us.ihmc.parameterTuner.guiElements.tabs;

import java.util.Map;

import javafx.scene.control.ContextMenu;
import javafx.scene.control.MenuItem;
import javafx.scene.control.Tab;
import javafx.scene.control.TabPane;
import us.ihmc.parameterTuner.guiElements.tuners.Tuner;

public class TuningTabManager
{
   private final TabPane tabPane;
   private Map<String, Tuner> tunerMap;

   private final MenuItem closeTab = new MenuItem("Close Open Tab");
   private final MenuItem createNewTab = new MenuItem("New Tab");
   private final MenuItem saveTab = new MenuItem("Save Open Tab");
   private final MenuItem loadTab = new MenuItem("Load Tab");

   public TuningTabManager(TabPane tabPane)
   {
      this.tabPane = tabPane;

      closeTab.setOnAction(event -> {
         Tab selectedTab = tabPane.getSelectionModel().getSelectedItem();
         tabPane.getTabs().remove(selectedTab);
         updateMenuItems();
      });
      createNewTab.setOnAction(event -> {
         String name = createUniqueName(tabPane, "NewTab");
         TuningTab newTab = new TuningTab(name, tabPane);
         newTab.setTunerMap(tunerMap);
         updateMenuItems();
      });
      saveTab.setOnAction(event -> {
         TuningTab selectedTab = (TuningTab) tabPane.getSelectionModel().getSelectedItem();
         TabSavingTools.saveTab(selectedTab, tabPane.getScene().getWindow());
      });
      loadTab.setOnAction(event -> {
         TabSavingTools.loadTab(tabPane, tunerMap, tabPane.getScene().getWindow());
         updateMenuItems();
      });

      ContextMenu tabContextMenu = new ContextMenu();
      tabContextMenu.getItems().add(createNewTab);
      tabContextMenu.getItems().add(loadTab);
      tabContextMenu.getItems().add(saveTab);
      tabContextMenu.getItems().add(closeTab);
      tabPane.setContextMenu(tabContextMenu);

      updateMenuItems();
   }

   private void updateMenuItems()
   {
      closeTab.setDisable(tabPane.getTabs().size() == 0);
      saveTab.setDisable(tabPane.getTabs().size() == 0);
   }

   public static String createUniqueName(TabPane tabPane, String name)
   {
      String uniqueName = name;
      int count = 1;
      while (doesTabExist(tabPane, uniqueName))
      {
         uniqueName = name + count++;
      }
      return uniqueName;
   }

   private static boolean doesTabExist(TabPane tabPane, String name)
   {
      return !tabPane.getTabs().filtered(tab -> name.equals(((TuningTab) tab).getName())).isEmpty();
   }

   public void setTunerMap(Map<String, Tuner> tunerMap)
   {
      this.tunerMap = tunerMap;
      tabPane.getTabs().forEach(tab -> ((TuningTab) tab).setTunerMap(tunerMap));
   }

   public void handleNewParameter(String uniqueName)
   {
      TuningTab activeTab = (TuningTab) tabPane.getSelectionModel().getSelectedItem();
      activeTab.handleNewParameter(uniqueName);
   }

}
