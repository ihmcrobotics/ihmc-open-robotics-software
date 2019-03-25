package us.ihmc.parameterTuner.guiElements.tabs;

import java.util.Map;

import javafx.scene.control.ContextMenu;
import javafx.scene.control.MenuItem;
import javafx.scene.control.Tab;
import javafx.scene.control.TabPane;
import us.ihmc.parameterTuner.guiElements.tuners.Tuner;
import us.ihmc.robotics.sliderboard.Sliderboard;

public class TuningTabManager
{
   private final TabPane tabPane;
   private Map<String, Tuner> tunerMap;

   private final MenuItem closeTab = new MenuItem("Close Open Tab");
   private final MenuItem createNewTab = new MenuItem("New Tab");
   private final MenuItem saveTab = new MenuItem("Save Open Tab");
   private final MenuItem loadTab = new MenuItem("Load Tab");

   private final TabSaver tabSaver;

   private final Sliderboard sliderboard = new Sliderboard();

   public TuningTabManager(TabPane tabPane)
   {
      this.tabPane = tabPane;
      tabSaver = new TabSaver(tabPane);

      closeTab.setOnAction(event -> {
         TuningTab selectedTab = (TuningTab) tabPane.getSelectionModel().getSelectedItem();
         tabPane.getTabs().remove(selectedTab);
         tabSaver.removeTab(selectedTab);
         updateMenuItems();
      });
      createNewTab.setOnAction(event -> {
         createNewTab(tabPane);
      });
      saveTab.setOnAction(event -> {
         TuningTab selectedTab = (TuningTab) tabPane.getSelectionModel().getSelectedItem();
         if (selectedTab != null)
         {
            tabSaver.saveTab(selectedTab);
         }
      });
      loadTab.setOnAction(event -> {
         TuningTab newTab = tabSaver.loadTab(tabPane, tunerMap);
         if (newTab != null)
         {
            newTab.setSliderboard(sliderboard);
            updateMenuItems();
         }
      });

      ContextMenu tabContextMenu = new ContextMenu();
      tabContextMenu.getItems().add(createNewTab);
      tabContextMenu.getItems().add(loadTab);
      tabContextMenu.getItems().add(saveTab);
      tabContextMenu.getItems().add(closeTab);
      tabPane.setContextMenu(tabContextMenu);

      updateMenuItems();
   }

   private void createNewTab(TabPane tabPane)
   {
      String name = createUniqueName(tabPane, "NewTab");
      TuningTab newTab = new TuningTab(name, tabPane);
      newTab.setTunerMap(tunerMap);
      newTab.setSliderboard(sliderboard);
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
      tabPane.getTabs().clear();
      tabSaver.loadDefaultTabs(tabPane, tunerMap);
      tabPane.getTabs().forEach(tab -> {
         ((TuningTab) tab).setSliderboard(sliderboard);
         ((TuningTab) tab).hide();
      });
      if (!tabPane.getSelectionModel().isEmpty())
      {
         tabPane.getSelectionModel().select(0);
         ((TuningTab) tabPane.getSelectionModel().getSelectedItem()).updateView();
      }
      updateMenuItems();
   }

   public void handleNewParameter(String uniqueName)
   {
      if (tabPane.getTabs().isEmpty())
      {
         createNewTab(tabPane);
      }

      TuningTab activeTab = (TuningTab) tabPane.getSelectionModel().getSelectedItem();
      activeTab.handleNewParameter(uniqueName, -1);
   }

   public void close()
   {
      sliderboard.close();
   }
}
