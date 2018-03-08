package us.ihmc.parameterTuner.guiElements.tabs;

import javafx.scene.control.ContextMenu;
import javafx.scene.control.MenuItem;
import javafx.scene.control.Tab;
import javafx.scene.control.TabPane;
import us.ihmc.parameterTuner.guiElements.tuners.TuningBoxManager;

public class TuningTabManager
{
   private final TabPane tabPane;
   private final TuningBoxManager tuningBoxManager;

   public TuningTabManager(TabPane tabPane)
   {
      this.tabPane = tabPane;

      MenuItem closeTab = new MenuItem("Close Open Tab");
      closeTab.setOnAction(event -> {
         Tab selectedTab = tabPane.getSelectionModel().getSelectedItem();
         tabPane.getTabs().remove(selectedTab);
         closeTab.setDisable(tabPane.getTabs().size() == 0);
      });
      MenuItem createNewTab = new MenuItem("New Tab");
      createNewTab.setOnAction(event -> {
         String name = createUniqueName("NewTab");
         TuningTab newTab = new TuningTab(name);
         tabPane.getTabs().add(newTab);
         closeTab.setDisable(false);
      });
      ContextMenu tabContextMenu = new ContextMenu();
      tabContextMenu.getItems().add(createNewTab);
      tabContextMenu.getItems().add(closeTab);
      tabPane.setContextMenu(tabContextMenu);

      TuningTab tuningTab = new TuningTab("TuningTab");
      tabPane.getTabs().add(tuningTab);
      tuningBoxManager = new TuningBoxManager(tuningTab.getTuningBox());
   }

   public TuningBoxManager getTuningBoxManager()
   {
      return tuningBoxManager;
   }

   private String createUniqueName(String name)
   {
      String uniqueName = name;
      int count = 1;
      while (doesTabExist(uniqueName))
      {
         uniqueName = name + count++;
      }
      return uniqueName;
   }

   private boolean doesTabExist(String name)
   {
      return !tabPane.getTabs().filtered(tab -> ((TuningTab) tab).getName().equals(name)).isEmpty();
   }

}
