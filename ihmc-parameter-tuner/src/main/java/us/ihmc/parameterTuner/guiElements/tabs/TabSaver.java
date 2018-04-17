package us.ihmc.parameterTuner.guiElements.tabs;

import java.io.File;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Optional;

import org.apache.commons.lang3.tuple.ImmutablePair;

import javafx.scene.control.TabPane;
import javafx.stage.Window;
import us.ihmc.parameterTuner.guiElements.tuners.Tuner;

public class TabSaver
{
   private final TabPane tabPane;
   private final List<ImmutablePair<TuningTab, File>> activeTabs = new ArrayList<>();

   public TabSaver(TabPane tabPane)
   {
      this.tabPane = tabPane;
   }

   public void saveTab(TuningTab tab)
   {
      removeTab(tab);
      File file = TabSavingTools.saveTab(tab, getWindow());
      ImmutablePair<TuningTab, File> filePair = new ImmutablePair<>(tab, file);
      activeTabs.add(filePair);
      saveDefaultTabs();
   }

   public TuningTab loadTab(TabPane tabPane, Map<String, Tuner> tunerMap)
   {
      ImmutablePair<TuningTab, File> filePair = TabSavingTools.loadTab(tabPane, tunerMap, getWindow());
      activeTabs.add(filePair);
      saveDefaultTabs();
      return filePair.getLeft();
   }

   public void loadDefaultTabs(TabPane tabPane, Map<String, Tuner> tunerMap)
   {
      activeTabs.clear();
      List<ImmutablePair<TuningTab, File>> filePairs = TabSavingTools.loadDefaultTabs(tabPane, tunerMap);
      activeTabs.addAll(filePairs);
   }

   public void removeTab(TuningTab tab)
   {
      Optional<ImmutablePair<TuningTab, File>> match = activeTabs.stream().filter(filePair -> filePair.getKey() == tab).findFirst();
      if (match.isPresent())
      {
         activeTabs.remove(match.get());
      }
      saveDefaultTabs();
   }

   private void saveDefaultTabs()
   {
      List<File> files = new ArrayList<>();
      activeTabs.forEach(filePair -> files.add(filePair.getValue()));
      TabSavingTools.saveDefaultTabFiles(files);
   }

   private Window getWindow()
   {
      return tabPane.getScene().getWindow();
   }

}
