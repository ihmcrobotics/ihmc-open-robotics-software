package us.ihmc.pathPlanning.visibilityGraphs.ui.controllers;

import static us.ihmc.pathPlanning.visibilityGraphs.ui.messager.UIVisibilityGraphsTopics.RandomizePlanarRegionIDRequest;

import java.io.File;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.stream.Collectors;

import javafx.fxml.FXML;
import javafx.scene.control.Accordion;
import javafx.scene.control.ListView;
import javafx.scene.input.MouseButton;
import javafx.scene.input.MouseEvent;
import javafx.scene.text.Text;
import javafx.stage.DirectoryChooser;
import javafx.stage.Window;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.messager.javafx.JavaFXMessager;
import us.ihmc.pathPlanning.DataSet;
import us.ihmc.pathPlanning.DataSetIOTools;
import us.ihmc.pathPlanning.visibilityGraphs.ui.messager.UIVisibilityGraphsTopics;
import us.ihmc.robotics.PlanarRegionFileTools;
import us.ihmc.robotics.geometry.PlanarRegionsList;

public class DatasetNavigationAccordionController
{
   private final ArrayList<DataSet> visualizerDataSets = new ArrayList<>();
   private final ArrayList<DataSet> testableDataSets = new ArrayList<>();
   private final ArrayList<DataSet> inDevelopmentDataSet = new ArrayList<>();
   private File customDataFolder = null;

   @FXML
   private Accordion datasetNavigationAccordion;

   @FXML
   private ListView<String> visualizerDataListView, testDataListView, inDevelopmentTestDataListView, customDataListView;

   private JavaFXMessager messager;
   private Window ownerWindow;

   public DatasetNavigationAccordionController()
   {
      List<DataSet> dataSets = DataSetIOTools.loadDataSets();
      for (int i = 0; i < dataSets.size(); i++)
      {
         DataSet dataSet = dataSets.get(i);
         if(dataSet.hasPlannerInput())
         {
            if(dataSet.getPlannerInput().getVisGraphIsTestable())
            {
               testableDataSets.add(dataSet);
            }
            else if(dataSet.getPlannerInput().getVisGraphIsInDevelopment())
            {
               inDevelopmentDataSet.add(dataSet);
            }
         }
         else
         {
            visualizerDataSets.add(dataSet);
         }
      }
   }

   public void attachMessager(JavaFXMessager messager)
   {
      this.messager = messager;
   }

   public void setMainWindow(Window ownerWindow)
   {
      this.ownerWindow = ownerWindow;
   }

   public void bindControls()
   {
   }

   @FXML
   public void load()
   {
      visualizerDataListView.getItems().clear();
      visualizerDataListView.getItems().addAll(visualizerDataSets.stream().map(DataSet::getName).collect(Collectors.toList()));

      testDataListView.getItems().clear();
      testDataListView.getItems().addAll(testableDataSets.stream().map(DataSet::getName).collect(Collectors.toList()));

      inDevelopmentTestDataListView.getItems().clear();
      inDevelopmentTestDataListView.getItems().addAll(inDevelopmentDataSet.stream().map(DataSet::getName).collect(Collectors.toList()));

      customDataListView.getItems().clear();
      if (customDataFolder != null && customDataFolder.exists() && customDataFolder.isDirectory())
         customDataListView.getItems().addAll(getPlanarRegionAndDataSetFilenames(customDataFolder));
   }

   public static String[] getPlanarRegionAndDataSetFilenames(File parentFolder)
   {
      if (!parentFolder.exists() || !parentFolder.isDirectory())
         return null;

      return Arrays.stream(parentFolder.listFiles(file -> DataSetIOTools.isDataSetFile(file, false) || PlanarRegionFileTools.isPlanarRegionFile(file))).map(File::getName)
         .toArray(String[]::new);
   }

   @FXML
   private void loadCustomDataFolder()
   {
      DirectoryChooser directoryChooser = new DirectoryChooser();
      File result = directoryChooser.showDialog(ownerWindow);
      if (result == null)
         return;

      customDataFolder = result;
      load();
   }

   @FXML
   private void requestRandomizeRegionIDs()
   {
      messager.submitMessage(RandomizePlanarRegionIDRequest, true);
   }

   @FXML
   private void requestNewVisualizerData(MouseEvent event)
   {
      loadDataSet(visualizerDataListView, event);
   }

   @FXML
   private void requestNewTestData(MouseEvent event)
   {
      loadDataSet(testDataListView, event);
   }

   @FXML
   private void requestNewInDevelopmentTestData(MouseEvent event)
   {
      loadDataSet(inDevelopmentTestDataListView, event);
   }

   @FXML
   private void requestNewCustomData(MouseEvent event)
   {
      loadPlanarRegionsOrDataSet(customDataListView, event);
   }

   private void loadPlanarRegionsOrDataSet(ListView<String> listViewOwner, MouseEvent event)
   {
      if (!hasListViewCellBeenDoubleClicked(event))
         return;

      String directoryName = listViewOwner.getSelectionModel().getSelectedItem();
      File file = new File(customDataFolder.getAbsolutePath() + File.separator + directoryName);
      if(DataSetIOTools.isDataSetFile(file, false))
      {
         loadDataSet(listViewOwner, event);
      }
      else
      {
         PlanarRegionsList loadedPlanarRegions = PlanarRegionFileTools.importPlanarRegionData(file);
         messager.submitMessage(UIVisibilityGraphsTopics.GlobalReset, true);
         messager.submitMessage(UIVisibilityGraphsTopics.PlanarRegionData, loadedPlanarRegions);
         messager.submitMessage(UIVisibilityGraphsTopics.StartPosition, new Point3D());
         messager.submitMessage(UIVisibilityGraphsTopics.GoalPosition, new Point3D());
      }
   }

   private void loadDataSet(ListView<String> listViewOwner, MouseEvent event)
   {
      if (!hasListViewCellBeenDoubleClicked(event))
         return;

      String dataSetName = listViewOwner.getSelectionModel().getSelectedItem();
      DataSet dataSet = DataSetIOTools.loadDataSet(dataSetName);

      messager.submitMessage(UIVisibilityGraphsTopics.GlobalReset, true);
      messager.submitMessage(UIVisibilityGraphsTopics.PlanarRegionData, dataSet.getPlanarRegionsList());

      if(dataSet.hasPlannerInput())
      {
         messager.submitMessage(UIVisibilityGraphsTopics.StartPosition, dataSet.getPlannerInput().getStartPosition());
         messager.submitMessage(UIVisibilityGraphsTopics.GoalPosition, dataSet.getPlannerInput().getGoalPosition());
      }
      else
      {
         messager.submitMessage(UIVisibilityGraphsTopics.StartPosition, new Point3D());
         messager.submitMessage(UIVisibilityGraphsTopics.GoalPosition, new Point3D());
      }
   }

   private static File findChildFile(File folder, String childFilename)
   {
      return folder.listFiles((dir, name) -> name.equals(childFilename))[0];
   }

   private static boolean hasListViewCellBeenDoubleClicked(MouseEvent event)
   {
      return event.getButton() == MouseButton.PRIMARY && event.getClickCount() == 2 && event.getTarget() instanceof Text;
   }
}
