package us.ihmc.humanoidBehaviors.ui.slam;

import javafx.fxml.FXML;
import javafx.scene.Group;
import javafx.scene.control.Button;
import javafx.scene.control.CheckBox;
import javafx.scene.control.RadioButton;
import javafx.scene.control.TextField;
import javafx.stage.Window;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.humanoidBehaviors.tools.FakeREAModule;
import us.ihmc.humanoidBehaviors.tools.perception.PlanarRegionSLAM;
import us.ihmc.humanoidBehaviors.tools.perception.PlanarRegionSLAMParameters;
import us.ihmc.humanoidBehaviors.tools.perception.PlanarRegionSLAMResult;
import us.ihmc.pathPlanning.visibilityGraphs.ui.graphics.PlanarRegionsGraphic;
import us.ihmc.humanoidBehaviors.ui.graphics.live.LivePlanarRegionsGraphic;
import us.ihmc.humanoidBehaviors.ui.slam.PlanarRegionSLAMGraphic.SLAMVisualizationState;
import us.ihmc.javaFXVisualizers.PrivateAnimationTimer;
import us.ihmc.log.LogTools;
import us.ihmc.robotEnvironmentAwareness.ui.io.PlanarRegionDataExporter;
import us.ihmc.robotEnvironmentAwareness.ui.io.PlanarRegionDataImporter;
import us.ihmc.robotics.PlanarRegionFileTools;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.ros2.Ros2Node;

import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.List;

import static us.ihmc.humanoidBehaviors.ui.slam.PlanarRegionSLAMGraphic.SLAMVisualizationState.Hidden;

public class PlanarRegionSLAMUITabController extends Group
{
   public static final PlanarRegionsList EMPTY_REGIONS_LIST = new PlanarRegionsList();

   private static final String DATASET_1 = "20190710_174025_PlanarRegion";
   private static final String DATASET_2 = "IntentionallyDrifted";
   private static final String DATASET_3 = "20190710_174422_PlanarRegion";

   @FXML private CheckBox acceptNewRegionListsCheckbox;
   @FXML private Button slamButton;
   @FXML private Button slamStepButton;
   @FXML private Button clearIncomingButton;
   @FXML private TextField slamStepStatus;

   @FXML private CheckBox fakeREAPublisherCheckbox;
   @FXML private RadioButton dataset1RadioButton;
   @FXML private RadioButton dataset2RadioButton;
   @FXML private RadioButton dataset3RadioButton;
   @FXML private RadioButton loadFromFileRadioButton;

   private List<RadioButton> datasetSelectionRadioButtons = new ArrayList<>();

   private PrivateAnimationTimer animationTimer = new PrivateAnimationTimer(this::fxUpdate);

   private Window window;
   private LivePlanarRegionsGraphic livePlanarRegionsGraphic;
   private FakeREAModule fakeREAModule;

   private PlanarRegionsList map = new PlanarRegionsList();
   private PlanarRegionsGraphic mapGraphic;

   private PlanarRegionSLAMGraphic visualizer;

   public void init(Window window, Ros2Node ros2Node)
   {
      this.window = window;

      dataset1RadioButton.setText(DATASET_1);
      dataset2RadioButton.setText(DATASET_2);
      dataset3RadioButton.setText(DATASET_3);
      datasetSelectionRadioButtons.add(dataset1RadioButton);
      datasetSelectionRadioButtons.add(dataset2RadioButton);
      datasetSelectionRadioButtons.add(dataset3RadioButton);
      datasetSelectionRadioButtons.add(loadFromFileRadioButton);

      livePlanarRegionsGraphic = new LivePlanarRegionsGraphic(ros2Node, false);
      getChildren().add(livePlanarRegionsGraphic);

      mapGraphic = new PlanarRegionsGraphic(false);
      mapGraphic.generateMeshesAsync(map);
      getChildren().add(mapGraphic);

      visualizer = new PlanarRegionSLAMGraphic();
      visualizer.setStateListener(this::onVisualizerStateChange);

      fakeREAModule = new FakeREAModule(loadDataSet(DATASET_1));

      animationTimer.start();
   }

   private void fxUpdate(long now)
   {
      mapGraphic.update();
   }

   private PlanarRegionsList loadDataSet(String dataSetName)
   {
      String prefix = "ihmc-open-robotics-software/robot-environment-awareness/Data/PlanarRegion/190710_SLAM_PlanarRegionFittingExamples/";
      Path path = Paths.get(prefix + dataSetName);
      return PlanarRegionFileTools.importPlanarRegionData(path.toFile());
   }

   private void setRadioButtonSelection(RadioButton selection)
   {
      datasetSelectionRadioButtons.forEach(it -> it.setSelected(false));
      selection.setSelected(true);
   }

   private void slam()
   {
      // TODO: Put these parameters on the GUI
      PlanarRegionSLAMParameters parameters = new PlanarRegionSLAMParameters();
      parameters.setIterations(3);
      parameters.setBoundingBoxHeight(0.05);

      PlanarRegionsList newData = livePlanarRegionsGraphic.getLatestPlanarRegionsList();
      PlanarRegionSLAMResult slamResult = PlanarRegionSLAM.slam(map, newData, parameters);
//      PlanarRegionSLAMResult slamResult = PlanarRegionSLAM.intentionallyDrift(livePlanarRegionsGraphic.getLatestPlanarRegionsList());
            
      RigidBodyTransform transformFromIncomingToMap = slamResult.getTransformFromIncomingToMap();
      LogTools.info("\nSlam result: transformFromIncomingToMap = \n" + transformFromIncomingToMap);

      map = slamResult.getMergedMap();
      mapGraphic.generateMeshesAsync(map);

      slamStepButton.setDisable(false);  // map must exist before step enabled
   }

   private void onVisualizerStateChange(SLAMVisualizationState state)
   {
      slamStepStatus.setText(state.name());

      if (state == Hidden)
      {
         slamButton.setDisable(false);
         livePlanarRegionsGraphic.setAcceptNewRegions(acceptNewRegionListsCheckbox.isSelected());
         acceptNewRegionListsCheckbox.setDisable(false);

         getChildren().add(mapGraphic);
         getChildren().add(livePlanarRegionsGraphic);
         getChildren().remove(visualizer);
      }
   }

   @FXML private void acceptNewRegionListsCheckbox()
   {
      livePlanarRegionsGraphic.setAcceptNewRegions(acceptNewRegionListsCheckbox.isSelected());
   }

   @FXML private void slamButton()
   {
      ThreadTools.startAThread(this::slam, "SLAM");
   }

   @FXML private void slamStepButton()
   {
      LogTools.info("slamStepButton() visualizer: {}", visualizer.getState().name());
      if (visualizer.getState() == Hidden)
      {
         slamButton.setDisable(true);
         livePlanarRegionsGraphic.setAcceptNewRegions(false);
         acceptNewRegionListsCheckbox.setDisable(true);

         visualizer.copyDataIn(map, livePlanarRegionsGraphic.getLatestPlanarRegionsList());

         getChildren().remove(mapGraphic);
         getChildren().remove(livePlanarRegionsGraphic);
         getChildren().add(visualizer);
      }

      visualizer.step();
   }

   @FXML private void exportMapButton()
   {
      PlanarRegionDataExporter.exportUsingFileChooser(window, map);
   }

   @FXML private void clearMapButton()
   {
      map.clear();
      mapGraphic.generateMeshesAsync(map);
   }

   @FXML private void clearIncomingButton()
   {
      livePlanarRegionsGraphic.clear();
   }

   @FXML private void exportIncomingButton()
   {
      PlanarRegionDataExporter.exportUsingFileChooser(window, livePlanarRegionsGraphic.getLatestPlanarRegionsList());
   }

   @FXML private void fakeREAPublisherCheckbox()
   {
      if (fakeREAPublisherCheckbox.isSelected())
      {
         fakeREAModule.start();
      }
      else
      {
         fakeREAModule.stop();
      }
   }

   @FXML private void dataset1RadioButton()
   {
      setRadioButtonSelection(dataset1RadioButton);
      fakeREAModule.setRegionsToPublish(loadDataSet(DATASET_1));
   }

   @FXML private void dataset2RadioButton()
   {
      setRadioButtonSelection(dataset2RadioButton);
      fakeREAModule.setRegionsToPublish(loadDataSet(DATASET_2));
   }

   @FXML private void dataset3RadioButton()
   {
      setRadioButtonSelection(dataset3RadioButton);
      fakeREAModule.setRegionsToPublish(loadDataSet(DATASET_3));
   }

   @FXML private void loadFromFileRadioButton()
   {
      setRadioButtonSelection(loadFromFileRadioButton);
      fakeREAModule.setRegionsToPublish(PlanarRegionDataImporter.importUsingFileChooser(window));
   }
}
