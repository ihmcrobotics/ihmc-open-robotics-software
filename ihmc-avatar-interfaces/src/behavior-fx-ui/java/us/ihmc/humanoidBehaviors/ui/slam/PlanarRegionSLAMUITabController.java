package us.ihmc.humanoidBehaviors.ui.slam;

import javafx.fxml.FXML;
import javafx.scene.Group;
import javafx.scene.control.Button;
import javafx.scene.control.CheckBox;
import javafx.scene.control.RadioButton;
import javafx.stage.Window;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.humanoidBehaviors.tools.FakeREAModule;
import us.ihmc.humanoidBehaviors.tools.perception.PlanarRegionSLAM;
import us.ihmc.humanoidBehaviors.tools.perception.PlanarRegionSLAMResult;
import us.ihmc.humanoidBehaviors.ui.graphics.PlanarRegionsGraphic;
import us.ihmc.humanoidBehaviors.ui.graphics.live.LivePlanarRegionsGraphic;
import us.ihmc.javaFXVisualizers.PrivateAnimationTimer;
import us.ihmc.robotEnvironmentAwareness.ui.io.PlanarRegionDataExporter;
import us.ihmc.robotEnvironmentAwareness.ui.io.PlanarRegionDataImporter;
import us.ihmc.robotics.PlanarRegionFileTools;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.ros2.Ros2Node;

import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.List;

public class PlanarRegionSLAMUITabController extends Group
{
   public static final PlanarRegionsList EMPTY_REGIONS_LIST = new PlanarRegionsList();

   private static final String DATASET_1 = "20190710_174025_PlanarRegion";
   private static final String DATASET_2 = "20190710_174208_PlanarRegion";
   private static final String DATASET_3 = "20190710_174422_PlanarRegion";

   @FXML private CheckBox acceptNewRegionListsCheckbox;
   @FXML private Button slamButton;
   @FXML private Button exportMapButton;
   @FXML private Button clearMapButton;
   @FXML private Button exportIncomingButton;

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

   public void init(Window window, Ros2Node ros2Node)
   {
      this.window = window;

      datasetSelectionRadioButtons.add(dataset1RadioButton);
      datasetSelectionRadioButtons.add(dataset2RadioButton);
      datasetSelectionRadioButtons.add(dataset3RadioButton);
      datasetSelectionRadioButtons.add(loadFromFileRadioButton);

      livePlanarRegionsGraphic = new LivePlanarRegionsGraphic(ros2Node, false);
      getChildren().add(livePlanarRegionsGraphic);

      mapGraphic = new PlanarRegionsGraphic(false);
      generateOnAThread(mapGraphic, map);
      getChildren().add(mapGraphic);

      fakeREAModule = new FakeREAModule(loadDataSet(DATASET_1));

      animationTimer.start();
   }

   private void fxUpdate(long now)
   {
      mapGraphic.update();
   }

   private void generateOnAThread(PlanarRegionsGraphic regionsGraphicOne, PlanarRegionsList planarRegionsList)
   {
      ThreadTools.startAThread(() -> regionsGraphicOne.generateMeshes(planarRegionsList), "MeshGeneration");
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
      PlanarRegionSLAMResult slamResult = PlanarRegionSLAM.slam(map, livePlanarRegionsGraphic.getLatestPlanarRegionsList());
      map = slamResult.getMergedMap();
      generateOnAThread(mapGraphic, map);
   }

   @FXML private void acceptNewRegionListsCheckbox()
   {
      livePlanarRegionsGraphic.setAcceptNewRegions(acceptNewRegionListsCheckbox.isSelected());
   }

   @FXML private void slamButton()
   {
      ThreadTools.startAThread(this::slam, "SLAM");
   }

   @FXML private void exportMapButton()
   {
      PlanarRegionDataExporter.exportUsingFileChooser(window, map);
   }

   @FXML private void clearMapButton()
   {
      map.clear();
      generateOnAThread(mapGraphic, map);
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
