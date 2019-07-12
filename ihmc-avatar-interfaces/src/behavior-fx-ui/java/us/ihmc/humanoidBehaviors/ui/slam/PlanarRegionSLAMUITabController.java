package us.ihmc.humanoidBehaviors.ui.slam;

import javafx.fxml.FXML;
import javafx.scene.Group;
import javafx.scene.control.CheckBox;
import us.ihmc.humanoidBehaviors.ui.graphics.PlanarRegionsGraphic;
import us.ihmc.robotics.PlanarRegionFileTools;
import us.ihmc.robotics.geometry.PlanarRegionsList;

import java.nio.file.Path;
import java.nio.file.Paths;

public class PlanarRegionSLAMUITabController extends Group
{
   public static final PlanarRegionsList EMPTY_REGIONS_LIST = new PlanarRegionsList();

   private static final String DATASET_1 = "20190710_174025_PlanarRegion";
   private static final String DATASET_2 = "20190710_174208_PlanarRegion";
   private static final String DATASET_3 = "20190710_174422_PlanarRegion";

   @FXML private CheckBox regionSet1;
   @FXML private CheckBox regionSet2;
   @FXML private CheckBox regionSet3;

   private PlanarRegionsGraphic regionsGraphicOne;
   private PlanarRegionsGraphic regionsGraphicTwo;
   private PlanarRegionsGraphic regionsGraphicThree;

   public void init()
   {
      regionsGraphicOne = createPlanarRegionsGraphic(DATASET_1);
      regionsGraphicTwo = createPlanarRegionsGraphic(DATASET_2);
      regionsGraphicThree = createPlanarRegionsGraphic(DATASET_3);
   }

   private PlanarRegionsGraphic createPlanarRegionsGraphic(String dataSetName)
   {
      PlanarRegionsGraphic regionsGraphic = new PlanarRegionsGraphic();
      generateRegions(regionsGraphic, loadDataSet(dataSetName));
      getChildren().add(regionsGraphic);
      return regionsGraphic;
   }

   private PlanarRegionsList loadDataSet(String dataSetName)
   {
      String prefix = "ihmc-open-robotics-software/robot-environment-awareness/Data/PlanarRegion/190710_SLAM_PlanarRegionFittingExamples/";
      Path path = Paths.get(prefix + dataSetName);
      return PlanarRegionFileTools.importPlanarRegionData(path.toFile());
   }

   private void generateRegions(PlanarRegionsGraphic graphic, PlanarRegionsList list)
   {
      graphic.generateMeshes(list);
      graphic.update();
   }

   @FXML private void regionSet1()
   {
      if (regionSet1.isSelected())
      {
         generateRegions(regionsGraphicOne, loadDataSet(DATASET_1));
      }
      else
      {
         generateRegions(regionsGraphicOne, EMPTY_REGIONS_LIST);
      }
   }

   @FXML private void regionSet2()
   {
      if (regionSet2.isSelected())
      {
         generateRegions(regionsGraphicTwo, loadDataSet(DATASET_2));
      }
      else
      {
         generateRegions(regionsGraphicTwo, EMPTY_REGIONS_LIST);
      }
   }

   @FXML private void regionSet3()
   {
      if (regionSet3.isSelected())
      {
         generateRegions(regionsGraphicThree, loadDataSet(DATASET_3));
      }
      else
      {
         generateRegions(regionsGraphicThree, EMPTY_REGIONS_LIST);
      }
   }
}
