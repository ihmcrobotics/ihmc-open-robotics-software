package us.ihmc.robotEnvironmentAwareness.ui.controller;

import java.io.IOException;
import java.net.URL;

import javafx.fxml.FXML;
import javafx.fxml.FXMLLoader;
import javafx.scene.Scene;
import javafx.scene.control.Slider;
import javafx.scene.control.Spinner;
import javafx.scene.control.SpinnerValueFactory.IntegerSpinnerValueFactory;
import javafx.scene.control.ToggleButton;
import javafx.scene.layout.BorderPane;
import javafx.stage.Stage;
import us.ihmc.javaFXToolkit.messager.MessageBidirectionalBinding.PropertyToMessageTypeConverter;
import us.ihmc.javaFXToolkit.scenes.View3DFactory;
import us.ihmc.robotEnvironmentAwareness.communication.REAModuleAPI;
import us.ihmc.robotEnvironmentAwareness.communication.REAUIMessager;
import us.ihmc.robotEnvironmentAwareness.slam.viewer.IhmcSLAMMeshViewer;
import us.ihmc.robotEnvironmentAwareness.ui.UIConnectionHandler;

public class PointCloudAnchorPaneController extends REABasicUIController
{
   public static REAUIMessager uiStaticMessager;
   @FXML
   private ToggleButton enableLidarButton;
   @FXML
   private Slider scanHistorySizeSlider;
   @FXML
   private ToggleButton enableStereoButton;
   @FXML
   private ToggleButton enableDepthButton;
   @FXML
   private Spinner<Integer> sizeOfPointCloudSpinner;
   @FXML
   private Slider navigationFramesSlider;

   private static final int maximumSizeOfPointCloud = 200000;
   private static final int minimumSizeOfPointCloud = 1000;
   public static final int initialSizeOfPointCloud = 5000;

   private final PropertyToMessageTypeConverter<Integer, Number> numberToIntegerConverter = new PropertyToMessageTypeConverter<Integer, Number>()
   {
      @Override
      public Integer convert(Number propertyValue)
      {
         return propertyValue.intValue();
      }

      @Override
      public Number interpret(Integer newValue)
      {
         return new Double(newValue.intValue());
      }
   };

   public PointCloudAnchorPaneController()
   {
   }

   public void bindControls()
   {
      load();
      sizeOfPointCloudSpinner.setValueFactory(createNumberOfPointsValueFactory(initialSizeOfPointCloud));
      uiMessager.bindBidirectionalInternal(REAModuleAPI.UILidarScanShow, enableLidarButton.selectedProperty(), true);
      uiMessager.bindBidirectionalInternal(REAModuleAPI.UILidarScanSize, scanHistorySizeSlider.valueProperty(), numberToIntegerConverter, true);
      uiMessager.bindBidirectionalInternal(REAModuleAPI.UIStereoVisionShow, enableStereoButton.selectedProperty(), true);
      uiMessager.bindBidirectionalInternal(REAModuleAPI.UIDepthCloudShow, enableDepthButton.selectedProperty(), true);

      uiMessager.bindBidirectionalGlobal(REAModuleAPI.UIStereoVisionSize, sizeOfPointCloudSpinner.getValueFactory().valueProperty());
      uiMessager.bindBidirectionalInternal(REAModuleAPI.UISensorPoseHistoryFrames, navigationFramesSlider.valueProperty(), numberToIntegerConverter, true);

      ihmcSLAMViewer = new IhmcSLAMMeshViewer(uiMessager);
   }

   @FXML
   public void clearLidar()
   {
      uiMessager.submitMessageInternal(REAModuleAPI.UILidarScanClear, true);
   }

   @FXML
   public void clearStereo()
   {
      uiMessager.submitMessageInternal(REAModuleAPI.UIStereoVisionClear, true);
   }

   @FXML
   public void clearDepth()
   {
      uiMessager.submitMessageInternal(REAModuleAPI.UIDepthCloudClear, true);
   }

   @FXML
   public void save()
   {
      saveUIControlProperty(REAModuleAPI.UILidarScanShow, enableLidarButton);
      saveUIControlProperty(REAModuleAPI.UILidarScanSize, scanHistorySizeSlider);
      saveUIControlProperty(REAModuleAPI.UIStereoVisionShow, enableStereoButton);
   }

   @FXML
   public void clearNavigation()
   {
      uiMessager.submitMessageInternal(REAModuleAPI.UISensorPoseHistoryClear, true);
   }

   public void load()
   {
      loadUIControlProperty(REAModuleAPI.UILidarScanShow, enableLidarButton);
      loadUIControlProperty(REAModuleAPI.UILidarScanSize, scanHistorySizeSlider);
      loadUIControlProperty(REAModuleAPI.UIStereoVisionShow, enableStereoButton);
   }

   private IntegerSpinnerValueFactory createNumberOfPointsValueFactory(int initialValue)
   {
      int min = minimumSizeOfPointCloud;
      int max = maximumSizeOfPointCloud;
      int amountToStepBy = minimumSizeOfPointCloud;
      return new IntegerSpinnerValueFactory(min, max, initialValue, amountToStepBy);
   }

   @FXML
   private SLAMAnchorPaneController slamAnchorPaneController;
   private IhmcSLAMMeshViewer ihmcSLAMViewer;
   private UIConnectionHandler uiConnectionHandler;

   @FXML
   public void openSLAM() throws IOException
   {
      uiStaticMessager = uiMessager;

      FXMLLoader loader = new FXMLLoader();
      URL url = getClass().getResource("../../ui/SLAMVisualizerMainPane" + ".fxml");
      loader.setLocation(url);

      if (url != null)
      {
         BorderPane mainPane = loader.load();

         View3DFactory view3dFactory = View3DFactory.createSubscene();
         view3dFactory.addCameraController(true);
         view3dFactory.addWorldCoordinateSystem(0.3);
         view3dFactory.addNodeToView(ihmcSLAMViewer.getRoot());
         mainPane.setCenter(view3dFactory.getSubSceneWrappedInsidePane());

         Stage stage = new Stage();
         Scene mainScene = new Scene(mainPane, 1000, 800);
         stage.setScene(mainScene);
         stage.setOnCloseRequest(event -> ihmcSLAMViewer.stop());

         uiConnectionHandler = new UIConnectionHandler(stage, uiMessager);
         uiConnectionHandler.start();

         stage.show();
      }
   }
}
