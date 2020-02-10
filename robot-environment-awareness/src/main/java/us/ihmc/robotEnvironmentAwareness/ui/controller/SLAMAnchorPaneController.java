package us.ihmc.robotEnvironmentAwareness.ui.controller;

import java.io.IOException;

import javafx.fxml.FXML;
import javafx.fxml.FXMLLoader;
import javafx.scene.Scene;
import javafx.scene.control.Slider;
import javafx.scene.control.ToggleButton;
import javafx.scene.layout.BorderPane;
import javafx.stage.Stage;
import us.ihmc.javaFXToolkit.messager.MessageBidirectionalBinding.PropertyToMessageTypeConverter;
import us.ihmc.javaFXToolkit.scenes.View3DFactory;
import us.ihmc.robotEnvironmentAwareness.communication.REAModuleAPI;
import us.ihmc.robotEnvironmentAwareness.slam.viewer.IhmcSLAMMeshViewer;
import us.ihmc.robotEnvironmentAwareness.ui.UIConnectionHandler;

public class SLAMAnchorPaneController extends REABasicUIController
{
   @FXML
   private ToggleButton enableSLAMButton;

//   private IhmcSLAMMeshViewer ihmcSLAMViewer;

//   private UIConnectionHandler uiConnectionHandler;

//   @FXML
//   private TextField queuedBufferSize;
//   
//   @FXML
//   private TextField slamStatus;

   @FXML
   private ToggleButton latestFrameEnable;

   @FXML
   private ToggleButton octreeMapEnable;

   @FXML
   private ToggleButton sensorFrameEnable;

   @FXML
   private ToggleButton planarRegionsEnable;

   @FXML
   private Slider sourcePointsSlider;

   @FXML
   private Slider searchingSizeSlider;

   @FXML
   private Slider windowMarginSlider;

   @FXML
   private Slider minimumOverlappedRatioSlider;

   @FXML
   private Slider minimumInliersRatioSlider;

   public SLAMAnchorPaneController()
   {

   }
   
   public void initialize()
   {
      
   }

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
         return new Double(newValue.doubleValue());
      }
   };

   private final PropertyToMessageTypeConverter<Double, Number> numberToDoubleConverter = new PropertyToMessageTypeConverter<Double, Number>()
   {
      @Override
      public Double convert(Number propertyValue)
      {
         return propertyValue.doubleValue();
      }

      @Override
      public Number interpret(Double newValue)
      {
         return new Double(newValue.doubleValue());
      }
   };

   @Override
   public void bindControls()
   {
//      ihmcSLAMViewer = new IhmcSLAMMeshViewer(uiMessager);
      uiMessager.bindBidirectionalGlobal(REAModuleAPI.SLAMEnable, enableSLAMButton.selectedProperty());

//      uiMessager.bindBidirectionalGlobal(REAModuleAPI.QueuedBuffers, queuedBufferSize.valueProperty(), numberToIntegerConverter);
//      uiMessager.bindBidirectionalGlobal(REAModuleAPI.SLAMStatus, slamStatus.valueProperty(), numberToIntegerConverter);
      
      uiMessager.bindBidirectionalGlobal(REAModuleAPI.ShowLatestFrame, latestFrameEnable.selectedProperty());
      uiMessager.bindBidirectionalGlobal(REAModuleAPI.ShowSLAMOctreeMap, octreeMapEnable.selectedProperty());
      uiMessager.bindBidirectionalGlobal(REAModuleAPI.ShowSLAMSensorTrajectory, sensorFrameEnable.selectedProperty());
      uiMessager.bindBidirectionalGlobal(REAModuleAPI.ShowPlanarRegionsMap, planarRegionsEnable.selectedProperty());

      uiMessager.bindBidirectionalGlobal(REAModuleAPI.SLAMSourcePoints, sourcePointsSlider.valueProperty(), numberToIntegerConverter);
      uiMessager.bindBidirectionalGlobal(REAModuleAPI.SLAMSearchingSize, searchingSizeSlider.valueProperty(), numberToIntegerConverter);
      uiMessager.bindBidirectionalGlobal(REAModuleAPI.SLAMMinimumOverlappedRatio, minimumOverlappedRatioSlider.valueProperty(), numberToDoubleConverter);
      uiMessager.bindBidirectionalGlobal(REAModuleAPI.SLAMWindowMargin, windowMarginSlider.valueProperty(), numberToDoubleConverter);
      uiMessager.bindBidirectionalGlobal(REAModuleAPI.SLAMMinimumInlierRatio, minimumInliersRatioSlider.valueProperty(), numberToDoubleConverter);

   }

//   public void openMap() throws IOException
//   {
//      FXMLLoader loader = new FXMLLoader();
//      loader.setController(this);
//      loader.setLocation(getClass().getResource("../SLAMVisualizerMainPane" + ".fxml"));
//
//      BorderPane mainPane = loader.load();
//
//      View3DFactory view3dFactory = View3DFactory.createSubscene();
//      view3dFactory.addCameraController(true);
//      view3dFactory.addWorldCoordinateSystem(0.3);
//      view3dFactory.addNodeToView(ihmcSLAMViewer.getRoot());
//      mainPane.setCenter(view3dFactory.getSubSceneWrappedInsidePane());
//
//      Stage stage = new Stage();
//      Scene mainScene = new Scene(mainPane, 600, 400);
//      stage.setScene(mainScene);
//      stage.setOnCloseRequest(event -> stop());
//
//      uiConnectionHandler = new UIConnectionHandler(stage, uiMessager);
//      uiConnectionHandler.start();
//
//      stage.show();
//   }

   public void stop()
   {
      try
      {
//         uiConnectionHandler.stop();
//         ihmcSLAMViewer.stop();
      }
      catch (Exception e)
      {
         e.printStackTrace();
      }
   }

   @FXML
   public void clear()
   {
      System.out.println("Clear");
   }

}
