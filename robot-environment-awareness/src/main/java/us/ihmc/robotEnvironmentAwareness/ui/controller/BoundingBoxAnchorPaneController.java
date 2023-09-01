package us.ihmc.robotEnvironmentAwareness.ui.controller;

import javafx.fxml.FXML;
import javafx.scene.control.Spinner;
import javafx.scene.control.SpinnerValueFactory.DoubleSpinnerValueFactory;
import javafx.scene.control.ToggleButton;
import us.ihmc.messager.MessagerAPIFactory;
import us.ihmc.robotEnvironmentAwareness.communication.SegmentationModuleAPI;
import us.ihmc.robotEnvironmentAwareness.communication.packets.BoundingBoxParametersMessage;
import us.ihmc.robotEnvironmentAwareness.ui.properties.BoundingBoxParametersProperty;

public class BoundingBoxAnchorPaneController extends REABasicUIController
{
   @FXML
   private ToggleButton enableBoundingBoxButton;
   @FXML
   private ToggleButton showBoundingBoxButton;
   @FXML
   private Spinner<Double> boundingBoxMinXSpinner;
   @FXML
   private Spinner<Double> boundingBoxMaxXSpinner;
   @FXML
   private Spinner<Double> boundingBoxMinYSpinner;
   @FXML
   private Spinner<Double> boundingBoxMaxYSpinner;
   @FXML
   private Spinner<Double> boundingBoxMinZSpinner;
   @FXML
   private Spinner<Double> boundingBoxMaxZSpinner;

   private final BoundingBoxParametersProperty boundingBoxParametersProperty = new BoundingBoxParametersProperty(this, "boundingBoxParameters");

   public BoundingBoxAnchorPaneController()
   {
   }

   private void setupControls()
   {
      boundingBoxMinXSpinner.setValueFactory(createBoundingBoxValueFactory(0.0));
      boundingBoxMaxXSpinner.setValueFactory(createBoundingBoxValueFactory(5.0));
      boundingBoxMinYSpinner.setValueFactory(createBoundingBoxValueFactory(-2.0));
      boundingBoxMaxYSpinner.setValueFactory(createBoundingBoxValueFactory(2.0));
      boundingBoxMinZSpinner.setValueFactory(createBoundingBoxValueFactory(-1.0));
      boundingBoxMaxZSpinner.setValueFactory(createBoundingBoxValueFactory(1.0));
   }

   private MessagerAPIFactory.Topic<Boolean> boundingBoxEnableTopic = SegmentationModuleAPI.OcTreeBoundingBoxEnable;
   private MessagerAPIFactory.Topic<Boolean> boundingBoxShowTopic = SegmentationModuleAPI.UIOcTreeBoundingBoxShow;
   private MessagerAPIFactory.Topic<Boolean> saveParameterConfigurationTopic = SegmentationModuleAPI.SaveUpdaterConfiguration;
   private MessagerAPIFactory.Topic<BoundingBoxParametersMessage> boundingBoxParametersTopic = SegmentationModuleAPI.OcTreeBoundingBoxParameters;

   public void setBoundingBoxEnableTopic(MessagerAPIFactory.Topic<Boolean> boundingBoxEnableTopic)
   {
      this.boundingBoxEnableTopic = boundingBoxEnableTopic;
   }

   public void setBoundingBoxShowTopic(MessagerAPIFactory.Topic<Boolean> boundingBoxShowTopic)
   {
      this.boundingBoxShowTopic = boundingBoxShowTopic;
   }

   public void setSaveParameterConfigurationTopic(MessagerAPIFactory.Topic<Boolean> saveParameterConfigurationTopic)
   {
      this.saveParameterConfigurationTopic = saveParameterConfigurationTopic;
   }

   public void setBoundingBoxParametersTopic(MessagerAPIFactory.Topic<BoundingBoxParametersMessage> boundingBoxParametersTopic)
   {
      this.boundingBoxParametersTopic = boundingBoxParametersTopic;
   }

   @Override
   public void bindControls()
   {
      setupControls();

      uiMessager.bindBidirectionalGlobal(boundingBoxEnableTopic, enableBoundingBoxButton.selectedProperty());

      boundingBoxParametersProperty.binBidirectionalMinX(boundingBoxMinXSpinner.getValueFactory().valueProperty());
      boundingBoxParametersProperty.binBidirectionalMinY(boundingBoxMinYSpinner.getValueFactory().valueProperty());
      boundingBoxParametersProperty.binBidirectionalMinZ(boundingBoxMinZSpinner.getValueFactory().valueProperty());
      boundingBoxParametersProperty.binBidirectionalMaxX(boundingBoxMaxXSpinner.getValueFactory().valueProperty());
      boundingBoxParametersProperty.binBidirectionalMaxY(boundingBoxMaxYSpinner.getValueFactory().valueProperty());
      boundingBoxParametersProperty.binBidirectionalMaxZ(boundingBoxMaxZSpinner.getValueFactory().valueProperty());
      uiMessager.bindBidirectionalGlobal(boundingBoxParametersTopic, boundingBoxParametersProperty);

      load();
      uiMessager.bindBidirectionalInternal(boundingBoxShowTopic, showBoundingBoxButton.selectedProperty(), true);
   }

   @FXML
   public void save()
   {
      uiMessager.submitStateRequestToModule(saveParameterConfigurationTopic);
      saveUIControlProperty(boundingBoxShowTopic, showBoundingBoxButton);
   }

   public void load()
   {
      loadUIControlProperty(boundingBoxShowTopic, showBoundingBoxButton);
   }

   private DoubleSpinnerValueFactory createBoundingBoxValueFactory(double initialValue)
   {
      double min = -100.0;
      double max = 100.0;
      double amountToStepBy = 0.1;
      return new DoubleSpinnerValueFactory(min, max, initialValue, amountToStepBy);
   }
}
