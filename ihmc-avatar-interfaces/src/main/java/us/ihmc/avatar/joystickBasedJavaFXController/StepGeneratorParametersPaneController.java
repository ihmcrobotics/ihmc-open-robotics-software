package us.ihmc.avatar.joystickBasedJavaFXController;

import static us.ihmc.avatar.joystickBasedJavaFXController.StepGeneratorJavaFXTopics.WalkingTrajectoryDuration;

import java.io.IOException;
import java.util.function.Function;

import javafx.beans.value.ChangeListener;
import javafx.collections.FXCollections;
import javafx.fxml.FXML;
import javafx.scene.control.ListView;
import javafx.scene.control.MenuItem;
import javafx.scene.control.Slider;
import javafx.scene.control.Spinner;
import javafx.scene.control.SpinnerValueFactory.DoubleSpinnerValueFactory;
import javafx.scene.control.TextField;
import javafx.scene.image.Image;
import javafx.scene.image.ImageView;
import javafx.scene.input.KeyCode;
import us.ihmc.avatar.joystickBasedJavaFXController.JoystickStepParametersProperty.JoystickStepParameters;
import us.ihmc.avatar.joystickBasedJavaFXController.StepGeneratorJavaFXController.SecondaryControlOption;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.javaFXToolkit.StringConverterTools;
import us.ihmc.javaFXToolkit.messager.JavaFXMessager;
import us.ihmc.javaFXToolkit.messager.MessageBidirectionalBinding.PropertyToMessageTypeConverter;
import us.ihmc.log.LogTools;

public class StepGeneratorParametersPaneController
{
   @FXML
   private TextField newProfileTextField;
   @FXML
   private ListView<String> profileListView;
   @FXML
   private Slider trajectoryDurationSlider;
   @FXML
   private Slider swingHeightSlider;
   @FXML
   private Slider swingDurationSlider;
   @FXML
   private Slider transferDurationSlider;
   @FXML
   private Spinner<Double> maxStepLengthSpinner;
   @FXML
   private Spinner<Double> defaultStepWidthSpinner;
   @FXML
   private Spinner<Double> minStepWidthSpinner;
   @FXML
   private Spinner<Double> maxStepWidthSpinner;
   @FXML
   private Spinner<Double> turnStepWidth;
   @FXML
   private Spinner<Double> turnMaxAngleInwardSpinner;
   @FXML
   private Spinner<Double> turnMaxAngleOutwardSpinner;
   @FXML
   private ImageView controlLayoutImageView;

   private final Image kickImageLayout = new Image(getClass().getResourceAsStream("XBoxOneControllerStepMapping_kick.png"));
   private final Image punchImageLayout = new Image(getClass().getResourceAsStream("XBoxOneControllerStepMapping_punch.png"));

   private final JoystickStepParameters defaultParameters = new JoystickStepParameters();
   private final JoystickStepParametersProperty stepParametersProperty = new JoystickStepParametersProperty(this, "stepParameters");

   private UserProfileManager<JoystickStepParameters> userProfileManager;

   public StepGeneratorParametersPaneController()
   {
   }

   public void initialize(JavaFXMessager messager, WalkingControllerParameters walkingControllerParameters, String workingDirectoryPath) throws IOException
   {
      defaultParameters.set(walkingControllerParameters);
      defaultParameters.setSwingHeight(0.025);
      JoystickStepParameters initialParameters = new JoystickStepParameters(defaultParameters);

      userProfileManager = new UserProfileManager<JoystickStepParameters>(workingDirectoryPath,
                                                                          initialParameters,
                                                                          JoystickStepParameters::parseFromPropertyMap,
                                                                          JoystickStepParameters::exportToPropertyMap);

      profileListView.setItems(FXCollections.observableArrayList(userProfileManager.getUserProfileNames()));
      profileListView.setCellFactory(ListViewTools.cellFactoryForDragAndDropReorder(Function.identity()));
      MenuItem deleteMenuItem = new MenuItem("Delete");
      deleteMenuItem.setOnAction(e -> deleteSelectedProfile());

      profileListView.setCellFactory(ListViewTools.cellFactoryForMouseRightClickContextMenu(profileListView.getCellFactory(), deleteMenuItem));
      profileListView.getSelectionModel().selectedItemProperty().addListener((ChangeListener<String>) (observable, oldProfileName, newProfileName) ->
      {
         JoystickStepParameters currentParameters = stepParametersProperty.get();
         userProfileManager.saveProfile(oldProfileName, currentParameters);
         JoystickStepParameters newParameters = userProfileManager.loadProfile(newProfileName);
         stepParametersProperty.setValue(newParameters);
      });

      if (!profileListView.getItems().isEmpty())
      {
         profileListView.getSelectionModel().select(0);
         initialParameters.set(userProfileManager.loadProfile(profileListView.getItems().get(0)));
      }

      newProfileTextField.setOnKeyPressed(event ->
      {
         if (event.getCode() == KeyCode.ENTER)
            addProfile();
      });

      stepParametersProperty.set(initialParameters);

      swingHeightSlider.setLabelFormatter(StringConverterTools.metersToRoundedCentimeters());
      maxStepLengthSpinner.setValueFactory(new DoubleSpinnerValueFactory(0.0, 1.0, initialParameters.getMaxStepLength(), 0.05));
      defaultStepWidthSpinner.setValueFactory(new DoubleSpinnerValueFactory(0.0, 1.0, initialParameters.getDefaultStepWidth(), 0.05));
      minStepWidthSpinner.setValueFactory(new DoubleSpinnerValueFactory(0.0, 1.0, initialParameters.getMinStepWidth(), 0.025));
      maxStepWidthSpinner.setValueFactory(new DoubleSpinnerValueFactory(0.0, 1.0, initialParameters.getMaxStepWidth(), 0.05));
      turnStepWidth.setValueFactory(new DoubleSpinnerValueFactory(0.0, 1.0, initialParameters.getTurnStepWidth(), 0.05));
      turnMaxAngleInwardSpinner.setValueFactory(newAngleSpinnerValueFactory(0.0,
                                                                            Math.PI / 2.0,
                                                                            initialParameters.getTurnMaxAngleInward(),
                                                                            Math.toRadians(5.0)));
      turnMaxAngleOutwardSpinner.setValueFactory(newAngleSpinnerValueFactory(0.0,
                                                                             Math.PI / 2.0,
                                                                             initialParameters.getTurnMaxAngleOutward(),
                                                                             Math.toRadians(5.0)));

      stepParametersProperty.bindBidirectionalSwingHeight(swingHeightSlider.valueProperty());
      stepParametersProperty.bindBidirectionalSwingDuration(swingDurationSlider.valueProperty());
      stepParametersProperty.bindBidirectionalTransferDuration(transferDurationSlider.valueProperty());
      stepParametersProperty.bindBidirectionalMaxStepLength(maxStepLengthSpinner.getValueFactory().valueProperty());
      stepParametersProperty.bindBidirectionalDefaultStepWidth(defaultStepWidthSpinner.getValueFactory().valueProperty());
      stepParametersProperty.bindBidirectionalMinStepWidth(minStepWidthSpinner.getValueFactory().valueProperty());
      stepParametersProperty.bindBidirectionalMaxStepWidth(maxStepWidthSpinner.getValueFactory().valueProperty());
      stepParametersProperty.bindBidirectionalTurnStepWidth(turnStepWidth.getValueFactory().valueProperty());
      stepParametersProperty.bindBidirectionalTurnMaxAngleInward(turnMaxAngleInwardSpinner.getValueFactory().valueProperty());
      stepParametersProperty.bindBidirectionalTurnMaxAngleOutward(turnMaxAngleOutwardSpinner.getValueFactory().valueProperty());

      messager.bindBidirectional(StepGeneratorJavaFXTopics.SteppingParameters, stepParametersProperty, true);
      messager.bindBidirectional(WalkingTrajectoryDuration, trajectoryDurationSlider.valueProperty(), createConverter(), true);
   }

   @FXML
   private void addProfile()
   {
      String newUserProfileName = newProfileTextField.textProperty().get();

      if (newUserProfileName.isEmpty())
         return;

      profileListView.getItems().add(newUserProfileName);
      profileListView.getSelectionModel().select(newUserProfileName);
      newProfileTextField.clear();
   }

   private void deleteSelectedProfile()
   {
      String profileToDelete = profileListView.getSelectionModel().getSelectedItem();
      profileListView.getItems().remove(profileToDelete);
   }

   public void close()
   {
      userProfileManager.saveProfile(profileListView.getSelectionModel().getSelectedItem(), stepParametersProperty.get());
      userProfileManager.close(profileListView.getItems());
   }

   public void updateImageLayout(SecondaryControlOption option)
   {
      switch (option)
      {
      case KICK:
         controlLayoutImageView.setImage(kickImageLayout);
         break;
      case PUNCH:
         controlLayoutImageView.setImage(punchImageLayout);
         break;
      default:
         LogTools.error("Unhandled option: " + option);
         break;
      }
   }

   private PropertyToMessageTypeConverter<Double, Number> createConverter()
   {
      return new PropertyToMessageTypeConverter<Double, Number>()
      {
         @Override
         public Double convert(Number propertyValue)
         {
            return propertyValue.doubleValue();
         }

         @Override
         public Number interpret(Double messageContent)
         {
            return messageContent;
         }
      };
   }

   public static DoubleSpinnerValueFactory newAngleSpinnerValueFactory(double min, double max, double initialValue, double angleToStepBy)
   {
      DoubleSpinnerValueFactory doubleSpinnerValueFactory = new DoubleSpinnerValueFactory(min, max, initialValue, angleToStepBy);
      doubleSpinnerValueFactory.setConverter(StringConverterTools.radiansToRoundedDegrees());
      return doubleSpinnerValueFactory;
   }
}
