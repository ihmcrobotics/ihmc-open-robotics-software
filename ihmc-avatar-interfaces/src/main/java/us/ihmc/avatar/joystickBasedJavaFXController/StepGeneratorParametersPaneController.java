package us.ihmc.avatar.joystickBasedJavaFXController;

import static us.ihmc.avatar.joystickBasedJavaFXController.StepGeneratorJavaFXTopics.WalkingTrajectoryDuration;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Scanner;
import java.util.regex.Pattern;

import org.ojalgo.netio.BufferedInputStreamReader;

import javafx.beans.value.ChangeListener;
import javafx.beans.value.ObservableValue;
import javafx.collections.FXCollections;
import javafx.fxml.FXML;
import javafx.scene.control.ListView;
import javafx.scene.control.Slider;
import javafx.scene.control.Spinner;
import javafx.scene.control.SpinnerValueFactory.DoubleSpinnerValueFactory;
import javafx.scene.image.Image;
import javafx.scene.image.ImageView;
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

   private List<String> userProfileNames;
   private File workingDirectory;

   public StepGeneratorParametersPaneController()
   {
   }

   public void initialize(JavaFXMessager messager, WalkingControllerParameters walkingControllerParameters, File workingDirectory) throws IOException
   {
      this.workingDirectory = workingDirectory;
      defaultParameters.set(walkingControllerParameters);
      defaultParameters.setSwingHeight(0.025);
      JoystickStepParameters initialParameters = new JoystickStepParameters(defaultParameters);

      userProfileNames = readUserProfileNames(workingDirectory);
      profileListView.setItems(FXCollections.observableArrayList(userProfileNames));
      profileListView.getSelectionModel().selectedItemProperty().addListener(new ChangeListener<String>()
      {
         @Override
         public void changed(ObservableValue<? extends String> observable, String oldProfileName, String newProfileName)
         {
            JoystickStepParameters parameters = stepParametersProperty.get();
            LogTools.info("Saving into " + oldProfileName + " parameters: " + parameters);
            saveProfileParameter(workingDirectory, oldProfileName, parameters);
            stepParametersProperty.setValue(loadProfileParameters(workingDirectory, newProfileName, defaultParameters));
         }
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

   public void close()
   {
      saveUserProfileNames(workingDirectory, userProfileNames);
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

   private static List<String> readUserProfileNames(File workingDirectory)
   {
      File userProfileNamesFile = new File(workingDirectory, JoystickBasedSteppingMainUI.userProfilesFilename);
      List<String> userProfileNameList = new ArrayList<>();
      Scanner scanner;
      try
      {
         scanner = new Scanner(userProfileNamesFile);
      }
      catch (FileNotFoundException e)
      {
         LogTools.error("User profiles file not found");
         return userProfileNameList;
      }

      scanner.useDelimiter(Pattern.compile(","));

      while (scanner.hasNext())
      {
         String next = scanner.next();
         next = next.replaceAll(" ", "").replaceAll("\n", "").replaceAll("\r", "");
         userProfileNameList.add(next);
      }

      scanner.close();
      return userProfileNameList;
   }

   private static void saveUserProfileNames(File workingDirectory, List<String> userProfileNames)
   {
      File userProfileNamesFile = new File(workingDirectory, JoystickBasedSteppingMainUI.userProfilesFilename);
      FileWriter fileWriter = null;

      try
      {
         fileWriter = new FileWriter(userProfileNamesFile);
         for (int i = 0; i < userProfileNames.size(); i++)
         {
            if (i > 0)
               fileWriter.write(", ");
            String userProfileName = userProfileNames.get(i);
            fileWriter.write(userProfileName);
         }
      }
      catch (IOException e)
      {
         throw new RuntimeException("Encountered problem saving profile names.", e);
      }
      finally
      {
         try
         {
            if (fileWriter != null)
               fileWriter.close();
         }
         catch (IOException e)
         {
            throw new RuntimeException("Encountered problem saving profile names.", e);
         }
      }
   }

   private static JoystickStepParameters loadProfileParameters(File workingDirectory, String userProfileName, JoystickStepParameters defaultParameters)
   {
      if (userProfileName == null)
         return defaultParameters;

      JoystickStepParameters loadedParameters;

      File userParameterFile = createParameterFile(workingDirectory, userProfileName);
      if (!userParameterFile.exists())
      {
         loadedParameters = new JoystickStepParameters(defaultParameters);
         saveProfileParameter(workingDirectory, userProfileName, loadedParameters);
      }
      else
      {
         try
         {
            BufferedInputStreamReader reader = new BufferedInputStreamReader(new FileInputStream(userParameterFile));
            loadedParameters = JoystickStepParameters.parse(reader.readLine());
            reader.close();
         }
         catch (FileNotFoundException e)
         {
            throw new RuntimeException("Should not get there as the file existence has been verified already.", e);
         }
         catch (IOException e)
         {
            throw new RuntimeException("Encountered problem loading " + userProfileName, e);
         }
      }
      return loadedParameters;
   }

   private static void saveProfileParameter(File workingDirectory, String userProfileName, JoystickStepParameters parameters)
   {
      if (userProfileName == null)
         return;

      File userParameterFile = createParameterFile(workingDirectory, userProfileName);
      if (!userParameterFile.exists())
      {
         try
         {
            userParameterFile.createNewFile();
         }
         catch (IOException e)
         {
            throw new RuntimeException("Encountered problem saving " + userProfileName, e);
         }
      }

      FileWriter fileWriter = null;
      try
      {
         fileWriter = new FileWriter(userParameterFile);
         fileWriter.write(parameters.toString());
      }
      catch (IOException e)
      {
         throw new RuntimeException("Encountered problem saving " + userProfileName, e);
      } finally
      {
         if (fileWriter != null)
         {
            try
            {
               fileWriter.close();
            }
            catch (IOException e)
            {
               throw new RuntimeException("Encountered problem saving " + userProfileName, e);
            }
         }
      }
   }

   private static File createParameterFile(File workingDirectory, String userProfileName)
   {
      return new File(workingDirectory, userProfileName + ".txt");
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
