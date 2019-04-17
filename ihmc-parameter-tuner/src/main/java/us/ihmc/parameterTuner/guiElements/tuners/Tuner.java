package us.ihmc.parameterTuner.guiElements.tuners;

import org.apache.commons.lang3.StringUtils;

import javafx.application.Platform;
import javafx.geometry.Pos;
import javafx.scene.Node;
import javafx.scene.control.ContextMenu;
import javafx.scene.control.Label;
import javafx.scene.control.TextField;
import javafx.scene.control.TitledPane;
import javafx.scene.control.Tooltip;
import javafx.scene.layout.HBox;
import javafx.scene.layout.Priority;
import javafx.scene.layout.Region;
import javafx.scene.layout.VBox;
import javafx.scene.text.Text;
import us.ihmc.javaFXToolkit.TextFormatterTools;
import us.ihmc.parameterTuner.JavaFXTitledPaneTools;
import us.ihmc.parameterTuner.ParameterTuningTools;
import us.ihmc.parameterTuner.guiElements.GuiElement;
import us.ihmc.parameterTuner.guiElements.GuiParameter;
import us.ihmc.parameterTuner.guiElements.ParameterChangeListener;
import us.ihmc.robotics.sliderboard.SliderboardListener;

public class Tuner extends VBox implements SliderboardListener
{
   private static final int MAX_DESCRIPTION_CHARACTERS = 255;

   static
   {
      JavaFXTitledPaneTools.setTitledPaneAnimationTime(50);
   }

   private final Label name = new Label();
   private final TitledPane descriptionPane = new TitledPane();
   private final TextField description = new TextField();
   private final InputNode inputNode;

   private final GuiParameter parameter;

   public Tuner(GuiParameter parameter)
   {
      this.parameter = parameter;

      name.setText(parameter.getName());
      description.setText(parameter.getCurrentDescription());
      description.setTextFormatter(TextFormatterTools.maxLengthTextFormatter(MAX_DESCRIPTION_CHARACTERS));
      ParameterTuningTools.addThreadSafeListeners(description, () -> parameter.setDescription(description.getText()));

      parameter.addChangedListener(p -> {
         // This listener will be triggered by an external change and is called from the animation timer.
         description.setText(parameter.getCurrentDescription());
      });

      switch (parameter.getType())
      {
      case "DoubleParameter":
         inputNode = new DoubleTuner(parameter);
         break;
      case "IntegerParameter":
         inputNode = new IntegerTuner(parameter);
         break;
      case "BooleanParameter":
         inputNode = new BooleanTuner(parameter);
         break;
      case "EnumParameter":
         inputNode = new EnumTuner(parameter);
         break;
      default:
         throw new RuntimeException("Implement tuner type: " + parameter.getType());
      }

      setupNode(parameter);
   }

   public ContextMenu getContextMenu()
   {
      return descriptionPane.getContextMenu();
   }

   private void setupNode(GuiParameter parameter)
   {
      Tooltip tooltip = new Tooltip(StringUtils.replaceAll(parameter.getUniqueName(), GuiElement.SEPERATOR, "\n"));
      Tooltip.install(descriptionPane, tooltip);
      ContextMenu contextMenu = new ContextMenu();
      descriptionPane.setContextMenu(contextMenu);

      HBox.setHgrow(this, Priority.ALWAYS);
      HBox.setHgrow(name, Priority.ALWAYS);
      HBox.setHgrow(description, Priority.ALWAYS);

      HBox parameterDescriptionBox = new HBox();
      parameterDescriptionBox.setSpacing(10.0);
      parameterDescriptionBox.setAlignment(Pos.CENTER_LEFT);
      parameterDescriptionBox.getChildren().add(new Text("Description"));
      parameterDescriptionBox.getChildren().add(description);
      HBox.setHgrow(parameterDescriptionBox, Priority.ALWAYS);

      VBox extendedOptionsBox = new VBox();
      extendedOptionsBox.setSpacing(10.0);
      extendedOptionsBox.getChildren().add(parameterDescriptionBox);
      extendedOptionsBox.getChildren().add(inputNode.getFullInputNode());

      HBox dropdownGraphic = new HBox();
      dropdownGraphic.setSpacing(10.0);
      dropdownGraphic.setAlignment(Pos.CENTER_LEFT);
      dropdownGraphic.getChildren().add(name);
      Region spacer = new Region();
      HBox.setHgrow(spacer, Priority.ALWAYS);
      dropdownGraphic.getChildren().add(spacer);
      dropdownGraphic.getChildren().add(inputNode.getSimpleInputNode(120.0, 20.0));
      dropdownGraphic.minWidthProperty().bind(descriptionPane.widthProperty().subtract(40));

      descriptionPane.setContent(extendedOptionsBox);
      descriptionPane.setAlignment(Pos.CENTER_LEFT);
      descriptionPane.setGraphic(dropdownGraphic);
      descriptionPane.setExpanded(false);
      getChildren().add(descriptionPane);

      setId("tuner-window");
      name.setId("parameter-name-in-tuner");
   }

   public Node getSimpleInputNode()
   {
      return inputNode.getSimpleInputNode(100.0, 20.0);
   }

   @Override
   public void sliderMoved(double sliderPercentage)
   {
      Platform.runLater(() -> inputNode.setValueFromPercent(sliderPercentage));
   }

   public void addChangeListener(ParameterChangeListener listener)
   {
      parameter.addChangedListener(listener);
   }

   public void removeChangeListener(ParameterChangeListener listener)
   {
      parameter.removeChangeListener(listener);
   }

   public double getValuePercent()
   {
      return inputNode.getValuePercent();
   }

   public GuiParameter getParameter()
   {
      return parameter;
   }
}
