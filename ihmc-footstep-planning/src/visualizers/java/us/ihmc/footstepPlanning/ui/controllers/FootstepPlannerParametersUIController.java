package us.ihmc.footstepPlanning.ui.controllers;

import javafx.beans.property.ReadOnlyObjectWrapper;
import javafx.collections.FXCollections;
import javafx.collections.ObservableList;
import javafx.fxml.FXML;
import javafx.geometry.Insets;
import javafx.scene.control.*;
import javafx.scene.layout.HBox;
import javafx.scene.layout.Pane;
import javafx.scene.layout.Priority;
import javafx.scene.layout.Region;
import javafx.scene.shape.Rectangle;
import javafx.util.Callback;
import javafx.util.StringConverter;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParameterKeys;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersBasics;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersReadOnly;
import us.ihmc.javaFXToolkit.messager.JavaFXMessager;
import us.ihmc.robotEnvironmentAwareness.ui.properties.JavaFXStoredPropertyMap;
import us.ihmc.tools.property.*;

import java.text.NumberFormat;
import java.util.*;
import java.util.concurrent.atomic.AtomicBoolean;

public class FootstepPlannerParametersUIController
{
   private static final int numTableColumns = 4;

   private JavaFXMessager messager;
   private FootstepPlannerParametersBasics planningParameters;
   private JavaFXStoredPropertyMap javaFXStoredPropertyMap;
   private final StepShapeManager stepShapeManager = new StepShapeManager();

   @FXML
   private Rectangle stepShape;
   @FXML
   private Rectangle stanceFootShape;
   @FXML
   private Rectangle swingFootShape;
   @FXML
   private Rectangle clearanceBox;
   private static final double footWidth = 0.15;
   private static final double footLength = 0.25;
   private static final double leftFootOriginX = 30;
   private static final double leftFootOriginY = 100;
   private static final double metersToPixel = 200;

   @FXML
   private TableView<ParametersTableRow> parameterTable;
   private final ObservableList<ParametersTableRow> parameterTableRows = FXCollections.observableArrayList();

   public FootstepPlannerParametersUIController()
   {
   }

   public void attachMessager(JavaFXMessager messager)
   {
      this.messager = messager;
   }

   public void setPlannerParameters(FootstepPlannerParametersBasics parameters)
   {
      this.planningParameters = parameters;
      this.javaFXStoredPropertyMap = new JavaFXStoredPropertyMap(planningParameters);
      stepShapeManager.update();
   }

   public void bindControls()
   {
      parameterTable.setPadding(new Insets(10.0));

      StoredPropertyKeyList keys = FootstepPlannerParameterKeys.keys;
      int numRows = keys.keys().size() / numTableColumns + 1;

      List<StoredPropertyKey<?>> orderedKeys = new ArrayList<>(keys.keys());
      Comparator<StoredPropertyKey<?>> nameSorter = Comparator.comparing(StoredPropertyKey::getTitleCasedName, String::compareTo);
      orderedKeys.sort(nameSorter);
      parameterTableRows.clear();

      // TableView sometimes doesn't show final row, add extra at the bottom...
      int additionalRows = 5;

      for (int row = 0; row < numRows + additionalRows; row++)
      {
         ParametersTableRow tableRow = new ParametersTableRow();
         for (int col = 0; col < numTableColumns; col++)
         {
            int index = numTableColumns * row + col;
            if (index < orderedKeys.size())
            {
               tableRow.parameters.add(new ParameterTableCell(orderedKeys.get(index), index == orderedKeys.size() - 1));
            }
            else
            {
               tableRow.parameters.add(new ParameterTableCell());
            }
         }
         parameterTableRows.add(tableRow);
      }

      parameterTable.setItems(parameterTableRows);

      for (int i = 0; i < numTableColumns; i++)
      {
         final int columnIndex = i;

         TableColumn<ParametersTableRow, ParameterTableCell> column = new TableColumn<>();
         column.setPrefWidth(ParameterTableCell.width);
         column.setCellFactory(new ParameterCellFactory());
         column.setCellValueFactory(param -> new ReadOnlyObjectWrapper<>(param.getValue().parameters.get(columnIndex)));
         parameterTable.getColumns().add(column);
      }

      parameterTable.refresh();

      // set messager updates to update all stored properties and select JavaFX properties
      messager.registerTopicListener(FootstepPlannerMessagerAPI.PlannerParameters, parameters ->
      {
         planningParameters.set(parameters);
         javaFXStoredPropertyMap.copyStoredToJavaFX();
         stepShapeManager.update();
      });

      // these dimensions work best for valkyrie
      stanceFootShape.setHeight(footLength * metersToPixel);
      stanceFootShape.setWidth(footWidth * metersToPixel);
      stanceFootShape.setLayoutX(leftFootOriginX);
      stanceFootShape.setLayoutY(leftFootOriginY);

      swingFootShape.setHeight(footLength * metersToPixel);
      swingFootShape.setWidth(footWidth * metersToPixel);
      swingFootShape.setLayoutY(leftFootOriginY);
   }

   public void setup()
   {
      Pane header = (Pane) parameterTable.lookup("TableHeaderRow");
      if (header.isVisible())
      {
         header.setMaxHeight(0);
         header.setMinHeight(0);
         header.setPrefHeight(0);
         header.setVisible(false);
      }

      parameterTable.setSelectionModel(null);
   }

   private void publishParameters()
   {
      messager.submitMessage(FootstepPlannerMessagerAPI.PlannerParameters, planningParameters);
   }

   @FXML
   public void saveToFile()
   {
      planningParameters.save();
   }

   private class StepShapeManager
   {
      double minStepYaw, maxStepYaw, minStepWidth, maxStepWidth, minStepLength, maxStepLength, worstYaw;

      void update()
      {
         double minStepYaw = planningParameters.getMinimumStepYaw();
         double maxStepYaw = planningParameters.getMaximumStepYaw();
         double minStepWidth = planningParameters.getMinimumStepWidth();
         double maxStepWidth = planningParameters.getMaximumStepWidth();
         double minStepLength = planningParameters.getMinimumStepLength();
         double maxStepLength = planningParameters.getMaximumStepReach();
         double worstYaw = maxStepYaw > Math.abs(minStepYaw) ? maxStepYaw : minStepYaw;

         if (EuclidCoreTools.epsilonEquals(minStepYaw, this.minStepYaw, 1e-3) &&
             EuclidCoreTools.epsilonEquals(maxStepYaw, this.maxStepYaw, 1e-3) &&
             EuclidCoreTools.epsilonEquals(minStepWidth, this.minStepWidth, 1e-3) &&
             EuclidCoreTools.epsilonEquals(maxStepWidth, this.maxStepWidth, 1e-3) &&
             EuclidCoreTools.epsilonEquals(minStepLength, this.minStepLength, 1e-3) &&
             EuclidCoreTools.epsilonEquals(maxStepLength, this.maxStepLength, 1e-3) &&
             EuclidCoreTools.epsilonEquals(worstYaw, this.worstYaw, 1e-3))
            return;

         this.minStepYaw = minStepYaw;
         this.maxStepYaw = maxStepYaw;
         this.minStepWidth = minStepWidth;
         this.maxStepWidth = maxStepWidth;
         this.minStepLength = minStepLength;
         this.maxStepLength = maxStepLength;
         this.worstYaw = worstYaw;
         setStepShape();
      }

      void setStepShape()
      {
         double minXClearance = planningParameters.getMinXClearanceFromStance();
         double minYClearance = planningParameters.getMinYClearanceFromStance();

         double footCenterX = leftFootOriginX + 0.5 * footWidth * metersToPixel;
         double footCenterY = leftFootOriginY + 0.5 * footLength * metersToPixel;

         double furthestIn = Math.max(minStepWidth, minYClearance) * metersToPixel;

         double width = maxStepWidth - minStepWidth;
         double height = maxStepLength - minStepLength;
         double xCenterInPanel = footCenterX + metersToPixel * minStepWidth;
         double yCenterInPanel = footCenterY - metersToPixel * maxStepLength;

         stepShape.setLayoutX(xCenterInPanel);
         stepShape.setWidth(metersToPixel * width);
         stepShape.setLayoutY(yCenterInPanel);
         stepShape.setHeight(metersToPixel * height);

         swingFootShape.setLayoutX(footCenterX + furthestIn - 0.5 * footWidth * metersToPixel);
         swingFootShape.setRotate(Math.toDegrees(worstYaw));

         clearanceBox.setLayoutX(leftFootOriginX + (0.5 * footWidth - minYClearance) * metersToPixel);
         clearanceBox.setLayoutY(leftFootOriginY + (0.5 * footLength - minXClearance) * metersToPixel);
         clearanceBox.setWidth(metersToPixel * (minYClearance * 2.0));
         clearanceBox.setHeight(metersToPixel * (minXClearance * 2.0));
      }
   }

   public class ParametersTableRow
   {
      private final List<ParameterTableCell> parameters = new ArrayList<>();
   }

   public class ParameterTableCell
   {
      private static final double width = 380.0;
      private static final double spinnerWidth = 120.0;
      private static final double labelWidth = width - spinnerWidth;

      private final StoredPropertyKey<?> parameter;

      // for empty cell
      public ParameterTableCell()
      {
         parameter = null;
      }

      public ParameterTableCell(StoredPropertyKey<?> parameter, boolean finalCell)
      {
         this.parameter = parameter;
      }

      boolean isEmpty()
      {
         return parameter == null;
      }
   }

   public class ParameterCellFactory implements Callback<TableColumn<ParametersTableRow, ParameterTableCell>, TableCell<ParametersTableRow, ParameterTableCell>>
   {
      @Override
      public TableCell<ParametersTableRow, ParameterTableCell> call(TableColumn<ParametersTableRow, ParameterTableCell> param)
      {
         return new TableCell<ParametersTableRow, ParameterTableCell>()
         {
            final HBox hBox = new HBox();
            final Label label = new Label();

            @Override
            protected void updateItem(ParameterTableCell tableCell, boolean empty)
            {
               super.updateItem(tableCell, empty);
               hBox.getChildren().clear();

               if (tableCell != null && !tableCell.isEmpty())
               {
                  StoredPropertyKey<?> propertyKey = tableCell.parameter;

                  hBox.getChildren().add(label);

                  label.setPrefWidth(ParameterTableCell.labelWidth);
                  label.textProperty().setValue(propertyKey.getCamelCasedName());

                  Region region = new Region();
                  HBox.setHgrow(region, Priority.ALWAYS);
                  hBox.getChildren().add(region);

                  Control spinner = createEditor(propertyKey);
                  spinner.setPrefWidth(ParameterTableCell.spinnerWidth);
                  javaFXStoredPropertyMap.bindStoredToJavaFXUserInput(propertyKey);
                  javaFXStoredPropertyMap.bindToJavaFXUserInput(propertyKey, FootstepPlannerParametersUIController.this::publishParameters);

                  hBox.getChildren().add(spinner);
               }

               setGraphic(hBox);
            }

            private Control createEditor(StoredPropertyKey<?> propertyKey)
            {
               if (propertyKey instanceof BooleanStoredPropertyKey)
               {
                  CheckBox checkBox = new CheckBox("");
                  javaFXStoredPropertyMap.put(checkBox, (BooleanStoredPropertyKey) propertyKey);
                  return checkBox;
               }
               else if (propertyKey instanceof DoubleStoredPropertyKey)
               {
                  Spinner<Double> spinner = new Spinner<>(-Double.MAX_VALUE, Double.MAX_VALUE, 0.0, 0.1);
                  spinner.setEditable(true);
                  spinner.getEditor().setTextFormatter(new TextFormatter<>(new DoubleStringConverter()));
                  spinner.getValueFactory().setConverter(new DoubleStringConverter());
                  javaFXStoredPropertyMap.put(spinner, (DoubleStoredPropertyKey) propertyKey);
                  return spinner;
               }
               else if (propertyKey instanceof IntegerStoredPropertyKey)
               {
                  Spinner<Integer> spinner = new Spinner<>(-Integer.MAX_VALUE, Integer.MAX_VALUE, 0, 1);
                  spinner.setEditable(true);
                  javaFXStoredPropertyMap.put(spinner, (IntegerStoredPropertyKey) propertyKey);
                  return spinner;
               }
               else
               {
                  throw new RuntimeException("Unknown parameter property: " + propertyKey.getClass());
               }
            }
         };
      }
   }

   private static final NumberFormat numberFormat = NumberFormat.getInstance();
   static
   {
      numberFormat.setMinimumIntegerDigits(1);
      numberFormat.setMinimumFractionDigits(3);
      numberFormat.setMaximumFractionDigits(8);
   }

   private static class DoubleStringConverter extends StringConverter<Double>
   {
      @Override
      public String toString(Double object)
      {
         if (object == null)
            return numberFormat.format(0.0);
         else if (!Double.isFinite(object))
            return Double.toString(object);
         else
            return numberFormat.format(object);
      }

      @Override
      public Double fromString(String string)
      {
         return Double.parseDouble(string);
      }
   }
}
