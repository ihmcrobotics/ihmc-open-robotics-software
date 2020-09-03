package us.ihmc.javafx.parameter;

import javafx.beans.property.ReadOnlyObjectWrapper;
import javafx.collections.FXCollections;
import javafx.collections.ObservableList;
import javafx.geometry.Insets;
import javafx.geometry.Pos;
import javafx.scene.control.*;
import javafx.scene.layout.HBox;
import javafx.scene.layout.Pane;
import javafx.scene.layout.Priority;
import javafx.scene.layout.Region;
import javafx.util.Callback;
import javafx.util.StringConverter;
import us.ihmc.tools.property.*;

import javax.swing.*;
import java.io.File;
import java.nio.file.Path;
import java.text.NumberFormat;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashMap;
import java.util.List;

public class StoredPropertyTableViewWrapper
{
   // total cell width
   private final double cellWidth;
   // amount of cell width for the parameter's label
   private final double cellLabelWidth;

   private final int tableColumns;

   private final TableView<ParametersTableRow> parameterTable;
   private final HashMap<StoredPropertyKey, Control> controlMap = new HashMap<>();
   private final ObservableList<ParametersTableRow> parameterTableRows = FXCollections.observableArrayList();
   private final JavaFXStoredPropertyMap javaFXStoredPropertyMap;

   public StoredPropertyTableViewWrapper(double cellWidth,
                                         double cellLabelWidth,
                                         int tableColumns,
                                         TableView<ParametersTableRow> parameterTable,
                                         JavaFXStoredPropertyMap javaFXStoredPropertyMap)
   {
      this.cellWidth = cellWidth;
      this.cellLabelWidth = cellLabelWidth;
      this.tableColumns = tableColumns;
      this.parameterTable = parameterTable;
      this.javaFXStoredPropertyMap = javaFXStoredPropertyMap;

      setup();
   }

   private void setup()
   {
      parameterTable.setPadding(new Insets(10.0));

      StoredPropertyKeyListReadOnly keys = javaFXStoredPropertyMap.getStoredPropertySet().getKeyList();
      int numRows = keys.keys().size() / tableColumns + 1;

      List<StoredPropertyKey<?>> orderedKeys = new ArrayList<>(keys.keys());
      Comparator<StoredPropertyKey<?>> nameSorter = Comparator.comparing(StoredPropertyKey::getTitleCasedName, String.CASE_INSENSITIVE_ORDER);
      orderedKeys.sort(nameSorter);

      // create all the controls up front
      for (StoredPropertyKey<?> propertyKey : orderedKeys)
      {
         controlMap.put(propertyKey, createEditor(propertyKey));
      }
      javaFXStoredPropertyMap.bindStoredToJavaFXUserInput();

      parameterTableRows.clear();

      // TableView sometimes doesn't show final row, add extra at the bottom...
      // make a parameter if this is too much, but for 2 rows it would still sometimes not render the bottom
      int additionalRows = 5;

      for (int row = 0; row < numRows + additionalRows; row++)
      {
         ParametersTableRow tableRow = new ParametersTableRow();
         for (int col = 0; col < tableColumns; col++)
         {
            int index = tableColumns * row + col;
            if (index < orderedKeys.size())
            {
               tableRow.parameters.add(new ParameterTableCell(orderedKeys.get(index)));
            }
            else
            {
               tableRow.parameters.add(new ParameterTableCell());
            }
         }
         parameterTableRows.add(tableRow);
      }

      parameterTable.setItems(parameterTableRows);

      for (int i = 0; i < tableColumns; i++)
      {
         final int columnIndex = i;

         TableColumn<ParametersTableRow, ParameterTableCell> column = new TableColumn<>();
         column.setPrefWidth(cellWidth);
         column.setCellFactory(new ParameterCellFactory());
         column.setCellValueFactory(param -> new ReadOnlyObjectWrapper<>(param.getValue().parameters.get(columnIndex)));
         parameterTable.getColumns().add(column);
      }

      parameterTable.refresh();

   }

   public void setTableUpdatedCallback(Runnable tableUpdateCallback)
   {
      javaFXStoredPropertyMap.addAnyJavaFXValueChangedListener(tableUpdateCallback);
   }

   /**
    * Should be called after table is loaded, after Stage.show() is called on primary stage
     */
   public void removeHeader()
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

   /**
    * Loads new file. Subsequent saves will write to the selected file
    */
   public void loadNewFile()
   {
      JFileChooser fileChooser = new JFileChooser();
      Path saveFileDirectory = javaFXStoredPropertyMap.getStoredPropertySet().findSaveFileDirectory();
      fileChooser.setCurrentDirectory(saveFileDirectory.toFile());
      int chooserState = fileChooser.showOpenDialog(null);

      if (chooserState == JFileChooser.APPROVE_OPTION)
      {
         File selectedFile = fileChooser.getSelectedFile();
         String fileName = selectedFile.getName();
         javaFXStoredPropertyMap.getStoredPropertySet().load(fileName);
      }
   }

   public class ParametersTableRow
   {
      private final List<ParameterTableCell> parameters = new ArrayList<>();
   }

   public class ParameterTableCell
   {
      private final StoredPropertyKey<?> parameter;

      // for empty cell
      public ParameterTableCell()
      {
         parameter = null;
      }

      public ParameterTableCell(StoredPropertyKey<?> parameter)
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

                  label.setPrefWidth(cellLabelWidth);
                  label.textProperty().setValue(propertyKey.getTitleCasedName());

                  Region region = new Region();
                  HBox.setHgrow(region, Priority.ALWAYS);
                  hBox.getChildren().add(region);

                  Control spinner = controlMap.get(propertyKey);
                  spinner.setPrefWidth(cellWidth - cellLabelWidth);

                  hBox.getChildren().add(spinner);
               }

               setGraphic(hBox);
            }
         };
      }
   }

   private Control createEditor(StoredPropertyKey<?> propertyKey)
   {
      Control control;
      if (propertyKey instanceof BooleanStoredPropertyKey)
      {
         CheckBox checkBox = new CheckBox("");
         javaFXStoredPropertyMap.put(checkBox, (BooleanStoredPropertyKey) propertyKey);
         checkBox.setAlignment(Pos.CENTER);
         control = checkBox;
      }
      else if (propertyKey instanceof DoubleStoredPropertyKey)
      {
         Spinner<Double> spinner = new Spinner<>(-Double.MAX_VALUE, Double.MAX_VALUE, 0.0, 0.1);
         spinner.setEditable(true);
         spinner.getEditor().setTextFormatter(new TextFormatter<>(new DoubleStringConverter()));
         spinner.getValueFactory().setConverter(new DoubleStringConverter());
         javaFXStoredPropertyMap.put(spinner, (DoubleStoredPropertyKey) propertyKey);
         control = spinner;
      }
      else if (propertyKey instanceof IntegerStoredPropertyKey)
      {
         Spinner<Integer> spinner = new Spinner<>(Integer.MIN_VALUE, Integer.MAX_VALUE, 0, 1);
         spinner.setEditable(true);
         javaFXStoredPropertyMap.put(spinner, (IntegerStoredPropertyKey) propertyKey);
         control = spinner;
      }
      else
      {
         throw new RuntimeException("Unknown parameter property: " + propertyKey.getClass());
      }

      return control;
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
