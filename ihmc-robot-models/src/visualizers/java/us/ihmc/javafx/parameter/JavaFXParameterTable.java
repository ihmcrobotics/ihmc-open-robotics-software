package us.ihmc.javafx.parameter;

import javafx.collections.FXCollections;
import javafx.collections.ObservableList;
import javafx.scene.control.Control;
import javafx.scene.control.TableColumn;
import javafx.scene.control.TableView;

public class JavaFXParameterTable
{
   private final TableView tableView;
   private final ObservableList<JavaFXParameterTableEntry> parameterTableItems = FXCollections.observableArrayList();

   public JavaFXParameterTable(TableView tableView)
   {
      this.tableView = tableView;

      // first type is the displayed object
      // second type is the object that gets put in the table and that the cellValueFactory interprets

      TableColumn<JavaFXParameterTableEntry, String> nameColumn = new TableColumn<>("Name");
      nameColumn.setCellValueFactory(cellDataFeatures -> ((JavaFXParameterTableEntry) cellDataFeatures.getValue()).getObservableName());

      TableColumn<JavaFXParameterTableEntry, Control> spinnerColumn = new TableColumn<>("Value");
      spinnerColumn.setCellValueFactory(cellDataFeatures -> ((JavaFXParameterTableEntry) cellDataFeatures.getValue()).getObservableValue());

      nameColumn.setSortable(false);
      spinnerColumn.setSortable(false);

      tableView.getColumns().add(nameColumn);
      tableView.getColumns().add(spinnerColumn);
   }

   public void addEntry(JavaFXParameterTableEntry entry)
   {
      parameterTableItems.add(entry);
   }

   public void updateEntries()
   {
      tableView.setItems(parameterTableItems);
   }
}
