package us.ihmc.avatar.joystickBasedJavaFXController;

import java.util.function.Function;

import javafx.beans.property.ObjectProperty;
import javafx.beans.property.SimpleObjectProperty;
import javafx.collections.ObservableList;
import javafx.scene.control.ContextMenu;
import javafx.scene.control.ListCell;
import javafx.scene.control.ListView;
import javafx.scene.control.MenuItem;
import javafx.scene.input.ClipboardContent;
import javafx.scene.input.DragEvent;
import javafx.scene.input.Dragboard;
import javafx.scene.input.MouseButton;
import javafx.scene.input.TransferMode;
import javafx.util.Callback;

public class ListViewTools
{
   public static <T> Callback<ListView<T>, ListCell<T>> cellFactoryForDragAndDropReorder(Function<T, String> textExtractor)
   {
      return cellFactoryForDragAndDropReorder(listView ->
      {
         ListCell<T> cell = new ListCell<T>()
         {
            @Override
            public void updateItem(T item, boolean empty)
            {
               super.updateItem(item, empty);
               setText(textExtractor.apply(item));
            }
         };
         return cell;
      }, textExtractor);
   }

   public static <T> Callback<ListView<T>, ListCell<T>> cellFactoryForDragAndDropReorder(Callback<ListView<T>, ListCell<T>> cellFactory,
                                                                                         Function<T, String> textExtractor)
   {
      ObjectProperty<ListCell<T>> dragSource = new SimpleObjectProperty<>();

      return listView ->
      {
         ListCell<T> cell = cellFactory.call(listView);

         cell.setOnDragDetected(event ->
         {
            if (cell.getItem() == null || cell.isEmpty())
               return;

            Dragboard dragboard = cell.startDragAndDrop(TransferMode.MOVE);
            ClipboardContent clipboardContent = new ClipboardContent();
            clipboardContent.putString(textExtractor.apply(cell.getItem()));
            dragboard.setContent(clipboardContent);
            dragSource.set(cell);
            event.consume();
         });

         cell.setOnDragEntered(event ->
         {
            if (event.getGestureSource() != cell && event.getDragboard().hasString())
            {
               cell.setOpacity(0.3);
            }
         });

         cell.setOnDragExited(event ->
         {
            if (event.getGestureSource() != cell && event.getDragboard().hasString())
               cell.setOpacity(1);
         });

         cell.setOnDragOver(event ->
         {
            if (event.getGestureSource() != cell && event.getDragboard().hasString())
               event.acceptTransferModes(TransferMode.MOVE);
         });

         cell.setOnDragDone(DragEvent::consume);

         cell.setOnDragDropped(event ->
         {
            if (cell.getItem() == null)
               return;

            if (event.getDragboard().hasString() && dragSource.get() != null)
            {
               ObservableList<T> items = listView.getItems();
               int cellIndex = items.indexOf(cell.getItem());

               T draggedItem = dragSource.get().getItem();
               items.remove(draggedItem);
               items.add(cellIndex, draggedItem);
               listView.getSelectionModel().select(draggedItem);
               event.setDropCompleted(true);
               dragSource.set(null);
            }
            else
            {
               event.setDropCompleted(false);
            }
         });

         return cell;
      };
   }

   public static <T> Callback<ListView<T>, ListCell<T>> cellFactoryForMouseRightClickContextMenu(Callback<ListView<T>, ListCell<T>> cellFactory,
                                                                                                 MenuItem... contextMenuItems)
   {
      return listView ->
      {
         ListCell<T> cell = cellFactory.call(listView);

         cell.setOnMousePressed(event ->
         {
            if (event.getButton() == MouseButton.SECONDARY)
               cell.setContextMenu(new ContextMenu(contextMenuItems));
         });

         return cell;
      };
   }
}
