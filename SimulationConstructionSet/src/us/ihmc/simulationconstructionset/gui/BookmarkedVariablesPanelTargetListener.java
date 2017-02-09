package us.ihmc.simulationconstructionset.gui;

import java.awt.Cursor;
import java.awt.dnd.DropTargetDragEvent;
import java.awt.dnd.DropTargetDropEvent;
import java.awt.dnd.DropTargetEvent;
import java.awt.dnd.DropTargetListener;

import javax.swing.TransferHandler;

import us.ihmc.robotics.dataStructures.variable.YoVariable;

public class BookmarkedVariablesPanelTargetListener implements DropTargetListener
{
   private BookmarkedVariablesPanel bookmarkedVariablesPanel;
   private static final Cursor
      droppableCursor = Cursor.getPredefinedCursor(Cursor.DEFAULT_CURSOR), notDroppableCursor = Cursor.getPredefinedCursor(Cursor.DEFAULT_CURSOR);

   public BookmarkedVariablesPanelTargetListener(BookmarkedVariablesPanel bookmarkedVariablesPanel)
   {
      this.bookmarkedVariablesPanel = bookmarkedVariablesPanel;
   }

   @Override
   public void dragEnter(DropTargetDragEvent dtde)
   {
   }

   @Override
   public void dragExit(DropTargetEvent dte)
   {
      this.bookmarkedVariablesPanel.setCursor(notDroppableCursor);
   }

   @Override
   public void dragOver(DropTargetDragEvent dtde)
   {
      YoGraph.setRecipientOfDrag(bookmarkedVariablesPanel);

      if (!this.bookmarkedVariablesPanel.getCursor().equals(droppableCursor)
              && ((YoGraph.getSourceOfDrag() == null) ||!YoGraph.getSourceOfDrag().equals(bookmarkedVariablesPanel)))
      {
         this.bookmarkedVariablesPanel.setCursor(droppableCursor);
      }
      else if ( /* !this.yoGraph.getCursor().equals(notDroppableCursor) && */dtde.getSource().equals(bookmarkedVariablesPanel))
      {
//       this.yoGraph.setCursor(notDroppableCursor);
         dtde.rejectDrag();
      }
   }

   @Override
   public void drop(DropTargetDropEvent dtde)
   {
      YoGraph.setRecipientOfDrag(bookmarkedVariablesPanel);

      if ((YoGraph.getSourceOfDrag() == null) ||!YoGraph.getSourceOfDrag().equals(bookmarkedVariablesPanel))
      {
         YoVariable<?> v = bookmarkedVariablesPanel.getSelectedVariableHolder().getSelectedVariable();
         if (v != null)
            bookmarkedVariablesPanel.bookmarkVariable(v);
      }
      else
      {
         YoGraph.setActionPerformedByDragAndDrop(TransferHandler.NONE);
         dtde.rejectDrop();
      }
   }

   @Override
   public void dropActionChanged(DropTargetDragEvent dtde)
   {
      YoGraph.setActionPerformedByDragAndDrop(dtde.getDropAction());
   }
}
