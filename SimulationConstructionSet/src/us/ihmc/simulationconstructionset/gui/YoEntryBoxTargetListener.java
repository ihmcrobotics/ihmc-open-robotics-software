package us.ihmc.simulationconstructionset.gui;

import java.awt.Cursor;
import java.awt.dnd.DropTargetDragEvent;
import java.awt.dnd.DropTargetDropEvent;
import java.awt.dnd.DropTargetEvent;
import java.awt.dnd.DropTargetListener;

import javax.swing.TransferHandler;

import us.ihmc.robotics.dataStructures.variable.YoVariable;

public class YoEntryBoxTargetListener implements DropTargetListener
{
   private YoEntryBox yoEntryBox;
   private static final Cursor
      droppableCursor = Cursor.getPredefinedCursor(Cursor.DEFAULT_CURSOR), notDroppableCursor = Cursor.getPredefinedCursor(Cursor.DEFAULT_CURSOR);

   public YoEntryBoxTargetListener(YoEntryBox yoEntryBox)
   {
      this.yoEntryBox = yoEntryBox;
   }

   public void dragEnter(DropTargetDragEvent dtde)
   {
   }

   public void dragExit(DropTargetEvent dte)
   {
      this.yoEntryBox.setCursor(notDroppableCursor);
   }

   public void dragOver(DropTargetDragEvent dtde)
   {
      YoGraph.setRecipientOfDrag(yoEntryBox);

      if ((YoGraph.getSourceOfDrag() == null) || (!YoGraph.getSourceOfDrag().equals(yoEntryBox) &&!(YoGraph.getSourceOfDrag() instanceof YoEntryBox)))
      {
         if (!this.yoEntryBox.getCursor().equals(droppableCursor))
         {
            this.yoEntryBox.setCursor(droppableCursor);
         }
      }
      else
      {
         dtde.rejectDrag();
      }
   }

   public void drop(DropTargetDropEvent dtde)
   {
      YoGraph.setRecipientOfDrag(yoEntryBox);

      if ((YoGraph.getSourceOfDrag() == null) || (!YoGraph.getSourceOfDrag().equals(yoEntryBox) &&!(YoGraph.getSourceOfDrag() instanceof YoEntryBox)))
      {
         YoVariable v = yoEntryBox.getSelectedVariableHolder().getSelectedVariable();
         if (v != null)
            yoEntryBox.addVariable(v);
      }
      else
      {
         YoGraph.setActionPerformedByDragAndDrop(TransferHandler.NONE);
         dtde.rejectDrop();
      }
   }

   public void dropActionChanged(DropTargetDragEvent dtde)
   {
      YoGraph.setActionPerformedByDragAndDrop(dtde.getDropAction());
   }

}
