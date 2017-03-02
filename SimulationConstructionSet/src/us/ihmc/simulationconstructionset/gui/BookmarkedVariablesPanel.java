package us.ihmc.simulationconstructionset.gui;

import java.awt.dnd.DropTarget;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;

import javax.swing.JMenuItem;

import us.ihmc.robotics.dataStructures.variable.YoVariable;
import us.ihmc.robotics.dataStructures.variable.YoVariableList;
import us.ihmc.simulationconstructionset.gui.yoVariableSearch.YoVariableListPanel;
import us.ihmc.simulationconstructionset.gui.yoVariableSearch.YoVariablePanelJPopupMenu;

public class BookmarkedVariablesPanel extends YoVariableListPanel
{
   private static final long serialVersionUID = 1390064658136845739L;
   private BookmarkedVariablesHolder bookmarkedVariablesHolder;

   public BookmarkedVariablesPanel(String name, SelectedVariableHolder holder, BookmarkedVariablesHolder bookmarkedVariablesHolder)
   {
      super(new YoVariableList(name), holder, new YoVariablePanelJPopupMenu(holder));
      this.bookmarkedVariablesHolder = bookmarkedVariablesHolder;
      initBookMarks();
   }

   public BookmarkedVariablesPanel(YoVariableList list, SelectedVariableHolder holder, BookmarkedVariablesHolder bookmarkedVariablesHolder)
   {
      super(list, holder, new YoVariablePanelJPopupMenu(holder));
      this.bookmarkedVariablesHolder = bookmarkedVariablesHolder;
      initBookMarks();
   }

   public BookmarkedVariablesPanel(YoVariableList list, SelectedVariableHolder holder, GraphArrayPanel graphArrayPanel, EntryBoxArrayTabbedPanel entryBoxArrayPanel,
                                   BookmarkedVariablesHolder bookmarkedVariablesHolder, YoVariableExplorerTabbedPane combinedVarPanel)
   {
      super(list, holder, new YoVariablePanelJPopupMenu(graphArrayPanel, entryBoxArrayPanel, holder, combinedVarPanel, bookmarkedVariablesHolder));
      this.bookmarkedVariablesHolder = bookmarkedVariablesHolder;
      initBookMarks();
   }

   public BookmarkedVariablesPanel(String name, SelectedVariableHolder holder, GraphArrayPanel graphArrayPanel, EntryBoxArrayTabbedPanel entryBoxArrayPanel,
                                   BookmarkedVariablesHolder bookmarkedVariablesHolder, YoVariableExplorerTabbedPane combinedVarPanel)
   {
      super(new YoVariableList(name), holder, new YoVariablePanelJPopupMenu(graphArrayPanel, entryBoxArrayPanel, holder, combinedVarPanel, bookmarkedVariablesHolder));
      this.bookmarkedVariablesHolder = bookmarkedVariablesHolder;
      initBookMarks();
   }

   private void initBookMarks()
   {
      this.setDropTarget(new DropTarget(this, new BookmarkedVariablesPanelTargetListener(this)));
      bookmarkedVariablesHolder.addBookmarkedVariableAddedListener(new BookmarkedVariableAddedListener()
      {
         @Override
         public void bookmarkAdded(YoVariable<?> variable)
         {
            addVariable(variable);
         }
      });
      bookmarkedVariablesHolder.addBookmarkedVariableRemovedListener(new BookmarkedVariableRemovedListener()
      {
         @Override
         public void bookmarkRemoved(YoVariable<?> variable)
         {
            removeVariable(variable);
         }
      });
      initPopupMenu();
   }

   private void initPopupMenu()
   {
      if (varPanelJPopupMenu != null)
      {
         varPanelJPopupMenu.removeBookmarkVariable();
         JMenuItem removeVarible = new JMenuItem("Remove Variable");
         removeVarible.addActionListener(new ActionListener()
         {
            @Override
            public void actionPerformed(ActionEvent e)
            {
               if (selectedVariableHolder.getSelectedVariable() != null)
               {
                  removeVariable(selectedVariableHolder.getSelectedVariable());
               }

               varPanelJPopupMenu.setVisible(false);
            }
         });
         varPanelJPopupMenu.add(removeVarible);
      }
      else
      {
         System.err.println("Warning: Bookmarked Variables popupmenu not initialized because it is null.");
      }
   }

   public SelectedVariableHolder getSelectedVariableHolder()
   {
      return selectedVariableHolder;
   }

   public void bookmarkVariable(YoVariable<?> variable)
   {
      bookmarkedVariablesHolder.addBookmark(variable);
   }

}
