package us.ihmc.simulationconstructionset.gui;

import java.awt.Toolkit;
import java.awt.datatransfer.Clipboard;
import java.awt.datatransfer.StringSelection;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;

import javax.swing.JFrame;
import javax.swing.JMenuItem;
import javax.swing.JPopupMenu;

public class VarPanelJPopupMenu extends JPopupMenu
{
   private static final long serialVersionUID = -1080363015468056576L;

   private final GraphArrayPanel graphArrayPanel;
   private final EntryBoxArrayTabbedPanel entryBoxArrayPanel;
   private final SelectedVariableHolder selectedVariableHolder;
   private final CombinedVarPanel combinedVarPanel;
   private final BookmarkedVariablesHolder bookmarkedVariablesHolder;

   private JMenuItem bookmarkVariable;
   //   private JMenuItem showNameSpaces;

   private JMenuItem addToSliderBoard;

   public VarPanelJPopupMenu(SelectedVariableHolder selectedVariableHolder)
   {
      super();
      this.selectedVariableHolder = selectedVariableHolder;

      this.graphArrayPanel = null;
      this.entryBoxArrayPanel = null;
      this.combinedVarPanel = null;
      this.bookmarkedVariablesHolder = null;
   }

   public VarPanelJPopupMenu(GraphArrayPanel graphArrayPanel, EntryBoxArrayTabbedPanel entryBoxArrayPanel, SelectedVariableHolder selectedVariableHolder,
         CombinedVarPanel combinedVarPanel, BookmarkedVariablesHolder bookmarkedVariablesHolder)
   {
      super();
      this.graphArrayPanel = graphArrayPanel;
      this.entryBoxArrayPanel = entryBoxArrayPanel;
      this.selectedVariableHolder = selectedVariableHolder;
      this.combinedVarPanel = combinedVarPanel;
      this.bookmarkedVariablesHolder = bookmarkedVariablesHolder;

      initialize();
   }

   private void initialize()
   {
      JMenuItem addToNewGraph = new JMenuItem("Add Variable to New Graph");
      addToNewGraph.addActionListener(new ActionListener()
      {
         public void actionPerformed(ActionEvent e)
         {
            if (graphArrayPanel != null)
            {
               graphArrayPanel.addSelectedVariableGraph();
            }
            else
            {
               System.err.println("Warning: Could not add variable to a new graph because graphArrayPanel was null.");
            }

            setVisible(false);
         }
      });

      this.add(addToNewGraph);
      JMenuItem addToNewEntryBox = new JMenuItem("Add Variable to new Entry Box");
      addToNewEntryBox.addActionListener(new ActionListener()
      {
         public void actionPerformed(ActionEvent e)
         {
            if (entryBoxArrayPanel != null)
            {
               if (selectedVariableHolder.getSelectedVariable() != null)
               {
                  entryBoxArrayPanel.addEntryBox(selectedVariableHolder.getSelectedVariable());
                  entryBoxArrayPanel.updateUI();
               }
            }
            else
            {
               System.err.println("Warning: Could not add variable to a new entry box because entryBoxArrayPanel was null.");
            }

            setVisible(false);
         }
      });

      this.add(addToNewEntryBox);
      JMenuItem copyToClipBoard = new JMenuItem("Copy Name to Clipboard");
      copyToClipBoard.addActionListener(new ActionListener()
      {
         public void actionPerformed(ActionEvent e)
         {
            if (selectedVariableHolder.getSelectedVariable() != null)
            {
               StringSelection stringSelection = new StringSelection(selectedVariableHolder.getSelectedVariable().getName());
               Clipboard clipboard = Toolkit.getDefaultToolkit().getSystemClipboard();
               clipboard.setContents(stringSelection, null);
            }

            setVisible(false);
         }
      });

      this.add(copyToClipBoard);
      JMenuItem copyFullNameToClipBoard = new JMenuItem("Copy Full Name to Clipboard");
      copyFullNameToClipBoard.addActionListener(new ActionListener()
      {
         public void actionPerformed(ActionEvent e)
         {
            if (selectedVariableHolder.getSelectedVariable() != null)
            {
               StringSelection stringSelection = new StringSelection(selectedVariableHolder.getSelectedVariable().getFullNameWithNameSpace());
               Clipboard clipboard = Toolkit.getDefaultToolkit().getSystemClipboard();
               clipboard.setContents(stringSelection, null);
            }

            setVisible(false);
         }
      });

      this.add(copyFullNameToClipBoard);
      JMenuItem showNameSpace = new JMenuItem("Open Name Space");
      showNameSpace.addActionListener(new ActionListener()
      {
         public void actionPerformed(ActionEvent e)
         {
            if (combinedVarPanel != null)
            {

               combinedVarPanel.setVisibleVarPanel(selectedVariableHolder.getSelectedVariable().getYoVariableRegistry());
            }

            setVisible(false);
         }
      });

      this.add(showNameSpace);

      JMenuItem displayNameSpaces = new JMenuItem("Display Name Spaces");
      displayNameSpaces.addActionListener(new ActionListener()
      {
         public void actionPerformed(ActionEvent e)
         {
            if (combinedVarPanel != null)
            {
               VarPanel.addNameSpaceToVarNames();
            }

            setVisible(false);
         }
      });

      this.add(displayNameSpaces);

      bookmarkVariable = new JMenuItem("Bookmark Variable");
      bookmarkVariable.addActionListener(new ActionListener()
      {
         public void actionPerformed(ActionEvent e)
         {
            if (bookmarkedVariablesHolder != null)
            {
               if (selectedVariableHolder.getSelectedVariable() != null)
               {
                  bookmarkedVariablesHolder.addBookmark(selectedVariableHolder.getSelectedVariable());
               }
            }
            else
            {
               System.err.println("Warning: Could not bookmark variable because bookmarkedVariablesHolder is null.");
            }

            setVisible(false);
         }
      });

      this.add(bookmarkVariable);

      addToSliderBoard = new JMenuItem("addToSliderBoard");
      addToSliderBoard.addActionListener(new ActionListener()
      {
         public void actionPerformed(ActionEvent e)
         {
            if (selectedVariableHolder.getSelectedVariable() != null)
            {
               JFrame tmp = new JFrame();
               tmp.getContentPane().add(new YoSliderpanel(selectedVariableHolder.getSelectedVariable()));
               tmp.setVisible(true);
            }

            setVisible(false);
         }
      });

      this.add(addToSliderBoard);

      // addFocusListener(this);
      // this.addFocusListener(this);
   }

   public void removeBookmarkVariable()
   {
      if (bookmarkVariable != null)
         this.remove(bookmarkVariable);

   }
}
