package us.ihmc.simulationconstructionset.gui.yoVariableSearch;

import java.awt.BorderLayout;
import java.awt.Dimension;
import java.awt.Graphics;
import java.awt.GridBagConstraints;
import java.awt.GridBagLayout;
import java.awt.GridLayout;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.util.ArrayList;
import java.util.concurrent.Executor;
import java.util.concurrent.Executors;

import javax.swing.JCheckBox;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.JScrollPane;
import javax.swing.JSplitPane;
import javax.swing.JTextArea;
import javax.swing.JTextField;
import javax.swing.ScrollPaneConstants;
import javax.swing.SwingUtilities;
import javax.swing.border.EtchedBorder;
import javax.swing.event.ChangeEvent;
import javax.swing.event.ChangeListener;
import javax.swing.event.DocumentEvent;
import javax.swing.event.DocumentListener;

import us.ihmc.graphicsDescription.graphInterfaces.SelectedVariableHolder;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.YoVariable;
import us.ihmc.simulationconstructionset.DataBuffer;
import us.ihmc.simulationconstructionset.DataBufferEntry;
import us.ihmc.simulationconstructionset.gui.BookmarkedVariablesHolder;
import us.ihmc.simulationconstructionset.gui.BookmarkedVariablesPanel;
import us.ihmc.simulationconstructionset.gui.YoVariableExplorerTabbedPane;
import us.ihmc.simulationconstructionset.gui.DoubleClickListener;
import us.ihmc.simulationconstructionset.gui.EntryBoxArrayTabbedPanel;
import us.ihmc.simulationconstructionset.gui.GraphArrayPanel;
import us.ihmc.simulationconstructionset.gui.RegularExpression;
import us.ihmc.simulationconstructionset.gui.YoEntryBox;

public class YoVariableSearchPanel extends JPanel implements ChangeListener
{
   private static final long serialVersionUID = -3986327052893068969L;
   private static final int SCROLL_PANE_INCREMENT = 12;
   private static final boolean USE_BOOKMARKS_PANEL = false;

   private VariableSearchBox variableSearchBox;
   private final DataBuffer dataBuffer;
   private final JCheckBox showInitCheckBox;
   private YoVariableListPanel yoVariableSearchResultsPanel;
   private final JTextArea entryBoxDescriptionArea;
   private final YoEntryBox entryBox;
   private final SelectedVariableHolder holder;
   private JLabel label;

   public YoVariableSearchPanel(SelectedVariableHolder holder, DataBuffer dataBuffer, GraphArrayPanel graphArrayPanel,
                              EntryBoxArrayTabbedPanel entryBoxArrayPanel, BookmarkedVariablesHolder bookmarkedVariablesHolder,
                              YoVariableExplorerTabbedPane combinedVarPanel)
   {
      super(new BorderLayout());

      this.setName("SearchPanel");

      if (bookmarkedVariablesHolder == null)
      {
         System.err.println("Error: Bookmarks panel null!");
      }

      // Setup a scroll panel for the VarPanel, then add it to the center of the display
      this.yoVariableSearchResultsPanel = new YoVariableListPanel("Search", holder,
                                          new YoVariablePanelJPopupMenu(graphArrayPanel, entryBoxArrayPanel, holder, combinedVarPanel, bookmarkedVariablesHolder),
                                          this);
      this.holder = yoVariableSearchResultsPanel.getVariableHolder();
      this.holder.addChangeListener(this);

      JScrollPane searchResultScrollPane = new JScrollPane(yoVariableSearchResultsPanel, ScrollPaneConstants.VERTICAL_SCROLLBAR_ALWAYS,
                                                           ScrollPaneConstants.HORIZONTAL_SCROLLBAR_NEVER);
      searchResultScrollPane.getVerticalScrollBar().setUnitIncrement(SCROLL_PANE_INCREMENT);
      searchResultScrollPane.getVerticalScrollBar().setBlockIncrement(SCROLL_PANE_INCREMENT);
      searchResultScrollPane.setPreferredSize(new Dimension(60, 260));
      searchResultScrollPane.setBorder(new EtchedBorder());

      JSplitPane splitPane = null;
      if (USE_BOOKMARKS_PANEL)
      {
         BookmarkedVariablesPanel bookmarkedVariablesPanel = new BookmarkedVariablesPanel("Bookmarks", yoVariableSearchResultsPanel.selectedVariableHolder, bookmarkedVariablesHolder);
         JScrollPane bookMarkScrollPane = new JScrollPane(bookmarkedVariablesPanel, ScrollPaneConstants.VERTICAL_SCROLLBAR_ALWAYS,
                                                          ScrollPaneConstants.HORIZONTAL_SCROLLBAR_NEVER);
         bookMarkScrollPane.getVerticalScrollBar().setUnitIncrement(SCROLL_PANE_INCREMENT);
         bookMarkScrollPane.getVerticalScrollBar().setBlockIncrement(SCROLL_PANE_INCREMENT);
         bookMarkScrollPane.setPreferredSize(new Dimension(60, 40));
         bookMarkScrollPane.setBorder(new EtchedBorder());
   
         splitPane = new JSplitPane(JSplitPane.VERTICAL_SPLIT, true, searchResultScrollPane, bookMarkScrollPane);
         splitPane.setResizeWeight(1);
         splitPane.setBorder(null);
         splitPane.setDividerSize(3);
         splitPane.setDividerLocation(200);
      }

      // Setup the description panel & YoEntryBox deal
      entryBox = new YoEntryBox(null, holder);

      // Setup a JTextArea to display the description and configure it
      // to word wrap
      entryBoxDescriptionArea = new JTextArea();
      entryBoxDescriptionArea.setEditable(false);
      entryBoxDescriptionArea.setLineWrap(true);
      entryBoxDescriptionArea.setWrapStyleWord(true);
      entryBoxDescriptionArea.setVisible(true);

      JScrollPane descrScroll = new JScrollPane(entryBoxDescriptionArea);
      descrScroll.setVisible(true);

      //    descrPanel.add(entryBox, BorderLayout.NORTH);
      //    descrPanel.add(descrScroll, BorderLayout.CENTER);
      //    descrPanel.setMinimumSize(new Dimension(10, 10));
      //    descrPanel.setPreferredSize(new Dimension(10, 10));
      //    descrPanel.setVisible(false);

      label = new JLabel("Search Settings:");
      label.setAlignmentX(CENTER_ALIGNMENT);

      JLabel varDescription = new JLabel("");
      label.setAlignmentX(CENTER_ALIGNMENT);

      this.add(entryBox, BorderLayout.SOUTH);
      this.add(varDescription, BorderLayout.SOUTH);
      this.add(label, BorderLayout.SOUTH);

      this.dataBuffer = dataBuffer;
      setLayout(new GridBagLayout());
      removeAll();
      GridBagConstraints c = new GridBagConstraints();

      c.fill = GridBagConstraints.HORIZONTAL;
      c.anchor = GridBagConstraints.NORTH;
      c.weightx = 1.0;
      c.weighty = 0.0;
      c.gridx = 0;
      c.gridy = 0;
      c.gridheight = 1;
      c.gridwidth = 1;
      showInitCheckBox = new JCheckBox("Show Variable Init Stacktrace", false);
      this.add(showInitCheckBox, c);

      variableSearchBox = new VariableSearchBox();
      c.gridy++;
      c.gridwidth = 6;
      this.add(variableSearchBox, c);

      c.weighty = 1.0;
      c.gridx = 0;
      c.gridy++;
      c.fill = GridBagConstraints.BOTH;
      c.gridwidth = 6;
      c.gridheight = 5;
      if (USE_BOOKMARKS_PANEL)
      {
         this.add(splitPane, c);
      }
      else
      {
         this.add(searchResultScrollPane, c);
      }

      c.weighty = 0.0;
      c.gridy += 5;
      c.gridheight = 1;
      c.fill = GridBagConstraints.HORIZONTAL;
      this.add(entryBox, c);
   }

   public void setDoubleClickListener(DoubleClickListener listener)
   {
      yoVariableSearchResultsPanel.setDoubleClickListener(listener);
   }

   public boolean showInitStackTrace()
   {
      return showInitCheckBox.isSelected();
   }

   @Override
   public void paintComponent(Graphics g)
   {
      super.paintComponent(g);
      entryBox.updateActiveContainer();
   }

   @Override
   public void stateChanged(ChangeEvent e)
   {
      Runnable entryBoxChanger = new Runnable()
      {
         @Override
         public void run()
         {
            YoVariable<?> selectedVariable = holder.getSelectedVariable();
            if (selectedVariable != null)
            {
               entryBox.addVariable(selectedVariable);
               entryBoxDescriptionArea.setText(selectedVariable.getDescription());
               entryBoxDescriptionArea.setCaretPosition(0);
            }
            else
            {
               entryBox.addVariable(new DoubleYoVariable("null", null));
               entryBoxDescriptionArea.setText("");
            }
         }
      };

      SwingUtilities.invokeLater(entryBoxChanger); // so that we don't interfere with paintComponent
   }

   public class VariableSearchBox extends JPanel implements ActionListener
   {
      private static final long serialVersionUID = 2584008799193548359L;
      private final JTextField searchTextField;
      private final Executor searchExecutor = Executors.newSingleThreadExecutor();
      private Searcher searcher;

      public VariableSearchBox()
      {
         this.setLayout(new GridLayout(1, 1));
         searchTextField = new JTextField();
         searchTextField.setName("SearchTextField");

         searchTextField.addActionListener(this);
         String s = "<html>Search Tips :<br>" + "A * B  = starting with 'A' and ending with 'B' <br>" + "A*  = starting with 'A' <br>"
               + "*A  = ending with 'A' <br>" + "\"A\"  = exactly 'A' <br>" + "A* | B*  anything starting with 'A' or 'B' <br>";

         searchTextField.setToolTipText(s);

         DocumentListener documentListener = new DocumentListener()
         {
            @Override
            public void insertUpdate(DocumentEvent e)
            {
               findMatchingVariablesRegularExpression();
            }

            @Override
            public void removeUpdate(DocumentEvent e)
            {
               findMatchingVariablesRegularExpression();
            }

            @Override
            public void changedUpdate(DocumentEvent e)
            {
            }
         };

         searchTextField.getDocument().addDocumentListener(documentListener);

         this.add(searchTextField);
      }

      @Override
      public void actionPerformed(ActionEvent e)
      {
         findMatchingVariablesRegularExpression();
      }

      public void findMatchingVariablesRegularExpression()
      {
         if (searcher != null)
         {
            searcher.stopSearch();
         }

         searcher = new Searcher(searchTextField.getText().toString());
         searchExecutor.execute(searcher);
      }

      public class Searcher implements Runnable
      {
         private String searchText;
         private boolean stopSearch = false;
         private ArrayList<YoVariable<?>> startsWithSearchTextList = new ArrayList<YoVariable<?>>();
         private ArrayList<YoVariable<?>> doesNotStartWithSearchTextList = new ArrayList<YoVariable<?>>();

         public Searcher(String searchText)
         {
            this.searchText = searchText;
         }

         @Override
         public void run()
         {
            final ArrayList<YoVariable<?>> matchedVariables = search(searchText);

            if (!stopSearch)
               yoVariableSearchResultsPanel.removeAllVariables();

            if (matchedVariables != null)
            {
               for (YoVariable<?> variable : matchedVariables)
               {
                  if (stopSearch)
                     break;
                  yoVariableSearchResultsPanel.addVariable(variable);
               }
            }
         }

         public ArrayList<YoVariable<?>> search(String searchText)
         {
            ArrayList<YoVariable<?>> ret = new ArrayList<YoVariable<?>>();
            ArrayList<DataBufferEntry> entries = dataBuffer.getEntries();
            for (int i = 0; i < entries.size(); i++)
            {
               if (stopSearch)
               {
                  return null;
               }

               DataBufferEntry entry = entries.get(i);
               boolean match = RegularExpression.check(entry.getVariable().getName(), searchText);

               if (match)
               {
                  ret.add(entry.getVariable());
               }
            }

            sortList(ret);

            return ret;
         }

         // display matches in the order: exact, starts with, rest
         private void sortList(ArrayList<YoVariable<?>> list)
         {
            YoVariable<?> temporaryYoVariable;
            String searchTextLowerCase = searchText.toLowerCase();

            for (int i = 0; i < list.size(); i++)
            {
               if ((list.get(i).getName().length() >= searchText.length())
                     && searchTextLowerCase.equals(list.get(i).getName().substring(0, searchText.length())))
               {
                  startsWithSearchTextList.add(list.get(i));
               }
               else
               {
                  doesNotStartWithSearchTextList.add(list.get(i));
               }
            }

            if (startsWithSearchTextList != null)
            {
               for (int i = 0; i < startsWithSearchTextList.size(); i++)
               {
                  if (searchTextLowerCase.equals(startsWithSearchTextList.get(i).getName().toLowerCase()))
                  {
                     temporaryYoVariable = startsWithSearchTextList.get(i);
                     startsWithSearchTextList.set(i, startsWithSearchTextList.get(0));
                     startsWithSearchTextList.set(0, temporaryYoVariable);
                  }
               }

               for (int i = 0; i < doesNotStartWithSearchTextList.size(); i++)
               {
                  startsWithSearchTextList.add(doesNotStartWithSearchTextList.get(i));
               }
            }

            list.clear();

            for (int i = 0; i < startsWithSearchTextList.size(); i++)
            {
               list.add(startsWithSearchTextList.get(i));
            }
         }

         public void stopSearch()
         {
            stopSearch = true;
         }
      }
   }

   public void refreshSearchPanelWidth()
   {
      yoVariableSearchResultsPanel.refreshPanelWidth();
   }
}
