package us.ihmc.simulationconstructionset.gui;

import java.awt.BorderLayout;
import java.awt.Component;
import java.awt.Dimension;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.util.LinkedHashMap;
import java.util.Timer;
import java.util.TimerTask;

import javax.swing.JCheckBox;
import javax.swing.JComponent;
import javax.swing.JFrame;
import javax.swing.JPanel;
import javax.swing.JScrollPane;
import javax.swing.JSplitPane;
import javax.swing.JTabbedPane;
import javax.swing.ScrollPaneConstants;
import javax.swing.event.ChangeEvent;
import javax.swing.event.ChangeListener;

import us.ihmc.graphicsDescription.graphInterfaces.SelectedVariableHolder;
import us.ihmc.simulationconstructionset.commands.WriteDataCommandExecutor;
import us.ihmc.simulationconstructionset.gui.hierarchyTree.NameSpaceHierarchyTree;
import us.ihmc.simulationconstructionset.gui.hierarchyTree.NameSpaceSearchPanel;
import us.ihmc.simulationconstructionset.gui.hierarchyTree.RegistrySelectedListener;
import us.ihmc.simulationconstructionset.gui.yoVariableSearch.YoVariablePanel;
import us.ihmc.simulationconstructionset.gui.yoVariableSearch.YoVariablePanelJPopupMenu;
import us.ihmc.simulationconstructionset.gui.yoVariableSearch.YoVariableSearchPanel;
import us.ihmc.yoVariables.registry.NameSpace;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoVariableList;

public class YoVariableExplorerTabbedPane extends JPanel implements RegistrySelectedListener
{
   private static final long serialVersionUID = -3403238490036872889L;

   private final YoVariableRegistry rootRegistry;
   private final LinkedHashMap<JComponent, Integer> tabIndices = new LinkedHashMap<JComponent, Integer>();
   
   private JTabbedPane tabPane;
   
   private YoVariablePanel visibleVarPanel;
   private YoVariableRegistry visibleVarPanelRegistry;
   
   private JScrollPane scrollPane;
   private NameSpaceHierarchyTree nameSpaceHierarchyTree;
   private JSplitPane splitPane;
   private JPanel variableDisplayPanel;
   private YoVariableSearchPanel variableSearchPanel;
   private YoEntryBox entryBox;
   private boolean onlyParametersState = false;
   private JCheckBox onlyParameters;
   private Timer alertChangeListenersTimer;
   private TimerTask alertChangeListenersTask;
   private final long OBSERVER_NOTIFICATION_PERIOD = 250;
   private JScrollPane bookmarkedVariablesScrollPane;

   private final SelectedVariableHolder selectedVariableHolder;
   private YoVariablePanelJPopupMenu varPanelJPopupMenu;
   private int tabIndex = 0;
   

  
   public YoVariableExplorerTabbedPane(YoVariableDoubleClickListener yoVariableDoubleClickListener, JFrame frame, BookmarkedVariablesHolder bookmarkedVariablesHolder,
                           final SelectedVariableHolder selectedVariableHolder, EntryBoxArrayPanel entryBoxArrayPanel,
                           WriteDataCommandExecutor writeDataCommandExecutor, YoVariableRegistry rootRegistry)
   {
      this.setName("CombinedVarPanel");
      this.setLayout(new BorderLayout());
      
      this.rootRegistry = rootRegistry;
      this.selectedVariableHolder = selectedVariableHolder;
      this.tabPane = new JTabbedPane();
      this.tabPane.addChangeListener(new TabChangedAction());
      
      entryBox = new YoEntryBox(entryBoxArrayPanel, selectedVariableHolder);

      if (selectedVariableHolder != null)
      {
         selectedVariableHolder.addChangeListener(new ChangeListener()
         {
            @Override
            public void stateChanged(ChangeEvent e)
            {
               entryBox.setVariableInThisBox(selectedVariableHolder.getSelectedVariable());
            }
         });
      }

      variableDisplayPanel = new JPanel(new BorderLayout());
      variableDisplayPanel.setName("VariableDisplayPanel");

      onlyParameters = new JCheckBox("Show parameters only");
      onlyParameters.setAlignmentX(Component.LEFT_ALIGNMENT);
      onlyParameters.addActionListener(new ShowOnlyParameterAction());

      scrollPane = new JScrollPane();
      scrollPane.setVerticalScrollBarPolicy(ScrollPaneConstants.VERTICAL_SCROLLBAR_AS_NEEDED);
      scrollPane.setHorizontalScrollBarPolicy(ScrollPaneConstants.HORIZONTAL_SCROLLBAR_NEVER);

      nameSpaceHierarchyTree = new NameSpaceHierarchyTree(this, frame, writeDataCommandExecutor, rootRegistry);
      NameSpaceSearchPanel nameSpaceSearchPanel = new NameSpaceSearchPanel(nameSpaceHierarchyTree);

      BookmarkedVariablesPanel bookmarkedVariablesPanel = new BookmarkedVariablesPanel(new YoVariableList("Bookmarked Variables"), selectedVariableHolder,
                                                             bookmarkedVariablesHolder);
      bookmarkedVariablesScrollPane = new JScrollPane(bookmarkedVariablesPanel, ScrollPaneConstants.VERTICAL_SCROLLBAR_AS_NEEDED,
              ScrollPaneConstants.HORIZONTAL_SCROLLBAR_NEVER);

      splitPane = new JSplitPane(JSplitPane.VERTICAL_SPLIT, true, scrollPane, bookmarkedVariablesScrollPane);
      splitPane.setDividerSize(3);
      splitPane.setResizeWeight(1);
      splitPane.setDividerLocation(200);
      variableDisplayPanel.add(splitPane, BorderLayout.CENTER);
      variableDisplayPanel.add(entryBox, BorderLayout.SOUTH);

      insertTab("Name Space", nameSpaceSearchPanel, tabIndex++);
      insertTab("Variables", variableDisplayPanel, tabIndex++);

      tabPane.setSelectedIndex(0);
      
      this.add(onlyParameters, BorderLayout.NORTH);
      this.add(tabPane, BorderLayout.CENTER);

      tabPane.setMinimumSize(new Dimension(246, 100));
      
      createAndStartPeriodicUIUpdateThread();
   }


   public void addVariableSearchPanel(YoVariableSearchPanel variableSearchPanel)
   {
      insertTab("Search", variableSearchPanel, tabIndex++);
      this.variableSearchPanel = variableSearchPanel;
      tabPane.setSelectedComponent(variableSearchPanel);
      this.variableSearchPanel.setShowOnlyParameters(onlyParameters.isSelected());
   }

   private void insertTab(String name, JComponent component, int tabIndex)
   {
      tabPane.insertTab(name, null, component, null, tabIndex);
      tabIndices.put(component, tabIndex);
   }

// public void addVisibleVarPanel(VarPanel varPanel)
// {
//    // ***jjc removed this because the tree is now generated based on the root tree
//    // nameSpaceHierarchyTree.addNode(varPanel);
//    
////     System.out.println("Adding visible varPanel: " + varPanel.getName());
////     varPanels.put(varPanel.getName(), varPanel);
//
//    if (visibleVarPanel == null)
//    {
////        setVisibleVarPanel(varPanel.getName());
//       setVisibleVarPanel(varPanel);
//    }
// }

   public void addExtraVarPanel(YoVariablePanel extraVarPanel)
   {
      setVisibleVarPanel(extraVarPanel, null);
   }

   public void setVisibleVarPanel(String nameSpaceName)
   {
      NameSpace fullNameSpace = new NameSpace(nameSpaceName);

      YoVariableRegistry registry = rootRegistry.getRegistry(fullNameSpace);

      if (registry == null)
      {
         System.err.println("CombinedVarPanel.setVisibleVarPanel() can't find registry named " + nameSpaceName);
      }

      setVisibleVarPanel(registry);
   }

   public void setVisibleVarPanel(YoVariableRegistry registry)
   {
      YoVariablePanel varPanel;
      if(onlyParameters.isSelected())
      {
         varPanel = new YoVariableRegistryParameterPanel(registry, selectedVariableHolder, varPanelJPopupMenu);
      }
      else
      {
         varPanel = new YoVariableRegistryVarPanel(registry, selectedVariableHolder, varPanelJPopupMenu);
      }

      setVisibleVarPanel(varPanel, registry);
   }

   private void setVisibleVarPanel(YoVariablePanel varPanel, YoVariableRegistry registry)
   {
      if (visibleVarPanel != null)
      {
         scrollPane.remove(visibleVarPanel);
      }

      visibleVarPanel = varPanel;
      visibleVarPanelRegistry = registry;
      scrollPane.add(visibleVarPanel);
      scrollPane.setViewportView(visibleVarPanel);
      Integer tabIndex = tabIndices.get(variableDisplayPanel);
      tabPane.setSelectedIndex(tabIndex);

      // StringTokenizer nameTokenizer = new StringTokenizer(name, ".");
      String name = varPanel.getName();
      String[] split = name.split("\\.");

      if (split.length > 0)
      {
         tabPane.setTitleAt(tabIndex, split[split.length - 1]);
      }

      updateOnlyParametersCheckboxEnabled();
   }

   public YoVariablePanel getVisibleVarPanel()
   {
      return visibleVarPanel;
   }

   private void createAndStartPeriodicUIUpdateThread()
   {
      alertChangeListenersTimer = new Timer("CombinedVarPanelTimer");
      alertChangeListenersTask = new TimerTask()
      {
         @Override
         public void run()
         {
            repaint();

            EventDispatchThreadHelper.justRun(new Runnable()
            {
               @Override
               public void run()
               {
                  entryBox.updateActiveContainer();
               }
            });
         } 

      };
      alertChangeListenersTimer.schedule(alertChangeListenersTask, 1000, OBSERVER_NOTIFICATION_PERIOD);
   }

   public void closeAndDispose()
   {
      EventDispatchThreadHelper.invokeAndWait(new Runnable()
      {
         @Override
         public void run()
         {
            closeAndDisposeLocal();
         }
      });
   }
   
   private void closeAndDisposeLocal()
   {
      this.removeAll();

//    if (varPanels != null)
//    {
//       varPanels.clear();
//       varPanels = null;
//    }

      visibleVarPanel = null;

      onlyParameters = null;
      
      if (scrollPane != null)
      {
         scrollPane.removeAll();
         scrollPane = null;
      }

      nameSpaceHierarchyTree = null;

      if (splitPane != null)
      {
         splitPane.removeAll();
         splitPane = null;
      }

      if (variableDisplayPanel != null)
      {
         variableDisplayPanel.removeAll();
         variableDisplayPanel = null;
      }

      variableSearchPanel.removeAll();
      variableSearchPanel = null;

      entryBox = null;

      if (alertChangeListenersTimer != null)
      {
         alertChangeListenersTimer.cancel();
         alertChangeListenersTimer = null;
      }

      alertChangeListenersTask = null;

      if (bookmarkedVariablesScrollPane != null)
      {
         bookmarkedVariablesScrollPane.removeAll();
         bookmarkedVariablesScrollPane = null;
      }
   }

   public void showNameSpace(YoVariableRegistry titleOfNameSpace)
   {
      tabPane.setSelectedComponent(nameSpaceHierarchyTree);
      nameSpaceHierarchyTree.showNameSpace(titleOfNameSpace);
   }

   public NameSpaceHierarchyTree getNameSpaceHierarchyTree()
   {
      return nameSpaceHierarchyTree;
   }

   @Override
   public void registryWasSelected(YoVariableRegistry selectedRegistry)
   {
      setVisibleVarPanel(selectedRegistry);
   }

   public void setVarPanelJPopupMenu(YoVariablePanelJPopupMenu varPanelJPopupMenu)
   {
      this.varPanelJPopupMenu = varPanelJPopupMenu;
   }


   public YoVariableSearchPanel getYoVariableSearchPanel()
   {
      return variableSearchPanel;
   }

   private void updateOnlyParametersCheckboxEnabled()
   {
      if (tabPane.getSelectedComponent().equals(variableDisplayPanel) && visibleVarPanel != null && visibleVarPanelRegistry == null)
      {
         onlyParametersState = onlyParameters.isSelected();
         onlyParameters.setEnabled(false);
         onlyParameters.setSelected(false);
      }

      else if (!onlyParameters.isEnabled())
      {
         onlyParameters.setSelected(onlyParametersState);
         onlyParameters.setEnabled(true);
      }
   }

   private class TabChangedAction implements ChangeListener
   {

      @Override
      public void stateChanged(ChangeEvent e)
      {
         updateOnlyParametersCheckboxEnabled();
      }

   }

   private class ShowOnlyParameterAction implements ActionListener
   {

      @Override
      public void actionPerformed(ActionEvent e)
      {
         if (nameSpaceHierarchyTree != null)
         {
            nameSpaceHierarchyTree.filterParameters(onlyParameters.isSelected());
         }

         if (visibleVarPanelRegistry != null)
         {
            setVisibleVarPanel(visibleVarPanelRegistry);
         }

         if (variableSearchPanel != null)
         {
            variableSearchPanel.setShowOnlyParameters(onlyParameters.isSelected());
         }
      }

   }
}
