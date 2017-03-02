package us.ihmc.simulationconstructionset.gui;

import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JMenuItem;
import javax.swing.JPanel;
import javax.swing.JSpinner;

import org.fest.swing.core.GenericTypeMatcher;
import org.fest.swing.core.MouseButton;
import org.fest.swing.edt.FailOnThreadViolationRepaintManager;
import org.fest.swing.edt.GuiActionRunner;
import org.fest.swing.edt.GuiQuery;
import org.fest.swing.exception.ComponentLookupException;
import org.fest.swing.exception.WaitTimedOutError;
import org.fest.swing.finder.FrameFinder;
import org.fest.swing.finder.WindowFinder;
import org.fest.swing.fixture.FrameFixture;
import org.fest.swing.fixture.JButtonFixture;
import org.fest.swing.fixture.JCheckBoxFixture;
import org.fest.swing.fixture.JComboBoxFixture;
import org.fest.swing.fixture.JLabelFixture;
import org.fest.swing.fixture.JMenuItemFixture;
import org.fest.swing.fixture.JPanelFixture;
import org.fest.swing.fixture.JPopupMenuFixture;
import org.fest.swing.fixture.JSpinnerFixture;
import org.fest.swing.fixture.JTabbedPaneFixture;
import org.fest.swing.fixture.JTextComponentFixture;
import org.fest.swing.fixture.JTreeFixture;

import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.tools.thread.ThreadTools;

public class SimulationGUITestFixture
{
   private FrameFixture focusedWindow;
   private final FrameFixture mainSCSWindow;

   public SimulationGUITestFixture(final SimulationConstructionSet scs)
   {
      FailOnThreadViolationRepaintManager.install();

      JFrame frame = GuiActionRunner.execute(new GuiQuery<JFrame>()
      {
         @Override
         protected JFrame executeInEDT() throws SimulationExceededMaximumTimeException
         {
            JFrame jFrame = scs.getJFrame();

            return jFrame;
         }
      });
      focusedWindow = new FrameFixture(frame);
      
      ThreadTools.sleep(1000);
      focusedWindow.focus();

      mainSCSWindow = focusedWindow;
   }
   

   public void closeAllViewportWindows()
   {
      while (true)
      {
         NthViewportWindowMatcher matcher = new NthViewportWindowMatcher(0);
         FrameFixture window = getWindowIfItExists(matcher);
         if (window == null)
         {
            return;
         }

         window.moveToFront();
         window.focus();
         window.close();
      }
      
   }

   public void focusNthViewportWindow(int n)
   {
      NthViewportWindowMatcher matcher = new NthViewportWindowMatcher(n);
      FrameFinder findFrame = WindowFinder.findFrame(matcher);
      FrameFixture frameFixture = findFrame.using(focusedWindow.robot);
      focusedWindow = frameFixture;
      focusedWindow.focus();
      
   }

   public void closeAllGraphArrayWindows()
   {
      while (true)
      {
         NthGraphArrayWindowMatcher matcher = new NthGraphArrayWindowMatcher(0);
         FrameFixture window = getWindowIfItExists(matcher);
         if (window == null)
         {
            return;
         }

         window.moveToFront();
         window.focus();
         window.close();
      }
   }

   public void focusNthGraphArrayWindow(int n)
   {
      NthGraphArrayWindowMatcher matcher = new NthGraphArrayWindowMatcher(n);
      FrameFinder findFrame = WindowFinder.findFrame(matcher);
      FrameFixture frameFixture = findFrame.using(focusedWindow.robot);
      focusedWindow = frameFixture;
      focusedWindow.focus();
   }
   
   public void focusWindow(String name)
   {
      FrameFinder findFrame = WindowFinder.findFrame(name);
      FrameFixture frameFixture = findFrame.using(focusedWindow.robot);
      focusedWindow = frameFixture;
      focusedWindow.focus();      
   }
   

   public void focusDialog(String name)
   {
      focusedWindow.dialog(name);
   }

   public void focusMainSCSWindow()
   {
      focusedWindow = mainSCSWindow;
      focusedWindow.focus();
   }

   public void showWindow()
   {
      focusedWindow.show();
   }

   public void closeAndDispose()
   {
      if (focusedWindow != null)
         focusedWindow.cleanUp();
   }

   // Buttons
   public void clickExportDataButton()
   {
      JButtonFixture exportDataButton = focusedWindow.button("Export Data...");
      exportDataButton.click();
   }

   public void clickImportDataButton()
   {
      JButtonFixture importDataButton = focusedWindow.button("Import Data...");
      importDataButton.click();
   }

   public void clickMediaCaptureButton()
   {
      JButtonFixture mediaCaptureButton = focusedWindow.button("Media Capture...");
      mediaCaptureButton.click();
   }

   public void clickExportSnapshotButton()
   {
      JButtonFixture exportSnapshotButton = focusedWindow.button("Export Snapshot...");
      exportSnapshotButton.click();
   }

   public void clickSimulateButton()
   {
      JButtonFixture simulateButton = focusedWindow.button("Simulate");
      simulateButton.click();
   }

   public void clickPlayButton()
   {
      JButtonFixture playButton = focusedWindow.button("Play");
      playButton.click();
   }

   public void clickStopButton()
   {
      JButtonFixture stopButton = focusedWindow.button("Stop");
      stopButton.click();
   }

   public void clickSetInPointButton()
   {
      JButtonFixture setInPointButton = focusedWindow.button("Set In Point");
      setInPointButton.click();
   }

   public void clickGotoInPointButton()
   {
      JButtonFixture gotoInPointButton = focusedWindow.button("Goto In Point");
      gotoInPointButton.click();
   }

   public void clickStepBackwardButton()
   {
      JButtonFixture stepBackwardButton = focusedWindow.button("Step Backward");
      stepBackwardButton.click();
   }

   public void clickStepForwardButton()
   {
      JButtonFixture stepForwardButton = focusedWindow.button("Step Forward");
      stepForwardButton.click();
   }

   public void clickGotoOutPointButton()
   {
      JButtonFixture gotoOutPointButton = focusedWindow.button("Goto Out Point");
      gotoOutPointButton.click();
   }

   public void clickSetOutPointButton()
   {
      JButtonFixture setOutPointButton = focusedWindow.button("Set Out Point");
      setOutPointButton.click();
   }

   public void clickAddKeyPointButton()
   {
      JButtonFixture addKeyPointButton = focusedWindow.button("Add Key Point");
      addKeyPointButton.click();
   }

   public void clickToggleKeyModeButton()
   {
      JButtonFixture toggleKeyModeButton = focusedWindow.button("Toggle Key Mode");
      toggleKeyModeButton.click();
   }

   public void clickZoomInButton()
   {
      JButtonFixture zoomInButton = focusedWindow.button("Zoom In");
      zoomInButton.click();
   }

   public void clickZoomOutButton()
   {
      JButtonFixture zoomOutButton = focusedWindow.button("Zoom Out");
      zoomOutButton.click();
   }

   public void clickPrintGraphsButton()
   {
      JButtonFixture printGraphsButton = focusedWindow.button("Print Graphs");
      printGraphsButton.click();
   }

   public void clickTrackCheckBox()
   {
      JCheckBoxFixture trackCheckBox = focusedWindow.checkBox("Track");
      trackCheckBox.click();
   }

   public void clickDollyCheckBox()
   {
      JCheckBoxFixture dollyCheckBox = focusedWindow.checkBox("Dolly");
      dollyCheckBox.click();
   }


   // Menus
   public void selectFileMenu()
   {
      JMenuItemFixture fileMenu = focusedWindow.menuItem("File");
      fileMenu.click();
   }

   public void selectRunMenu()
   {
      JMenuItemFixture runMenu = focusedWindow.menuItem("Run");
      runMenu.click();
   }

   public void selectConfigurationMenu()
   {
      JMenuItemFixture configurationMenu = focusedWindow.menuItem("Configuration");
      configurationMenu.click();
   }

   public void selectGraphsMenu()
   {
      JMenuItemFixture graphsMenu = focusedWindow.menuItem("Graphs");
      graphsMenu.click();
   }


   public void selectDataBufferMenu()
   {
      JMenuItemFixture dataBufferMenu = focusedWindow.menuItem("Data Buffer");
      dataBufferMenu.click();
   }

   public void selectViewportMenu()
   {
      JMenuItemFixture viewportMenu = focusedWindow.menuItem("Viewport");
      viewportMenu.click();
   }

   public void selectWindowMenu()
   {
      JMenuItemFixture windowMenu = focusedWindow.menuItem("Window");
      windowMenu.click();
   }

   public void selectHelpMenu()
   {
      JMenuItemFixture helpMenu = focusedWindow.menuItem("Help");
      helpMenu.click();
   }

   public void selectPlayMenu()
   {
      MenuItemTextTypeMatcher matcher = new MenuItemTextTypeMatcher("Play");
      JMenuItemFixture playMenu = focusedWindow.menuItem(matcher);
      playMenu.click();
   }

   public void selectNewGraphWindowMenu()
   {
      MenuItemTextTypeMatcher matcher = new MenuItemTextTypeMatcher("New Graph Window");
      JMenuItemFixture newGraphWindowMenu = focusedWindow.menuItem(matcher);
      newGraphWindowMenu.click();
   }

   public void selectNewViewportWindowMenu()
   {
      MenuItemTextTypeMatcher matcher = new MenuItemTextTypeMatcher("New Viewport Window");
      JMenuItemFixture newViewportWindowMenu = focusedWindow.menuItem(matcher);
      newViewportWindowMenu.click();
   }

   public void selectPlaybackPropertiesMenu()
   {
      MenuItemTextTypeMatcher matcher = new MenuItemTextTypeMatcher("Playback Properties...");
      JMenuItemFixture playbackPropertiesMenu = focusedWindow.menuItem(matcher);
      playbackPropertiesMenu.click();
   }

   public void selectNameSpaceTab()
   {
      JTabbedPaneFixture combinedVarPanelTabbedPane = focusedWindow.tabbedPane("CombinedVarPanel");
      combinedVarPanelTabbedPane.selectTab("Name Space");
   }

   public void selectNameSpace(String nameSpace)
   {
      JTabbedPaneFixture combinedVarPanelTabbedPane = focusedWindow.tabbedPane("CombinedVarPanel");
      combinedVarPanelTabbedPane.selectTab("Name Space");
      JTreeFixture tree = focusedWindow.tree();

      tree.doubleClickPath(nameSpace);
   }

   public void selectSearchTab()
   {
      JTabbedPaneFixture combinedVarPanelTabbedPane = focusedWindow.tabbedPane("CombinedVarPanel");
      combinedVarPanelTabbedPane.selectTab("Search");
   }

   public void deleteSearchText()
   {
      JPanelFixture searchPanel = focusedWindow.panel("SearchPanel");
      JTextComponentFixture searchTextField = searchPanel.textBox("SearchTextField");
      searchTextField.deleteText();
   }

   public void enterSearchText(String text)
   {
      JPanelFixture searchPanel = focusedWindow.panel("SearchPanel");
      JTextComponentFixture searchTextField = searchPanel.textBox("SearchTextField");
      searchTextField.enterText(text);
   }

   public void selectVariableInOpenTab(String variableNameEndsWith)
   {
      JSpinnerNameEndsWithMatcher matcher = new JSpinnerNameEndsWithMatcher(variableNameEndsWith);

      // Focusing the spinner seems to do the trick, though this seems hackish and brittle.

      JSpinnerFixture spinner = focusedWindow.spinner(matcher);
      spinner.focus();
   }

   public void selectVariableInSearchTab(String variableNameEndsWith)
   {
      JPanelFixture searchPanel = focusedWindow.panel("SearchPanel");
      JPanelFixture searchVarListVarPanel = searchPanel.panel("Search");

      JSpinnerNameEndsWithMatcher matcher = new JSpinnerNameEndsWithMatcher(variableNameEndsWith);

      // Focusing the spinner seems to do the trick, though this seems hackish and brittle.

      JSpinnerFixture spinner = searchVarListVarPanel.spinner(matcher);
      spinner.focus();
   }

   public void selectVariableAndSetValueInSearchTab(String variableNameEndsWith, double value)
   {
      JPanelFixture searchPanel = focusedWindow.panel("SearchPanel");
      JPanelFixture searchVarListVarPanel = searchPanel.panel("Search");

      JSpinnerNameEndsWithMatcher matcher = new JSpinnerNameEndsWithMatcher(variableNameEndsWith);

      JSpinnerFixture spinner = searchVarListVarPanel.spinner(matcher);

      spinner.click();
      spinner.enterTextAndCommit(String.valueOf(value));
   }

   public void middleClickInEmptyGraph()
   {
      YoGraphIsEmptyMatcher matcher = new YoGraphIsEmptyMatcher();

      JPanelFixture panel = focusedWindow.panel(matcher);
      panel.click(MouseButton.MIDDLE_BUTTON);
   }

   public void middleClickInNthGraph(int nThToFind)
   {
      NthYoGraphMatcher matcher = new NthYoGraphMatcher(nThToFind);

      JPanelFixture panel = focusedWindow.panel(matcher);
      panel.click(MouseButton.MIDDLE_BUTTON);
   }

   public void removeAllGraphs()
   {
      this.clickRemoveEmptyGraphButton();

      while (true)
      {
         NthYoGraphMatcher matcher = new NthYoGraphMatcher(0);

         JPanelFixture panel = getPanelIfItExists(matcher);
         if (panel == null)
            return;

         panel.rightClick();
         JPopupMenuFixture popupMenu = new JPopupMenuFixture(panel.robot, panel.showPopupMenu().target);
         MenuItemTextTypeMatcher textMatcher = new MenuItemTextTypeMatcher("Delete Graph");
         popupMenu.menuItem(textMatcher).click();
      }

   }

   private FrameFixture getWindowIfItExists(GenericTypeMatcher<JFrame> matcher)
   {
      try
      {
         FrameFinder findFrame = WindowFinder.findFrame(matcher);
         FrameFixture frameFixture = findFrame.using(focusedWindow.robot);

         return frameFixture;
      }
      catch (WaitTimedOutError e)
      {
      }

      return null;
   }
   
   private JPanelFixture getPanelIfItExists(GenericTypeMatcher<JPanel> matcher)
   {
      try
      {
         return focusedWindow.panel(matcher);
      }
      catch (ComponentLookupException e)
      {
      }

      return null;
   }


   public void removeVariableFromNthGraph(String variableName, int nThToFind)
   {
      NthYoGraphMatcher matcher = new NthYoGraphMatcher(nThToFind);

      JPanelFixture panel = focusedWindow.panel(matcher);
      panel.rightClick();

      JPopupMenuFixture popupMenu = new JPopupMenuFixture(panel.robot, panel.showPopupMenu().target);

      MenuItemTextTypeMatcher textMatcher = new MenuItemTextTypeMatcher("Remove " + variableName);
      popupMenu.menuItem(textMatcher).click();

   }

   public void clickOnUnusedEntryBox()
   {
      JPanelFixture unusedEntryBox = focusedWindow.panel("UNUSED");
      unusedEntryBox.click();

      LabelTextMatcher matcher = new LabelTextMatcher("UNUSED");
      JLabelFixture label = unusedEntryBox.label(matcher);
      label.click();
   }

   public void removeAllEntryBoxes()
   {
      while (true)
      {
         NthEntryBoxMatcher matcher = new NthEntryBoxMatcher(0);

         JPanelFixture panel = getPanelIfItExists(matcher);
         if (panel == null)
            return;

         panel.rightClick();
         JPopupMenuFixture popupMenu = new JPopupMenuFixture(panel.robot, panel.showPopupMenu().target);
         MenuItemTextTypeMatcher textMatcher = new MenuItemTextTypeMatcher("Delete Entry Box");
         popupMenu.menuItem(textMatcher).click();
      }
   }

   public void removeNthEntryBox(int n)
   {
      NthEntryBoxMatcher matcher = new NthEntryBoxMatcher(n);
      JPanelFixture entryBoxPanel = focusedWindow.panel(matcher);
      entryBoxPanel.rightClick();

      JPopupMenuFixture popupMenu = new JPopupMenuFixture(entryBoxPanel.robot, entryBoxPanel.showPopupMenu().target);
      MenuItemTextTypeMatcher textMatcher = new MenuItemTextTypeMatcher("Delete Entry Box");
      popupMenu.menuItem(textMatcher).click();
   }

   public void findEntryBoxAndEnterValue(String name, double value)
   {
      JPanelFixture entryBoxArrayPanel = focusedWindow.panel("EntryBoxArrayPanel");
      JPanelFixture enumEntryBox = entryBoxArrayPanel.panel(name + "_YoEntryBox");
      JTextComponentFixture textBox = enumEntryBox.textBox();

      // For some reason deleting, and then entering doesn't seem to work. It only deletes part of the text!?
      // Instead here we have to call setText.
      // textBox.deleteText();
      // textBox.enterText(Double.toString(value) + "\n");

      textBox.setText(Double.toString(value));
      textBox.enterText("\n");
      ThreadTools.sleep(500);
   }

   public void findEnumEntryBoxAndSelectValue(String name, String value)
   {
      JPanelFixture entryBoxArrayPanel = focusedWindow.panel("EntryBoxArrayPanel");
      JPanelFixture enumEntryBox = entryBoxArrayPanel.panel(name + "_YoEntryBox");
      JComboBoxFixture comboBox = enumEntryBox.comboBox();
      comboBox.selectItem(value);
   }


   public void clickNewGraphButton()
   {
      JButtonFixture newGraphButton = focusedWindow.button("New Graph");
      newGraphButton.click();
   }

   public void clickRemoveEmptyGraphButton()
   {
      JButtonFixture removeEmptyGraphButton = focusedWindow.button("Remove Empty");
      removeEmptyGraphButton.click();
   }

   public void clickAddGraphColumnButton()
   {
      JButtonFixture addGraphColumnButton = focusedWindow.button("Add Column");
      addGraphColumnButton.click();
   }

   public void clickSubGraphColumnButton()
   {
      JButtonFixture subGraphColumnButton = focusedWindow.button("Sub Column");
      subGraphColumnButton.click();
   }


   private class MenuItemTextTypeMatcher extends GenericTypeMatcher<JMenuItem>
   {
      private final String text;

      public MenuItemTextTypeMatcher(String text)
      {
         super(JMenuItem.class);
         this.text = text;
      }

      @Override
      protected boolean isMatching(JMenuItem component)
      {
         return text.equals(component.getText());
      }
   }


   private class JSpinnerNameEndsWithMatcher extends GenericTypeMatcher<JSpinner>
   {
      private final String nameEnding;

      public JSpinnerNameEndsWithMatcher(String nameEnding)
      {
         super(JSpinner.class);
         this.nameEnding = nameEnding;
      }

      @Override
      protected boolean isMatching(JSpinner component)
      {
         return component.getName().endsWith(nameEnding);
      }
   }


   private class YoGraphIsEmptyMatcher extends GenericTypeMatcher<JPanel>
   {
      public YoGraphIsEmptyMatcher()
      {
         super(JPanel.class);
      }

      @Override
      protected boolean isMatching(JPanel component)
      {
         String name = component.getName();
         if (name == null)
            return false;

         boolean isAYoGraph = name.equals("YoGraph");
         if (!isAYoGraph)
            return false;

         YoGraph yoGraph = (YoGraph) component;

         return yoGraph.isEmpty();
      }
   }


   private class NthYoGraphMatcher extends GenericTypeMatcher<JPanel>
   {
      private final int nThToFind;
      private int currentIndex = 0;

      public NthYoGraphMatcher(int nThToFind)
      {
         super(JPanel.class);
         this.nThToFind = nThToFind;
      }

      @Override
      protected boolean isMatching(JPanel component)
      {
         String name = component.getName();
         if (name == null)
            return false;

         boolean isAYoGraph = name.equals("YoGraph");
         if (!isAYoGraph)
            return false;

         if (currentIndex == nThToFind)
         {
            currentIndex++;

            return true;
         }

         currentIndex++;

         return false;
      }
   }


   private class NthEntryBoxMatcher extends GenericTypeMatcher<JPanel>
   {
      private final int nThToFind;
      private int currentIndex = 0;

      public NthEntryBoxMatcher(int nThToFind)
      {
         super(JPanel.class);
         this.nThToFind = nThToFind;
      }

      @Override
      protected boolean isMatching(JPanel component)
      {
         String name = component.getName();
         if (name == null)
            return false;

         boolean isAYoGraph = name.endsWith("YoEntryBox");
         if (!isAYoGraph)
            return false;

         if (currentIndex == nThToFind)
         {
            currentIndex++;

            return true;
         }

         currentIndex++;

         return false;
      }
   }



   private class LabelTextMatcher extends GenericTypeMatcher<JLabel>
   {
      private final String labelText;

      public LabelTextMatcher(String labelText)
      {
         super(JLabel.class);
         this.labelText = labelText;
      }

      @Override
      protected boolean isMatching(JLabel label)
      {
         return label.getText().equals(labelText);
      }
   }


   private class NthGraphArrayWindowMatcher extends GenericTypeMatcher<JFrame>
   {
      private final int nThToFind;
      private int currentIndex = 0;

      public NthGraphArrayWindowMatcher(int nThToFind)
      {
         super(JFrame.class);
         this.nThToFind = nThToFind;
      }

      @Override
      protected boolean isMatching(JFrame component)
      {
         String name = component.getName();
         if (name == null)
            return false;

         boolean isAGraphWindow = name.startsWith("Graph Window");
         if (!isAGraphWindow)
            return false;

         if (!component.isVisible()) return false;
         
         if (currentIndex == nThToFind)
         {
            currentIndex++;

            return true;
         }

         currentIndex++;

         return false;
      }
   }
   
   private class NthViewportWindowMatcher extends GenericTypeMatcher<JFrame>
   {
      private final int nThToFind;
      private int currentIndex = 0;

      public NthViewportWindowMatcher(int nThToFind)
      {
         super(JFrame.class);
         this.nThToFind = nThToFind;
      }

      @Override
      protected boolean isMatching(JFrame component)
      {
         String name = component.getName();
         if (name == null)
            return false;

         boolean isAGraphWindow = name.startsWith("Viewport Window");
         if (!isAGraphWindow)
            return false;

         if (!component.isVisible()) return false;
         
         if (currentIndex == nThToFind)
         {
            currentIndex++;

            return true;
         }

         currentIndex++;

         return false;
      }
   }


}
