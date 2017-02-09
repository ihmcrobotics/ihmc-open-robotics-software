package us.ihmc.simulationconstructionset.gui.dialogConstructors;

import java.awt.Component;
import java.awt.GridBagConstraints;
import java.awt.GridBagLayout;
import java.awt.Insets;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.util.StringTokenizer;

import javax.swing.JCheckBox;
import javax.swing.JFileChooser;
import javax.swing.JFrame;
import javax.swing.JPanel;

import us.ihmc.jMonkeyEngineToolkit.camera.CameraConfiguration;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.gui.GUIConfigurationSaveAndLoad;
import us.ihmc.simulationconstructionset.gui.StandardSimulationGUI;
import us.ihmc.tools.gui.MyFileFilter;
import us.ihmc.tools.io.xml.XMLReaderUtility;

public class LoadConfigurationDialogGenerator implements LoadConfigurationDialogConstructor
{
   private javax.swing.filechooser.FileFilter dataFileFilter = new MyFileFilter(new String[] {".guiConf"}, "Gui Configuration (.guiConf)");
   private JFileChooser dataFileChooser;
   private JFrame frame;
   private StandardSimulationGUI myGUI;
   private SimulationConstructionSet sim;
   private boolean loadGraphGroups = true;
   private boolean loadEntryBoxes = true;
   private boolean loadViewPorts = false;
   private boolean loadGraphWindows = true;
   private boolean loadJPanels = false;
   private boolean loadMultiViews = false;

   public LoadConfigurationDialogGenerator(SimulationConstructionSet sim, JFrame frame, StandardSimulationGUI myGUI)
   {
      this.frame = frame;
      this.sim = sim;
      this.myGUI = myGUI;

      try
      {
         dataFileChooser = new JFileChooser();

         JPanel checkBoxes = new JPanel(new GridBagLayout());
         GridBagConstraints g = new GridBagConstraints();
         g.weightx = 1;
         g.weighty = 1;
         g.fill = GridBagConstraints.NONE;
         g.gridx = 0;
         g.gridy = 0;
         g.anchor = GridBagConstraints.NORTHWEST;
         g.insets = new Insets(0, 0, 0, 0);

         final JCheckBox graphGroups = new JCheckBox("Load Graph Groups", loadGraphGroups);
         graphGroups.addActionListener(new ActionListener()
         {
            @Override
            public void actionPerformed(ActionEvent e)
            {
               loadGraphGroups = graphGroups.isSelected();
            }
         });
         checkBoxes.add(graphGroups, g);
         g.gridy++;

         final JCheckBox entryBoxes = new JCheckBox("Load Entry Boxes", loadEntryBoxes);
         entryBoxes.addActionListener(new ActionListener()
         {
            @Override
            public void actionPerformed(ActionEvent e)
            {
               loadEntryBoxes = entryBoxes.isSelected();
            }
         });
         checkBoxes.add(entryBoxes, g);
         g.gridy++;

         final JCheckBox viewPorts = new JCheckBox("Load Extra View Ports", loadViewPorts);
         viewPorts.addActionListener(new ActionListener()
         {
            @Override
            public void actionPerformed(ActionEvent e)
            {
               loadViewPorts = viewPorts.isSelected();
            }
         });
         checkBoxes.add(viewPorts, g);
         g.gridy++;

         final JCheckBox graphWindows = new JCheckBox("Load Graph Windows", loadGraphWindows);
         graphWindows.addActionListener(new ActionListener()
         {
            @Override
            public void actionPerformed(ActionEvent e)
            {
               loadGraphWindows = graphWindows.isSelected();
            }
         });
         checkBoxes.add(graphWindows, g);
         g.gridy++;

         final JCheckBox loadjpanels = new JCheckBox("Load Extra JPanels", loadJPanels);
         loadjpanels.addActionListener(new ActionListener()
         {
            @Override
            public void actionPerformed(ActionEvent e)
            {
               loadJPanels = loadjpanels.isSelected();
            }
         });
         checkBoxes.add(loadjpanels, g);
         g.gridy++;

         final JCheckBox multiviews = new JCheckBox("Load Multiple Views", loadMultiViews);
         multiviews.addActionListener(new ActionListener()
         {
            @Override
            public void actionPerformed(ActionEvent e)
            {
               loadMultiViews = multiviews.isSelected();
            }
         });
         checkBoxes.add(multiviews, g);
         g.gridy++;

         dataFileChooser.setAccessory(checkBoxes);

         File Configs = new File("Configurations");
         if (!Configs.exists())
         {
            Configs.mkdir();
         }

         String path = Configs.toURI().getPath();
         setCurrentDirectory(path);

         dataFileChooser.setAcceptAllFileFilterUsed(false);
         dataFileChooser.addChoosableFileFilter(dataFileFilter);
      }
      catch (Exception e)
      {
         // e.printStackTrace();
      }
   }

   @Override
   public void setCurrentDirectory(File dir)
   {
      dataFileChooser.setCurrentDirectory(dir);
   }

   @Override
   public void setCurrentDirectory(String dir)
   {
      dataFileChooser.setCurrentDirectory(new File(dir));
   }

   @Override
   public void constructDialog()
   {
      load();
   }

   public void load()
   {
      sim.disableGUIComponents();


      if (dataFileChooser.showOpenDialog(frame) == JFileChooser.APPROVE_OPTION)
      {
         File chosenFile = dataFileChooser.getSelectedFile();

         loadGUIConfigurationFile(chosenFile);
      }

      sim.enableGUIComponents();
   }

   /*
    *  Note that this method is called on startup of SCS
    * Therefore, if you want to load a configuration on start up,
    * you need to wait a few seconds until SCS has started before
    * calling this method with the desired GUI config.
    */
   @Override
   public synchronized void loadGUIConfigurationFile(File file)
   {
      String fileEnding = ".guiConf";

      // TODO Auto-generated method stub
      try
      {
         String filename = file.getName();

         if (!filename.endsWith(fileEnding))
         {
            filename = filename.concat(fileEnding);

            if (!file.getName().equals(filename))
            {
               File newChosenFile = new File(file.getParent(), filename);

               file = newChosenFile;
            }
         }

         String name = file.getName().substring(0, file.getName().length() - fileEnding.length());

         BufferedReader reader = new BufferedReader(new FileReader(file));
         String xmlRepresentation = "";
         String tempLine;

         while ((tempLine = reader.readLine()) != null)
         {
            xmlRepresentation += tempLine;
         }

         if (loadGraphGroups)
            GUIConfigurationSaveAndLoad.setupGraphGroups(myGUI, xmlRepresentation, name);

         if (loadEntryBoxes)
            loadEntryBoxArrayPanel(xmlRepresentation, file);

         if (loadViewPorts)
         {
            setupMainViewport(xmlRepresentation);
            setupViewportWindows(xmlRepresentation);
         }

         if (loadGraphWindows)
         {
            GUIConfigurationSaveAndLoad.setupGraphWindows(myGUI, xmlRepresentation, "GraphWindows" + name);
            setupConfiguration(name);
         }

         if (loadMultiViews)
         {
            setupMultiViews(xmlRepresentation);
         }

         if (loadJPanels)
         {
            setupJPanels(xmlRepresentation, myGUI.canvas); //TODO: Why is this here?
         }

         myGUI.makeCheckBoxesConsistentWithCamera();

         reader.close();
         System.out.println("Your file has been loaded.");
      }
      catch (Exception e)
      {
         e.printStackTrace();
      }
   }

   public void setupMultiViews(String xmlRepresentation)
   {
      myGUI.addViewportPanelToMainPanel(); //TODO: Why is this here?
      String CurrentView = XMLReaderUtility.getMiddleString(0, xmlRepresentation, "<Current View>", "</Current View>");
      myGUI.selectViewport(CurrentView);
      myGUI.setupMultiViews(xmlRepresentation, CurrentView);
   }

   public void setupJPanels(String xmlRepresentation, Component canvas)
   {
      String extraPanel = XMLReaderUtility.getMiddleString(0, xmlRepresentation, "<Extra Panels>", "</Extra Panels>");
      int numberOfViewports = Integer.parseInt(XMLReaderUtility.getMiddleString(0, xmlRepresentation, "<Number of ViewPorts>", "</Number of ViewPorts>"));

      String mainViewportPanels = XMLReaderUtility.getMiddleString(0, extraPanel, "<Main Viewport>", "</Main Viewport>");
      StringTokenizer tok = new StringTokenizer(mainViewportPanels, ",");
      String jPanelName = "";
      int size = tok.countTokens();
      for (int i = 0; i < size; i++)
      {
         jPanelName = tok.nextToken();
         myGUI.addPanelToTempHolderMainViewport(jPanelName);
         myGUI.makeCheckMarksConsistentForExtraPanels(jPanelName, true);
      }

      for (int i = 1; i < numberOfViewports; i++)
      {
         String viewportStartName = "<Viewport" + i + ">";
         String viewportEndName = "</Viewport" + i + ">";
         String viewport = XMLReaderUtility.getMiddleString(0, extraPanel, viewportStartName, viewportEndName);
         StringTokenizer viewportToken = new StringTokenizer(viewport, ",");
         jPanelName = "";
         size = viewportToken.countTokens();

         for (int j = 0; j < size; j++)
         {
            jPanelName = viewportToken.nextToken();
            myGUI.addPanelToTempHolderViewport(jPanelName, i);
            myGUI.makeCheckMarksConsistentForExtraPanels(jPanelName, true);
         }
      }
   }

   public void setupMainViewport(String xmlRepresentation)
   {
      boolean visible = myGUI.setViewportFromXMLDescription(xmlRepresentation);

      if (!visible)
      {
         myGUI.hideViewport();
      }
   }

   public void setupViewportWindows(String xmlRepresentation)
   {
      int numberofviewports = Integer.parseInt(XMLReaderUtility.getMiddleString(0, xmlRepresentation, "<Number of ViewPorts>", "</Number of ViewPorts>"));

      for (int i = 1; i < numberofviewports; i++)
      {
         CameraConfiguration config = new CameraConfiguration("viewport" + i);
         boolean maximizeWindow = false;
         String first = "<Viewport" + i + ">";
         String second = "</Viewport" + i + ">";
         String textToLoad = XMLReaderUtility.getMiddleString(0, xmlRepresentation, first, second);
         String visible = XMLReaderUtility.getMiddleString(0, textToLoad, "<Visible>", "</Visible>").trim();
         if (visible.equals("false"))
         {
            myGUI.createNewViewportWindow("viewport" + i, 1, maximizeWindow, config).hideViewport();
         }
         else
         {
            myGUI.createNewViewportWindow("viewport" + i, 1, maximizeWindow, config);
         }

         int canvasNumber = Integer.parseInt(XMLReaderUtility.getMiddleString(0, textToLoad, "<Canvas Number>", "</Canvas Number>"));

         String currentView = "Normal View";

         if (canvasNumber == 2)
         {
            currentView = "Split Screen";
         }

         if (canvasNumber == 3)
         {
            currentView = "Three Views";
         }

         if (canvasNumber == 4)
         {
            currentView = "Four Views";
         }

         myGUI.selectViewport_ViewPorts(currentView);
         myGUI.setupMultiViewsMultipleViewports(textToLoad, canvasNumber);

      }
   }

   public void setupConfiguration(String configurationName)
   {
      myGUI.setupConfiguration(configurationName, configurationName, configurationName);
      myGUI.selectGraphConfiguration(configurationName);
   }

   public void loadEntryBoxArrayPanel(String XMLStyleRepresentation, File file)
   {
      String XMLData = XMLReaderUtility.getMiddleString(0, XMLStyleRepresentation, "<Entry Boxes>", "</Entry Boxes>");
      int index = 0;
      int currentNumberOfVariables = 0;

      while ((XMLReaderUtility.getEndIndexOfSubString(index, XMLData, ",") <= XMLData.length())
             && (XMLReaderUtility.getEndIndexOfSubString(index, XMLData, ",") > 0))
      {
         currentNumberOfVariables++;
         index++;
      }

      if (index < XMLData.length())
         currentNumberOfVariables++;
      String[] name = new String[currentNumberOfVariables];

      currentNumberOfVariables = 0;
      index = 0;

      while ((XMLReaderUtility.getEndIndexOfSubString(index, XMLData, ",") <= XMLData.length())
             && (XMLReaderUtility.getEndIndexOfSubString(index, XMLData, ",") > 0))
      {
         int endIndex = XMLReaderUtility.getEndIndexOfSubString(index, XMLData, ",");

         name[currentNumberOfVariables] = XMLData.substring(index, endIndex - 1);
         name[currentNumberOfVariables] = name[currentNumberOfVariables].trim();
         currentNumberOfVariables++;
         index = endIndex;
      }

      if (index < XMLData.length())
      {
         name[currentNumberOfVariables] = XMLData.substring(index, XMLData.length());
         name[currentNumberOfVariables] = name[currentNumberOfVariables].trim();
      }

      sim.setupEntryBoxGroup(file.getName(), name);
      myGUI.updateGUI();
      myGUI.selectEntryBoxGroup(file.getName());
   }

   public void closeAndDispose()
   {
      dataFileFilter = null;
      dataFileChooser = null;
      frame = null;
      myGUI = null;
      sim = null;
   }
}

