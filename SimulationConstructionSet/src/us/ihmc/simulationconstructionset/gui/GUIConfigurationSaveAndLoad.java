package us.ihmc.simulationconstructionset.gui;

import java.awt.Dimension;
import java.awt.Point;
import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.io.Writer;
import java.util.ArrayList;
import java.util.StringTokenizer;

import javax.swing.JFileChooser;

import us.ihmc.jMonkeyEngineToolkit.camera.CameraConfiguration;
import us.ihmc.simulationconstructionset.GraphConfiguration;
import us.ihmc.simulationconstructionset.gui.dialogConstructors.GUIEnablerAndDisabler;
import us.ihmc.tools.io.xml.XMLReaderUtility;

public class GUIConfigurationSaveAndLoad
{
   private JFileChooser dataFileChooser;
   private StandardSimulationGUI myGUI;

   private boolean loadGraphGroups = true;
   private boolean loadEntryBoxes = true;
   private boolean loadViewPorts = false;
   private boolean loadGraphWindows = true;
   private boolean loadJPanels = false;
   private boolean loadMultiViews = false;

   public GUIConfigurationSaveAndLoad(GUIEnablerAndDisabler guiEnablerAndDisabler, StandardSimulationGUI myGUI)
   {
      super();
      this.myGUI = myGUI;
   }

   public void setCurrentDirectory(File dir)
   {
      dataFileChooser.setCurrentDirectory(dir);
   }

   public void setCurrentDirectory(String dir)
   {
      dataFileChooser.setCurrentDirectory(new File(dir));
   }

   public void defaultSave(String fileName)
   {

      File guiConfig = new File(fileName);

      if (guiConfig.exists())
      {
         guiConfig.delete();
      }

      Writer output = null;
      guiConfig = new File(fileName);

      try
      {
         output = new BufferedWriter(new FileWriter(guiConfig));
         output.write(writeGUIConfig());
         output.close();
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }

   }

   private String writeGUIConfig()
   {
      String textToWrite = myGUI.getXMLStyleRepresentationOfGraphArrayPanel();
      textToWrite += "\n" + myGUI.getXMLStyleRepresentationOfEntryBoxes();
      textToWrite += "\n" + myGUI.getXMLStyleRepresentationOfViewPorts();
      textToWrite += "\n" + myGUI.getXMLStyleRepresentationOfGraphWindows();
      textToWrite += "\n" + myGUI.getXMLStyleRepresentationofJPanels();
      textToWrite += "\n" + myGUI.getXMLStyleRepresentationofMultiViews();

      return textToWrite;
   }

   public void saveNormalGUIConfiguration()
   {
      String path = getConfigurationDirectoryPath();

      File NormalConfig = new File(path + "/" + "NormalConfiguration.guiConf");
      if (!NormalConfig.exists())
      {
         Writer output = null;

         try
         {
            output = new BufferedWriter(new FileWriter(NormalConfig));
            output.write(writeGUIConfig());
            output.close();
         }
         catch (IOException e)
         {
            e.printStackTrace();
         }
      }
   }

   public void defaultGraphConfigurationLoad(String fileName)
   {
      String path = getConfigurationDirectoryPath();

      try
      {
         File guiConfig = new File(path + "/" + fileName + ".graphConf");

         if (guiConfig.exists())
         {
            BufferedReader reader = new BufferedReader(new FileReader(guiConfig));
            String xmlRepresentation = "";
            String tempLine;
            String fileEnding = ".graphConf";
            String name = guiConfig.getName().substring(0, guiConfig.getName().length() - fileEnding.length());

            while ((tempLine = reader.readLine()) != null)
            {
               xmlRepresentation += tempLine;
            }

            setupGraphGroups(myGUI, xmlRepresentation, name);

            reader.close();
         }
      }
      catch (Exception e)
      {
         e.printStackTrace();
      }
   }

   public void loadGUIConfiguration(String fileName)
   {
      if (fileName == null)
         return;

      try
      {
         File guiConfig = new File(fileName);

         if (guiConfig.exists())
         {
            BufferedReader reader = new BufferedReader(new FileReader(guiConfig));
            String xmlRepresentation = "";
            String tempLine;
            String fileEnding = ".dat";
            String name = guiConfig.getName().substring(0, guiConfig.getName().length() - fileEnding.length());

            while ((tempLine = reader.readLine()) != null)
            {
               xmlRepresentation += tempLine;
            }

            if (loadGraphGroups)
               setupGraphGroups(myGUI, xmlRepresentation, name);

            if (loadEntryBoxes)
               loadEntryBoxArrayTabbedPanel(xmlRepresentation);

            if (loadViewPorts)
            {
               setupMainViewport(xmlRepresentation);
               setupViewportWindows(xmlRepresentation);
            }

            if (loadGraphWindows)
            {
               setupGraphWindows(xmlRepresentation, "GraphWindows" + name);
               setupConfiguration(name);
            }

            if (loadMultiViews)
            {
               setupMultiViews(xmlRepresentation);
            }

            if (loadJPanels)
            {
               setupJPanels(xmlRepresentation);
            }

            myGUI.makeCheckBoxesConsistentWithCamera();

            reader.close();
         }
      }
      catch (Exception e)
      {
         e.printStackTrace();
      }
   }

   public void setupMultiViews(String xmlRepresentation)
   {
      myGUI.addViewportPanelToMainPanel(); //TODO: Why is this here? 
      String currentView = XMLReaderUtility.getMiddleString(0, xmlRepresentation, "<Current View>", "</Current View>");
      myGUI.selectViewport(currentView);
      myGUI.setupMultiViews(xmlRepresentation, currentView);
   }

   public void setupJPanels(String xmlRepresentation)
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
      int numberOfViewports = Integer.parseInt(XMLReaderUtility.getMiddleString(0, xmlRepresentation, "<Number of ViewPorts>", "</Number of ViewPorts>"));

      ArrayList<ViewportWindow> windows = myGUI.getViewportWindows();

      for (int i = windows.size() - 1; i >= 0; i--)
      {
         windows.get(i).closeWindow();
         windows.remove(i);
      }

      for (int i = 1; i < numberOfViewports; i++)
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

   public static void setupGraphGroups(StandardSimulationGUI myGUI, String xmlRepresentation, String name)
   {
      try
      {
         String graphGroupString = XMLReaderUtility.getMiddleString(0, xmlRepresentation, "<GraphGroup>", "</GraphGroup>");
         int numColumns = Integer.parseInt(XMLReaderUtility.getMiddleString(0, graphGroupString, "<Cols>", "</Cols>"));
         String graphString = "";
         int currentgraphIndex = 0;
         String[] strArray = graphGroupString.split("<Graph>");
         int numberOfGraphs = strArray.length - 1;
         GraphConfiguration[] graphConfigurations = new GraphConfiguration[numberOfGraphs];
         String[][][] var = new String[numberOfGraphs][][];

         for (int graphIndex = 0; graphIndex < numberOfGraphs; graphIndex++)
         {
            graphString = XMLReaderUtility.getMiddleString(currentgraphIndex, graphGroupString, "<Graph>", "</Graph>");
            currentgraphIndex = XMLReaderUtility.getEndIndexOfSubString(currentgraphIndex, graphGroupString, "</Graph>");

            String variables = XMLReaderUtility.getMiddleString(0, graphString, "<Variables>", "</Variables>");
            StringTokenizer tokenizer = new StringTokenizer(variables, ",");
            int numberOfTokens = tokenizer.countTokens();
            GraphConfiguration tmpGraphConfiguration = GraphConfiguration.createClassBasedOnXMLRepresentation(0, graphString);

            graphConfigurations[graphIndex] = tmpGraphConfiguration;

            String[] graphConfigurationStringArray = { tmpGraphConfiguration.getName() };

            var[graphIndex] = new String[2][numberOfTokens];

            for (int i = 0; i < numberOfTokens; i++)
            {
               var[graphIndex][0][i] = tokenizer.nextToken().trim();
            }

            var[graphIndex][1] = graphConfigurationStringArray;
         }

         myGUI.setupGraphConfigurations(graphConfigurations);
         myGUI.setupGraphGroup(name, var, numColumns);
      }
      catch (Exception e)
      {
         e.printStackTrace();
      }
   }

   public void setupGraphWindows(String xmlRepresentation, String name)
   {
      removeOldWindows();
      setupGraphWindows(myGUI, xmlRepresentation, name);
   }

   public static void setupGraphWindows(StandardSimulationGUI myGUI, String xmlRepresentation, String name)
   {
      int graphArrayWindowSize = Integer.parseInt(XMLReaderUtility.getMiddleString(0, xmlRepresentation, "<Graph Array Window Size>",
            "</Graph Array Window Size>"));
      for (int windowNumber = 1; windowNumber <= graphArrayWindowSize; windowNumber++)
      {
         String first = "<Graph Array Window" + windowNumber + ">";
         String second = "</Graph Array Window" + windowNumber + ">";
         String graphWindowString = XMLReaderUtility.getMiddleString(0, xmlRepresentation, first, second);

         first = "<ScreenID>";
         second = "</ScreenID>";
         String screenIDString = XMLReaderUtility.getMiddleString(0, graphWindowString, first, second);

         int screenID = 1;
         if (screenIDString != null)
            screenID = Integer.parseInt(screenIDString);

         first = "<WindowLocation>";
         second = "</WindowLocation>";
         String windowLocationString = XMLReaderUtility.getMiddleString(0, graphWindowString, first, second);

         Point windowLocation = null;
         if (windowLocationString != null)
         {
            StringTokenizer tokenizer = new StringTokenizer(windowLocationString, ",");
            double xPosition = Double.parseDouble(tokenizer.nextToken());
            double yPosition = Double.parseDouble(tokenizer.nextToken());
            windowLocation = new Point((int) xPosition, (int) yPosition);
         }

         first = "<WindowSize>";
         second = "</WindowSize>";
         String windowSizeString = XMLReaderUtility.getMiddleString(0, graphWindowString, first, second);

         Dimension windowSize = null;
         if (windowSizeString != null)
         {
            StringTokenizer tokenizer = new StringTokenizer(windowSizeString, ",");
            double width = Double.parseDouble(tokenizer.nextToken());
            double height = Double.parseDouble(tokenizer.nextToken());
            windowSize = new Dimension((int) width, (int) height);
         }

         setupGraphGroups(myGUI, graphWindowString, name + windowNumber);
         myGUI.createNewGraphWindow(name + windowNumber, screenID, windowLocation, windowSize, false);
      }

   }

   private void removeOldWindows()
   {
      ArrayList<GraphArrayWindow> windows = myGUI.getGraphArrayWindows();

      for (int i = windows.size() - 1; i >= 0; i--)
      {
         windows.get(i).closeWindow();
         windows.remove(i);
      }
   }

   public void setupConfiguration(String configurationName)
   {
      myGUI.setupConfiguration(configurationName, configurationName, configurationName);
      myGUI.selectGraphConfiguration(configurationName);
   }

   public void loadEntryBoxArrayTabbedPanel(String XMLStyleRepresentation)
   {
      String XMLData = XMLReaderUtility.getMiddleString(0, XMLStyleRepresentation, "<Entry Boxes Tab Pane>", "</Entry Boxes Tab Pane>");

      if(XMLData == null)
    	  return;
      
      int index = 0;
      while ((XMLReaderUtility.getEndIndexOfSubString(index, XMLData, "</EntryBoxTab>") <= XMLData.length())
            && (XMLReaderUtility.getEndIndexOfSubString(index, XMLData, "</EntryBoxTab>") > 0))
      {
         int endIndex = XMLReaderUtility.getEndIndexOfSubString(index, XMLData, "</EntryBoxTab>");

         String currentXMLEntryTab = XMLData.substring(index, endIndex - 1);

         String name = XMLReaderUtility.getMiddleString(0, currentXMLEntryTab, "<Title>", "</Title>");

         
         String currentXMLEntry = XMLReaderUtility.getMiddleString(0, currentXMLEntryTab, "<Entry Boxes>", "</Entry Boxes>");

         
         loadEntryBoxArrayPanel(name, currentXMLEntry);
         index = endIndex;
      }

   }
   public void loadEntryBoxArrayPanel(String tabName, String XMLData)
   {
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

      myGUI.setupEntryBoxGroup(tabName, name);
      myGUI.updateGUI();
      myGUI.createNewEntryBoxTabFromEntryBoxGroup(tabName);
   }

   public void loadGraphConfigurationsInConfigurationMenu()
   {
      String[] potentialConfigurationFilenames = getPotentialConfigurationFilenames();

      for (String child : potentialConfigurationFilenames)
      {
         if (!child.endsWith(".graphConf"))
            continue;

         int index = child.indexOf(".");
         String name = child.substring(0, index);

         defaultGraphConfigurationLoad(name);
      }
   }

   public static String getConfigurationDirectoryPath()
   {
      File configurationDirectory = GUIConfigurationSaveAndLoad.makeOrFindConfigurationDirectory();

      if (configurationDirectory.isDirectory())
      {
         return configurationDirectory.getPath();
      }

      return null;
   }

   public static String[] getPotentialConfigurationFilenames()
   {
      File configurationDirectory = makeOrFindConfigurationDirectory();

      if (configurationDirectory.isDirectory())
      {
         return configurationDirectory.list();
      }
      else
      {
         return new String[] {};
      }
   }

   public static File makeOrFindConfigurationDirectory()
   {
      File configurationDirectory = new File("Configurations");
      if (!configurationDirectory.exists())
      {
         configurationDirectory.mkdir();
      }
      return configurationDirectory;
   }

}
