package us.ihmc.simulationconstructionset.gui.dialogConstructors;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.net.URL;

import javax.swing.JFileChooser;
import javax.swing.JFrame;

import us.ihmc.jMonkeyEngineToolkit.camera.CameraConfiguration;
import us.ihmc.simulationconstructionset.GroundContactModel;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.gui.StandardSimulationGUI;
import us.ihmc.simulationconstructionset.robotdefinition.RobotDefinitionFixedFrame;
import us.ihmc.tools.gui.MyFileFilter;
import us.ihmc.tools.io.xml.XMLReaderUtility;

public class LoadRobotConfigurationDialogGenerator implements LoadRobotConfigurationDialogConstructor
{
   private javax.swing.filechooser.FileFilter dataFileFilter = new MyFileFilter(new String[] {".robotConf"}, "Robot Configuration (.robotConf)");
   private File chosenFile;
   private JFileChooser dataFileChooser;
   private JFrame frame;
   private StandardSimulationGUI myGUI;
   private SimulationConstructionSet sim;

   public LoadRobotConfigurationDialogGenerator(SimulationConstructionSet sim, JFrame frame, StandardSimulationGUI myGUI)
   {
      this.frame = frame;
      this.sim = sim;
      this.myGUI = myGUI;

      try
      {
         dataFileChooser = new JFileChooser();

         if (sim != null)
         {
            URL defaultDirURL = sim.getClass().getResource(".");

            if (defaultDirURL != null)
            {
               String defaultDirString = defaultDirURL.getPath();

               if (defaultDirString != null)
               {
                  int idx = defaultDirString.indexOf("classes");

                  if (idx > 0)
                  {
                     defaultDirString = defaultDirString.substring(0, idx);
                  }

                  setCurrentDirectory(defaultDirString);
               }
            }
         }

         dataFileChooser.setAcceptAllFileFilterUsed(false);
         dataFileChooser.addChoosableFileFilter(dataFileFilter);
      }
      catch (Exception e)
      {
         // e.printStackTrace();
      }
   }

   public void setCurrentDirectory(File dir)
   {
      dataFileChooser.setCurrentDirectory(dir);
   }

   public void setCurrentDirectory(String dir)
   {
      dataFileChooser.setCurrentDirectory(new File(dir));
   }

   public void constructDialog()
   {
      sim.disableGUIComponents();

      // Determine what the file ending should be...
      String fileEnding;

      fileEnding = ".robotConf";

      if (dataFileChooser.showOpenDialog(frame) == JFileChooser.APPROVE_OPTION)
      {
         try
         {
            chosenFile = dataFileChooser.getSelectedFile();

            String filename = chosenFile.getName();

            if (!filename.endsWith(fileEnding))
            {
               filename = filename.concat(fileEnding);

               if (!chosenFile.getName().equals(filename))
               {
                  File newChosenFile = new File(chosenFile.getParent(), filename);

                  chosenFile = newChosenFile;
               }
            }

            String name = chosenFile.getName().substring(0, chosenFile.getName().length() - fileEnding.length());

            System.out.println("Loading: " + name);

            BufferedReader reader = new BufferedReader(new FileReader(chosenFile));
            String xmlRepresentation = "";
            String tempLine;

            while ((tempLine = reader.readLine()) != null)
            {
               xmlRepresentation += tempLine;
            }

            GroundContactModel gcm = null;

            Robot[] robots = sim.getRobots();
            if (robots.length > 0)
            {
               gcm = robots[0].getGroundContactModel();
            }


            RobotDefinitionFixedFrame robotDef = new RobotDefinitionFixedFrame();
            robotDef.createRobotDefinitionFromRobotConfigurationString(xmlRepresentation);

//          System.out.println(robotDef.toString());
            Robot r = new Robot(robotDef, "Loaded");
            r.setGroundContactModel(gcm);
            sim.setRobot(r);

            reader.close();
            System.out.println("Your file has been loaded.");
         }
         catch (Exception e)
         {
            e.printStackTrace();
         }
      }

      sim.enableGUIComponents();
   }

   public void setupMainViewport(String xmlRepresentation)
   {
      boolean visible = myGUI.setViewportFromXMLDescription(xmlRepresentation);

      if (!visible)    // for main viewport
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
         boolean maximizeWindow = true;
         String first = "<Viewport" + i + ">";
         String second = "</Viewport" + i + ">";
         String textToLoad = XMLReaderUtility.getMiddleString(0, xmlRepresentation, first, second);
         String visible = XMLReaderUtility.getMiddleString(0, textToLoad, "<Visible>", "</Visible>").trim();
         double posX = Double.parseDouble(XMLReaderUtility.getMiddleString(0, textToLoad, "<Viewport Camera X>", "</Viewport Camera X>"));
         double posY = Double.parseDouble(XMLReaderUtility.getMiddleString(0, textToLoad, "<Viewport Camera Y>", "</Viewport Camera Y>"));
         double posZ = Double.parseDouble(XMLReaderUtility.getMiddleString(0, textToLoad, "<Viewport Camera Z>", "</Viewport Camera Z>"));

         config.setCameraPosition(posX, posY, posZ);
         String Dolly = XMLReaderUtility.getMiddleString(0, textToLoad, "<Dolly data>", "</Dolly data>");
         double DollyX = Double.parseDouble(XMLReaderUtility.getMiddleString(0, Dolly, "<Position X>", "</Position X>"));
         double DollyY = Double.parseDouble(XMLReaderUtility.getMiddleString(0, Dolly, "<Position Y>", "</Position Y>"));
         double DollyZ = Double.parseDouble(XMLReaderUtility.getMiddleString(0, Dolly, "<Position Z>", "</Position Z>"));

         config.setCameraDollyOffsets(DollyX, DollyY, DollyZ);
         String Dolly_Boolean = XMLReaderUtility.getMiddleString(0, Dolly, "<Dolly>", "</Dolly>");
         String Dolly_Boolean_X = XMLReaderUtility.getMiddleString(0, Dolly, "<Dolly X>", "</Dolly X>");
         String Dolly_Boolean_Y = XMLReaderUtility.getMiddleString(0, Dolly, "<Dolly Y>", "</Dolly Y>");
         String Dolly_Boolean_Z = XMLReaderUtility.getMiddleString(0, Dolly, "<Dolly Z>", "</Dolly Z>");
         boolean dolly_set = true;
         boolean dolly_setX = true;
         boolean dolly_setY = true;
         boolean dolly_setZ = true;

         if (Dolly_Boolean.equals("false"))
         {
            dolly_set = false;
         }

         if (Dolly_Boolean_X.equals("false"))
         {
            dolly_setX = false;
         }

         if (Dolly_Boolean_Y.equals("false"))
         {
            dolly_setY = false;
         }

         if (Dolly_Boolean_Z.equals("false"))
         {
            dolly_setZ = false;
         }

         config.setCameraDolly(dolly_set, dolly_setX, dolly_setY, dolly_setZ);
         String Track = XMLReaderUtility.getMiddleString(0, textToLoad, "<Track data>", "</Track data>");
         double TrackX = Double.parseDouble(XMLReaderUtility.getMiddleString(0, Track, "<Position X>", "</Position X>"));
         double TrackY = Double.parseDouble(XMLReaderUtility.getMiddleString(0, Track, "<Position Y>", "</Position Y>"));
         double TrackZ = Double.parseDouble(XMLReaderUtility.getMiddleString(0, Track, "<Position Z>", "</Position Z>"));

         config.setCameraTrackingOffsets(TrackX, TrackY, TrackZ);

         String Track_Boolean = XMLReaderUtility.getMiddleString(0, Track, "<Track>", "</Track>");
         String Track_Boolean_X = XMLReaderUtility.getMiddleString(0, Track, "<Track X>", "</Track X>");
         String Track_Boolean_Y = XMLReaderUtility.getMiddleString(0, Track, "<Track Y>", "</Track Y>");
         String Track_Boolean_Z = XMLReaderUtility.getMiddleString(0, Track, "<Track Z>", "</Track Z>");
         boolean track_set = true;
         boolean track_setX = true;
         boolean track_setY = true;
         boolean track_setZ = true;

         if (Track_Boolean.equals("false"))
         {
            track_set = false;
         }

         if (Track_Boolean_X.equals("false"))
         {
            track_setX = false;
         }

         if (Track_Boolean_Y.equals("false"))
         {
            track_setY = false;
         }

         if (Track_Boolean_Z.equals("false"))
         {
            track_setZ = false;
         }

         config.setCameraTracking(track_set, track_setX, track_setY, track_setZ);
         String Fix = XMLReaderUtility.getMiddleString(0, textToLoad, "<Fix Position>", "</Fix Position>");
         double FixX = Double.parseDouble(XMLReaderUtility.getMiddleString(0, Fix, "<Fix X>", "</Fix X>"));
         double FixY = Double.parseDouble(XMLReaderUtility.getMiddleString(0, Fix, "<Fix Y>", "</Fix Y>"));
         double FixZ = Double.parseDouble(XMLReaderUtility.getMiddleString(0, Fix, "<Fix Z>", "</Fix Z>"));

         config.setCameraFix(FixX, FixY, FixZ);

         if (visible.equals("false"))
         {
            myGUI.createNewViewportWindow("viewport" + i, 1, maximizeWindow, config).hideViewport();
         }
         else
         {
            myGUI.createNewViewportWindow("viewport" + i, 1, maximizeWindow, config);
         }

      }
   }

   public void setupConfiguration(String configurationName)
   {
      myGUI.setupConfiguration(configurationName, configurationName, configurationName);
      myGUI.selectGraphConfiguration(configurationName);
   }

   public void loadEntryBoxArrayPanel(String XMLStyleRepresentation)
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

      sim.setupEntryBoxGroup(chosenFile.getName(), name);
      myGUI.updateGUI();
      myGUI.selectEntryBoxGroup(chosenFile.getName());
   }
}
