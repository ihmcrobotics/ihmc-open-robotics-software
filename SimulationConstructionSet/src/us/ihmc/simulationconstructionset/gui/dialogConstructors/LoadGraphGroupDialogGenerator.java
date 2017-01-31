package us.ihmc.simulationconstructionset.gui.dialogConstructors;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.util.StringTokenizer;

import javax.swing.JFileChooser;
import javax.swing.JFrame;

import us.ihmc.simulationconstructionset.GraphConfiguration;
import us.ihmc.simulationconstructionset.gui.GraphArrayPanel;
import us.ihmc.simulationconstructionset.gui.StandardSimulationGUI;
import us.ihmc.simulationconstructionset.gui.config.GraphGroupSelector;
import us.ihmc.tools.gui.MyFileFilter;
import us.ihmc.tools.io.xml.XMLReaderUtility;

public class LoadGraphGroupDialogGenerator implements LoadGraphGroupDialogConstructor
{
   private javax.swing.filechooser.FileFilter dataFileFilter = new MyFileFilter(new String[] {".graphConf"}, "GraphGroup Configuration (.graphConf)");
   private JFileChooser dataFileChooser;
   private JFrame frame;
   private GUIEnablerAndDisabler guiEnablerAndDisabler;
   private StandardSimulationGUI myGUI;
   private GraphGroupSelector graphGroupSelector;


   public LoadGraphGroupDialogGenerator(GUIEnablerAndDisabler guiEnablerAndDisabler, StandardSimulationGUI myGUI, GraphGroupSelector graphGroupSelector, JFrame frame, GraphArrayPanel graphArrayPanel)
   {
      this.graphGroupSelector = graphGroupSelector;
      this.frame = frame;
      this.guiEnablerAndDisabler = guiEnablerAndDisabler;
      this.myGUI = myGUI;

      try
      {
         File Configs = new File("Configurations");

         String path = Configs.toURI().getPath();
         dataFileChooser = new JFileChooser();

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
      guiEnablerAndDisabler.disableGUIComponents();


      if (dataFileChooser.showOpenDialog(frame) == JFileChooser.APPROVE_OPTION)
      {
         File chosenFile = dataFileChooser.getSelectedFile();

         loadGraphGroupFile(chosenFile);
      }

      guiEnablerAndDisabler.enableGUIComponents();
   }

   @Override
   public void loadGraphGroupFile(File file)
   {
      String fileEnding = ".graphConf";

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

         setupGraphGroups(xmlRepresentation, name);

         reader.close();
         System.out.println("Your file has been loaded.");
      }
      catch (Exception e)
      {
         e.printStackTrace();
      }
   }

   public void setupGraphGroups(String xmlRepresentation, String name)
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

            String[] graphConfigurationStringArray = {tmpGraphConfiguration.getName()};

            var[graphIndex] = new String[2][numberOfTokens];

            for (int i = 0; i < numberOfTokens; i++)
            {
               var[graphIndex][0][i] = tokenizer.nextToken().trim();
            }

            var[graphIndex][1] = graphConfigurationStringArray;
         }

         myGUI.setupGraphConfigurations(graphConfigurations);
         myGUI.setupGraphGroup(name, var, numColumns);
         graphGroupSelector.selectGraphGroup(name);
      }
      catch (Exception e)
      {
         e.printStackTrace();
      }
   }

   @Override
   public void closeAndDispose()
   {      
       dataFileFilter = null;
       dataFileChooser = null;
       frame = null;
       guiEnablerAndDisabler = null;
       myGUI = null;
       graphGroupSelector = null;
   }

}
