package us.ihmc.simulationconstructionset.dataExporter;

import java.awt.Color;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.io.File;
import java.util.ArrayList;

import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.YoVariable;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.tools.thread.ThreadTools;
import us.ihmc.tools.time.DateTools;

public abstract class AbstractDataExporter implements ActionListener
{
   public abstract ArrayList<Color> getDataVsTimeColorsToCycle();

   public abstract ArrayList<Color> getDataVsDataColorsToCycle();

   public abstract ArrayList<String> getDataVsTimeVariableNames();

   public abstract ArrayList<String[]> getDataVsDataVariableNames();

   private final SimulationConstructionSet scs;
   private final String name;
   private final String subdirectoryName;

   //   private DataExporterOptionsDialog optionsPanel;
   private final DataExporterReadmeWriter readmeWriter = new DataExporterReadmeWriter();
   private final DataExporterGraphCreator graphCreator;
   //   private final DataExporterExcelWorkbookCreator excelWorkbookCreator;

   private final Class<?> rootClassForDirectory;

   public AbstractDataExporter(SimulationConstructionSet scs, String name, DoubleYoVariable timeYoVariable, Class<?> rootClassForDirectory,
         String subdirectoryName)
   {
      this.scs = scs;
      this.name = name;
      this.subdirectoryName = subdirectoryName;

      this.graphCreator = new DataExporterGraphCreator(timeYoVariable, scs.getDataBuffer());
      //      this.excelWorkbookCreator = new DataExporterExcelWorkbookCreator(robot, scs.getDataBuffer());

      this.rootClassForDirectory = rootClassForDirectory;
   }

   @Override
   public void actionPerformed(ActionEvent e)
   {
      File simulationRootDirectory = DataExporterDirectoryFinder.findSimulationRootLocation(rootClassForDirectory);

      // Stop the sim and disable the GUI:
      scs.stop();
      scs.disableGUIComponents();

      // Wait till done running:
      while (scs.isSimulating())
      {
         ThreadTools.sleep(1000);
      }

      // Crop the Buffer to In/Out. This is important because of how we use the DataBuffer later and we assume that in point is at index=0:
      scs.cropBuffer();

      // For some reason external YoVariable Visualizers freeze up if we do scs.gotoInPointNow(). Not sure why.
      //      scs.gotoInPointNow();

      // confirm directory structure is correct
      if (simulationRootDirectory == null)
         return;
      File simulationDataAndVideoDirectory = DataExporterDirectoryFinder.findSimulationDataAndVideoRootLocation(simulationRootDirectory, subdirectoryName);
      if (simulationDataAndVideoDirectory == null)
         return;

      // create label
      String timeStamp = DateTools.getDateString() + "_" + DateTools.getTimeString();
      String tagName = timeStamp + "_" + name;

      //      optionsPanel = new DataExporterOptionsDialog(tagName, simulationDataAndVideoDirectory.getAbsolutePath());

      //      if (!optionsPanel.isCancelled())
      //      {
      //         if (optionsPanel.saveData() || optionsPanel.createSpreadSheet() || optionsPanel.createGraphsJPG() || optionsPanel.createGraphsPDF()
      //                 || optionsPanel.createVideo() || optionsPanel.tagCode())
      //         {
      //            tagName = optionsPanel.tagName();
      System.out.println("Saving data using tag: " + tagName);

      // make destination directory
      File dataAndVideosTagDirectory = new File(simulationDataAndVideoDirectory, tagName);
      dataAndVideosTagDirectory.mkdir();

      // make graph directory inside destination directory
      File graphDirectory = new File(dataAndVideosTagDirectory, "graphs");
      graphDirectory.mkdir();

      //            if (optionsPanel.saveReadMe())
      //            {
      System.out.println("Saving ReadMe");
      readmeWriter.writeReadMe(dataAndVideosTagDirectory, tagName, -1);
      System.out.println("Done Saving ReadMe");
      //            }

      //            if (optionsPanel.saveData())
      {
         System.out.println("Saving data");
         saveDataFile(dataAndVideosTagDirectory, tagName);
         System.out.println("Done Saving Data");
      }

      //            if (optionsPanel.createSpreadSheet())
      //            {
      //               System.out.println("creating torque and speed spreadsheet");
      //               excelWorkbookCreator.createAndSaveTorqueAndSpeedSpreadSheet(dataAndVideosTagDirectory, tagName);
      //               System.out.println("done creating torque and speed spreadsheet");
      //            }

      //            if (optionsPanel.createGraphsJPG() || optionsPanel.createGraphsPDF())
      {
         System.out.println("creating data graphs");
         boolean createGraphsJPG = true;
         boolean createGraphsPDF = false;

         ArrayList<String> dataVsTimeVariableNames = getDataVsTimeVariableNames();
         ArrayList<Color> dataVsTimeColors = getDataVsTimeColorsToCycle();

         int colorIndex = 0;
         for (String variableName : dataVsTimeVariableNames)
         {
            YoVariable<?> variable = scs.getVariable(variableName);
            if (variable == null)
            {
               System.err.println("Couldn't find variable named " + variableName);
               continue;
            }
            graphCreator.createDataVsTimeGraph(graphDirectory, tagName, variable, createGraphsJPG, createGraphsPDF, dataVsTimeColors.get(colorIndex));
            colorIndex++;
            if (colorIndex >= dataVsTimeColors.size())
               colorIndex = 0;
         }

         ArrayList<String[]> dataVsDataVariableNames = getDataVsDataVariableNames();
         ArrayList<Color> dataVDataColors = getDataVsDataColorsToCycle();

         colorIndex = 0;
         for (String[] variableNames : dataVsDataVariableNames)
         {
            if (variableNames.length != 2)
            {
               System.err.println(getClass().getSimpleName() + ": Must have exactly 2 variable names in getDataVsDataVariableNames()");
               continue;
            }

            YoVariable<?> variableOne = scs.getVariable(variableNames[0]);
            YoVariable<?> variableTwo = scs.getVariable(variableNames[1]);

            if (variableOne == null)
            {
               System.err.println("Couldn't find variable named " + variableNames[0]);
               continue;
            }

            if (variableTwo == null)
            {
               System.err.println("Couldn't find variable named " + variableNames[1]);
               continue;
            }

            graphCreator.createDataOneVsDataTwoGraph(graphDirectory, tagName, variableOne, variableTwo, createGraphsJPG, createGraphsPDF,
                  dataVDataColors.get(colorIndex));
            colorIndex++;
            if (colorIndex >= dataVsTimeColors.size())
               colorIndex = 0;
         }

         System.out.println("done creating variable graphs");
      }

      //            if (optionsPanel.createVideo())
      //            {
      //               System.out.println("creating video");
      //               createVideo(dataAndVideosTagDirectory, tagName);
      //               System.out.println("done creating video");
      //            }

      //         }
      //      }
      //      else
      //      {
      //         System.out.println("Data export cancelled.");
      //      }

      scs.enableGUIComponents();
   }

   private void saveDataFile(File directory, String fileHeader)
   {
      File file = new File(directory, fileHeader + ".data.gz");
      scs.writeData(file);
   }

   //   /**
   //    * Create video from current viewport using the file path and file header
   //    * @param dataAndVideosTagDirectory
   //    * @param fileHeader
   //    */
   //   private void createVideo(File dataAndVideosTagDirectory, String fileHeader)
   //   {
   //      File video = new File(dataAndVideosTagDirectory, fileHeader + "_Video.mov");
   //      scs.getStandardSimulationGUI().getViewportPanel().getStandardGUIActions().createVideo(video);
   //   }
}
