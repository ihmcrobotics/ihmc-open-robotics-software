package us.ihmc.simulationconstructionsettools.dataExporter;

import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.io.File;

import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.tools.FormattingTools;
import us.ihmc.tools.thread.ThreadTools;

public class TorqueSpeedDataExporter implements ActionListener
{
   private final SimulationConstructionSet scs;
   private final Robot robot;
   private final String subdirectoryName;

   private DataExporterOptionsDialog optionsPanel;
   private final DataExporterReadmeWriter readmeWriter = new DataExporterReadmeWriter();
   private final TorqueSpeedDataExporterGraphCreator graphCreator;
   private final DataExporterExcelWorkbookCreator excelWorkbookCreator;

   private final Class<?> rootClassForDirectory;

   public TorqueSpeedDataExporter(SimulationConstructionSet scs, Robot robot)
   {
      this(scs, robot, robot.getClass(), robot.getName());
   }

   public TorqueSpeedDataExporter(SimulationConstructionSet scs, Robot robot, Class<?> rootClassForDirectory)
   {
      this(scs, robot, rootClassForDirectory, robot.getName());
   }

   public TorqueSpeedDataExporter(SimulationConstructionSet scs, Robot robot, Class<?> rootClassForDirectory, String subdirectoryName)
   {
      this.scs = scs;
      this.robot = robot;
      this.subdirectoryName = subdirectoryName;

      this.graphCreator = new TorqueSpeedDataExporterGraphCreator(robot, scs.getDataBuffer());
      this.excelWorkbookCreator = new DataExporterExcelWorkbookCreator(robot, scs.getDataBuffer());

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
      scs.gotoInPointNow();

      // confirm directory structure is correct
      if (simulationRootDirectory == null)
         return;
      File simulationDataAndVideoDirectory = DataExporterDirectoryFinder.findSimulationDataAndVideoRootLocation(simulationRootDirectory, subdirectoryName);
      if (simulationDataAndVideoDirectory == null)
         return;

      // create label
      String timeStamp = FormattingTools.getDateString() + "_" + FormattingTools.getTimeString();
      String tagName = timeStamp + "_" + robot.getName();

      // figure out svn revsision number for project
      long revisionNumber = -1;

      optionsPanel = new DataExporterOptionsDialog(tagName, simulationDataAndVideoDirectory.getAbsolutePath());

      if (!optionsPanel.isCancelled())
      {
         if (optionsPanel.saveData() || optionsPanel.createSpreadSheet() || optionsPanel.createGraphsJPG() || optionsPanel.createGraphsPDF()
                 || optionsPanel.createVideo() || optionsPanel.tagCode())
         {
            tagName = optionsPanel.tagName();
            System.out.println("Saving data using tag: " + tagName);

            // make destination directory
            File dataAndVideosTagDirectory = new File(simulationDataAndVideoDirectory, tagName);
            dataAndVideosTagDirectory.mkdir();

            // make graph directory inside destination directory
            File graphDirectory = new File(dataAndVideosTagDirectory, "graphs");
            graphDirectory.mkdir();

            if (optionsPanel.saveReadMe())
            {
               System.out.println("Saving ReadMe");
               readmeWriter.writeReadMe(dataAndVideosTagDirectory, tagName, revisionNumber);
               System.out.println("Done Saving ReadMe");
            }

            if (optionsPanel.saveData())
            {
               System.out.println("Saving data");
               saveDataFile(dataAndVideosTagDirectory, tagName);
               System.out.println("Done Saving Data");
            }

            if (optionsPanel.createSpreadSheet())
            {
               System.out.println("creating torque and speed spreadsheet");
               excelWorkbookCreator.createAndSaveTorqueAndSpeedSpreadSheet(dataAndVideosTagDirectory, tagName);
               System.out.println("done creating torque and speed spreadsheet");
            }

            if (optionsPanel.createGraphsJPG() || optionsPanel.createGraphsPDF())
            {
               System.out.println("creating torque and speed graphs");
               graphCreator.createJointTorqueSpeedGraphs(graphDirectory, tagName, optionsPanel.createGraphsJPG(), optionsPanel.createGraphsPDF());
               System.out.println("done creating torque and speed graphs");
            }

            if (optionsPanel.createVideo())
            {
               System.out.println("creating video");
               createVideo(dataAndVideosTagDirectory, tagName);
               System.out.println("done creating video");
            }

         }
      }
      else
      {
         System.out.println("Data export cancelled.");
      }

      scs.enableGUIComponents();
   }

   private void saveDataFile(File directory, String fileHeader)
   {
      File file = new File(directory, fileHeader + ".data.gz");
      scs.writeData(file);
   }

   /**
    * Create video from current viewport using the file path and file header
    * @param dataAndVideosTagDirectory
    * @param fileHeader
    */
   private void createVideo(File dataAndVideosTagDirectory, String fileHeader)
   {
      File video = new File(dataAndVideosTagDirectory, fileHeader + "_Video.mov");
      scs.getStandardSimulationGUI().getViewportPanel().getStandardGUIActions().createVideo(video);
   }
}
