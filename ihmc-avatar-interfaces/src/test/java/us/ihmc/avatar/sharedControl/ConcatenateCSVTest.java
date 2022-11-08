package us.ihmc.avatar.sharedControl;

import us.ihmc.rdx.ui.tools.TrajectoryRecordReplay;
import us.ihmc.tools.io.WorkspaceDirectory;

import java.io.File;

public class ConcatenateCSVTest
{
   public static void main(String args[])
   {
      try {
         WorkspaceDirectory directory = new WorkspaceDirectory("ihmc-open-robotics-software", "promp/etc");
         String directoryAbsolutePath = directory.getDirectoryPath().toAbsolutePath().toString();
         String demoTrainingDirectory = directoryAbsolutePath + "/test/PushDoorLeftRightHand";
         // Create a file object
         File demoFolder = new File(demoTrainingDirectory);

         // Get all the names of the files present
         // in the given directory
         File[] files = demoFolder.listFiles();

         System.out.println("File paths are:");

         // Display the names of the files
         for (int i = 0; i < files.length; i++) {
            String csvFilePath = demoTrainingDirectory + "/" + files[i].getName();
            TrajectoryRecordReplay<Double> trajectoryRecorder = new TrajectoryRecordReplay<>(Double.class, csvFilePath, 2);
            System.out.println(trajectoryRecorder.getPath());
            trajectoryRecorder.readCSV();
            trajectoryRecorder.concatenateData();
            trajectoryRecorder.setPath(directoryAbsolutePath+"/test",false); // false = do not reset trajectoryRecorder after changing path
            trajectoryRecorder.writeCSV(trajectoryRecorder.getConcatenatedData());
         }
      }
      catch (Exception e) {
         System.err.println(e.getMessage());
      }
   }
}
