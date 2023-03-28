package us.ihmc.behaviors.sharedControl;

import us.ihmc.rdx.ui.tools.TrajectoryRecordReplay;
import us.ihmc.tools.io.WorkspaceDirectory;
import org.junit.Test;
import static org.junit.Assert.*;

import java.io.File;
import java.util.ArrayList;

public class ConcatenateCSVTest
{
   @Test
   public void testConcatenateCSV()
   {
      try
      {
         WorkspaceDirectory directory = new WorkspaceDirectory("ihmc-open-robotics-software", "promp/etc");
         String directoryAbsolutePath = directory.getFilesystemDirectory().toAbsolutePath().toString();
         String demoTrainingDirectory = directoryAbsolutePath + "/test/PushDoorLeftRightHand";
         // Create a file object
         File demoFolder = new File(demoTrainingDirectory);

         // Get all the names of the files present in the given directory
         File[] files = demoFolder.listFiles();
         System.out.println("File paths are:");
         assertTrue(files.length > 0);
         int numberOfParts = 2;
         // Concatenate data of pairs of lines
         for (int i = 0; i < files.length; i++)
         {
            String csvFilePath = demoTrainingDirectory + "/" + files[i].getName();
            TrajectoryRecordReplay<Double> trajectoryRecorder = new TrajectoryRecordReplay<>(Double.class, csvFilePath, numberOfParts);
            System.out.println(trajectoryRecorder.getPath());
            trajectoryRecorder.readCSV();
            ArrayList<Double[]> data = trajectoryRecorder.getData();
            assertTrue(data.size() > 0); // check data is not empty
            System.out.println(data.size());
            assertTrue(data.get(0).length == 7); // check columns are 7dofs (orientation + position) = frame pose
            trajectoryRecorder.concatenateData();
            trajectoryRecorder.setPath(directoryAbsolutePath + "/test", false); // false = do not reset trajectoryRecorder after changing path
            trajectoryRecorder.writeCSV(trajectoryRecorder.getConcatenatedData());

            // try to open saved file and check if data has been concatenated successfully
            TrajectoryRecordReplay<Double> trajectoryReplay = new TrajectoryRecordReplay<>(Double.class,
                                                                                           directoryAbsolutePath + "/test/"
                                                                                           + trajectoryRecorder.getRecordFileName(),
                                                                                           numberOfParts);
            System.out.println(trajectoryReplay.getPath());
            trajectoryReplay.readCSV();
            ArrayList<Double[]> dataConcatenated = trajectoryReplay.getData();
            assertTrue(dataConcatenated.size() > 0); // check data is not empty
            assertTrue(dataConcatenated.size() == data.size() / 2); // check number of rows is half
            assertTrue(dataConcatenated.get(0).length == 7 * numberOfParts); // check number of columns is double
         }
      }
      catch (Exception e)
      {
         System.err.println(e.getMessage());
      }
   }
}
