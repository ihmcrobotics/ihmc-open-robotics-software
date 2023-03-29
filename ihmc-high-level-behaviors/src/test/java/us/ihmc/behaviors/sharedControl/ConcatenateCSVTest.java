package us.ihmc.behaviors.sharedControl;

import us.ihmc.log.LogTools;
import us.ihmc.promp.ProMPUtil;
import us.ihmc.rdx.ui.tools.TrajectoryRecordReplay;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertTrue;

import java.io.File;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;

public class ConcatenateCSVTest
{
   @Test
   public void testConcatenateCSV()
   {
      try
      {
         String etcDirectory = ProMPUtil.getEtcDirectory().toString();
         String demoTrainingDirectory = etcDirectory + "/test/PushDoorLeftRightHand";
         String recorderPath = Paths.get(etcDirectory + "/test").toAbsolutePath().toString();
         // Create a file object
         File demoFolder = new File(demoTrainingDirectory);

         // Get all the names of the files present in the given directory
         File[] files = demoFolder.listFiles();
         LogTools.info("File paths are:");
         assertTrue(files.length > 0);
         int numberOfParts = 2;
         // Concatenate data of pairs of lines
         for (int i = 0; i < files.length; i++)
         {
            String csvFilePath = demoTrainingDirectory + "/" + files[i].getName();
            TrajectoryRecordReplay trajectoryRecorder = new TrajectoryRecordReplay(csvFilePath, numberOfParts);
            LogTools.info(trajectoryRecorder.getPath());
            trajectoryRecorder.readCSV();
            ArrayList<double[]> data = trajectoryRecorder.getData();
            assertTrue(data.size() > 0); // check data is not empty
            LogTools.info(data.size());
            assertTrue(data.get(0).length == 7); // check columns are 7dofs (orientation + position) = frame pose
            trajectoryRecorder.concatenateData();
            trajectoryRecorder.setPath(recorderPath, false); // false = do not reset trajectoryRecorder after changing path
            trajectoryRecorder.writeCSV(trajectoryRecorder.getConcatenatedData());

            // try to open saved file and check if data has been concatenated successfully
            TrajectoryRecordReplay trajectoryReplay = new TrajectoryRecordReplay(recorderPath + "/" + trajectoryRecorder.getRecordFileName(),
                                                                                 numberOfParts);
            LogTools.info(trajectoryRecorder.getRecordFileName());
            trajectoryReplay.readCSV();
            ArrayList<double[]> dataConcatenated = trajectoryReplay.getData();
            assertTrue(dataConcatenated.size() > 0); // check data is not empty
            assertTrue(dataConcatenated.size() == data.size() / 2); // check number of rows is half
            assertTrue(dataConcatenated.get(0).length == 7 * numberOfParts); // check number of columns is double
            Thread.sleep(1000);
         }
      }
      catch (Exception e)
      {
         LogTools.error(e.getMessage());
      }
   }
}
