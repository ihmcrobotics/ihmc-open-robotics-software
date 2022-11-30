package us.ihmc.behaviors.sharedControl;

import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.log.LogTools;
import us.ihmc.rdx.ui.tools.TrajectoryRecordReplay;
import us.ihmc.tools.io.WorkspaceDirectory;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.nio.file.StandardCopyOption;
import java.util.ArrayList;
import java.util.List;

public class ProMPAssistantTest
{
   public static void main(String[] args) throws IOException
   {
      // learn ProMPs
      // Check ProMPAssistant.json if you want to change parameters (e.g, task to learn, body parts to consider in the motion)
      ProMPAssistant proMPAssistant = new ProMPAssistant();
      // use a csv file with the trajectories of the hands of the robot for testing
      WorkspaceDirectory directory = new WorkspaceDirectory("ihmc-open-robotics-software", "promp/etc");
      String directoryAbsolutePath = directory.getDirectoryPath().toAbsolutePath().toString();
      String demoDirectory = directoryAbsolutePath + "/test/PushDoorTest";
      //get test number from config file
      String testFilePath = demoDirectory + "/" + proMPAssistant.getTestNumber() + ".csv";
      //copy test file to have it always under same name for faster plotting
      Path originalPath = Paths.get(testFilePath);
      Path copyForPlottingPath = Paths.get(demoDirectory + "/test.csv");
      Files.copy(originalPath, copyForPlottingPath, StandardCopyOption.REPLACE_EXISTING);

      // replay that file
      TrajectoryRecordReplay<Double> trajectoryPlayer = new TrajectoryRecordReplay<>(Double.class, testFilePath, 2); //2 body parts: the hands
      trajectoryPlayer.setDoneReplay(false);

      //let's focus on the hands
      List<String> bodyParts = new ArrayList<>();
      bodyParts.add("leftHand");
      bodyParts.add("rightHand");
      TrajectoryRecordReplay<Double> trajectoryRecorder = new TrajectoryRecordReplay<>(Double.class, directoryAbsolutePath, bodyParts.size());
      trajectoryRecorder.setRecordFileName("generatedMotion.csv");
      LogTools.info("Processing trajectory ...");
      while (!trajectoryPlayer.hasDoneReplay())
      {
         for (String bodyPart : bodyParts){
            FramePose3D framePose = new FramePose3D();
            framePose.setFromReferenceFrame(ReferenceFrame.getWorldFrame());
            // Read file with stored trajectories: read set point per timestep until file is over
            Double[] dataPoint = trajectoryPlayer.play(true);
            // [0,1,2,3] quaternion of body segment; [4,5,6] position of body segment
            framePose.getOrientation().set(dataPoint[0], dataPoint[1], dataPoint[2], dataPoint[3]);
            framePose.getPosition().set(dataPoint[4], dataPoint[5], dataPoint[6]);

            if (proMPAssistant.readyToPack())
            {
               if (!proMPAssistant.isCurrentTaskDone())
                  proMPAssistant.framePoseToPack(framePose, bodyPart);  //change frame according to generated ProMP
            }
            else
            {
               //do not change the frame, just observe it in order to generate a prediction later
               proMPAssistant.processFrameInformation(framePose, bodyPart);
            }
            //record frame and store it in csv file
            Double[] bodyPartTrajectories = new Double[] {framePose.getOrientation().getX(),
                                                          framePose.getOrientation().getY(),
                                                          framePose.getOrientation().getZ(),
                                                          framePose.getOrientation().getS(),
                                                          framePose.getPosition().getX(),
                                                          framePose.getPosition().getY(),
                                                          framePose.getPosition().getZ()};
            trajectoryRecorder.record(bodyPartTrajectories);
         }
      }
      //concatenate each set point of hands in single row
      trajectoryRecorder.concatenateData();
      //save recording in csv file
      trajectoryRecorder.saveRecording();

      LogTools.info("Test completed successfully!");
      LogTools.info("You can visualize the ProMPs plots by running the file {}/1Dplots_ProMPAssistantTest.py", directoryAbsolutePath);
      LogTools.info("You can use file {}/{} as a replay file in Kinematics Streaming Mode",
                    directoryAbsolutePath,
                    trajectoryRecorder.getRecordFileName());
   }
}
