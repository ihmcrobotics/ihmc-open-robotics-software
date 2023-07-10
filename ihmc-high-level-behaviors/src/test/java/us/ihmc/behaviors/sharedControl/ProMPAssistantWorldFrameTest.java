package us.ihmc.behaviors.sharedControl;

import org.junit.jupiter.api.Test;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.log.LogTools;
import us.ihmc.promp.ProMPUtil;
import us.ihmc.behaviors.tools.TrajectoryRecordReplay;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.nio.file.StandardCopyOption;
import java.util.ArrayList;
import java.util.List;
import java.util.Set;

import static org.junit.jupiter.api.Assertions.assertTrue;

public class ProMPAssistantWorldFrameTest
{
   @Test
   public void ProMPTrajectoryGenerationWorldFrameTest() throws IOException
   {
      /* ProMPAssistant.json:
       {
           "testNumberUseOnlyForTesting": 1,
           "logging" : true,
           "numberBasisFunctions": 20,
           "numberObservations": 50,
           "conditionOnlyLastObservation": true,
           "numberOfInferredSpeeds": 30,
           "allowedIncreaseDecreaseSpeedFactor": 2,
           "tasks":[
             {
               "context": "PushDoorLeverHandle",
               "name": "PushDoorWorldFrame",
               "bodyPartForInference": "rightHand",
               "bodyPartWithObservableGoal": "rightHand",
               "translationGoalToEE": [0.135, -0.027,  0.174],
               "rotationGoalToEE": [-0.158, -0.691,  0.681, -0.185],
               "bodyParts":[
                 { "name":"leftHand", "geometry":"Pose" },
                 { "name":"rightHand", "geometry":"Pose" }
               ]
             }
           ]
         }
      */
      // learn ProMPs
      // Check ProMPAssistant.json if you want to change parameters (e.g, task to learn, body parts to consider in the motion)
      ProMPAssistant proMPAssistant = new ProMPAssistant();
      Set<String> tasks = proMPAssistant.getTaskNames();
      assertTrue(tasks.size() > 0);
      String task = tasks.iterator().next();
      ProMPManager myManager = proMPAssistant.getProMPManager(task);
      assertTrue(myManager != null);
      assertTrue(proMPAssistant.getProMPManager(task) != null);
      // use a csv file with the trajectories of the hands of the robot for testing
      String etcDirectory = ProMPUtil.getEtcDirectory().toString();
      String demoDirectory = etcDirectory + "/test/PushDoorWorldFrameTest";
      //get test number from config file
      String testFilePath = demoDirectory + "/" + proMPAssistant.getTestNumber() + ".csv";
      //copy test file to have it always under same name for faster plotting
      Path originalPath = Paths.get(testFilePath);
      Path copyForPlottingPath = Paths.get(demoDirectory + "/../test.csv");
      Files.copy(originalPath, copyForPlottingPath, StandardCopyOption.REPLACE_EXISTING);

      // replay that file
      LogTools.info(testFilePath);
      TrajectoryRecordReplay trajectoryPlayer = new TrajectoryRecordReplay(testFilePath, 2); //2 body parts: the hands
      // start parsing data immedediately, assuming user is moving from beginning of recorded test trajectory
      proMPAssistant.setIsMovingThreshold(0.00001);

      //let's focus on the hands
      List<String> bodyParts = new ArrayList<>();
      bodyParts.add("leftHand");
      bodyParts.add("rightHand");
      TrajectoryRecordReplay trajectoryRecorder = new TrajectoryRecordReplay(etcDirectory, bodyParts.size());
      trajectoryRecorder.setRecordFileName("generatedMotion.csv");
      LogTools.info("Processing trajectory ...");

      while (!trajectoryPlayer.hasDoneReplay())
      {
         for (String bodyPart : bodyParts)
         {
            FramePose3D framePose = new FramePose3D();
            framePose.setFromReferenceFrame(ReferenceFrame.getWorldFrame());
            // Read file with stored trajectories: read set point per timestep until file is over
            double[] dataPoint = trajectoryPlayer.play(true);
            framePose.getOrientation().set(dataPoint);
            framePose.getPosition().set(4, dataPoint);

            if (proMPAssistant.readyToPack())
            {
               if (!proMPAssistant.isCurrentTaskDone())
                  proMPAssistant.framePoseToPack(framePose, bodyPart, true);  //change frame according to generated ProMP
            }
            else
            {
               assertTrue(!proMPAssistant.readyToPack());
               FramePose3D observedGoalPose = null; // no observed goal
               //do not change the frame, just observe it in order to generate a prediction later
               proMPAssistant.processFrameAndObjectInformation(framePose, bodyPart, "PushDoorLeverHandle", observedGoalPose);
            }
            //record frame and store it in csv file
            double[] bodyPartTrajectories = new double[7];
            framePose.getOrientation().get(bodyPartTrajectories);
            framePose.getPosition().get(4, bodyPartTrajectories);
            trajectoryRecorder.record(bodyPartTrajectories);
         }
      }
      //concatenate each set point of hands in single row
      trajectoryRecorder.concatenateData();
      ArrayList<double[]> dataConcatenated = trajectoryRecorder.getConcatenatedData();
      assertTrue(dataConcatenated.size() > 0); // check data is not empty
      // save recorded file name
      String recordFile = trajectoryRecorder.getRecordFileName();
      //save recording in csv file
      trajectoryRecorder.saveRecording();

      LogTools.info("Test completed successfully!");
      LogTools.info("You can visualize the ProMPs plots by running the file {}/1Dplots_ProMPAssistantTest.py", etcDirectory);
      LogTools.info("You can use file {}/{} as a replay file in Kinematics Streaming Mode", etcDirectory, recordFile);
   }
}
