package us.ihmc.behaviors.sharedControl;

import org.junit.Test;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DBasics;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.log.LogTools;
import us.ihmc.perception.OpenCVArUcoMarker;
import us.ihmc.perception.objects.ArUcoMarkerObject;
import us.ihmc.perception.objects.ObjectInfo;
import us.ihmc.rdx.ui.tools.TrajectoryRecordReplay;
import us.ihmc.tools.io.WorkspaceDirectory;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.nio.file.StandardCopyOption;
import java.util.ArrayList;
import java.util.List;
import java.util.Set;

import static org.junit.Assert.assertTrue;

public class ProMPAssistantObjectFrameTest
{
   private ReferenceFrame objectFrame;

   @Test
   public void ProMPTrajectoryGenerationObjectFrameTest() throws IOException
   {
      createObjectFrameFromAruco();
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
      WorkspaceDirectory directory = new WorkspaceDirectory("ihmc-open-robotics-software", "promp/etc");
      String directoryAbsolutePath = directory.getDirectoryPath().toAbsolutePath().toString();
      String demoDirectory = directoryAbsolutePath + "/test/ReachHandleTest";
      //get test number from config file
      String testFilePath = demoDirectory + "/" + proMPAssistant.getTestNumber() + ".csv";
      //copy test file to have it always under same name for faster plotting
      Path originalPath = Paths.get(testFilePath);
      Path copyForPlottingPath = Paths.get(demoDirectory + "/../test.csv");
      Files.copy(originalPath, copyForPlottingPath, StandardCopyOption.REPLACE_EXISTING);

      // replay that file
      TrajectoryRecordReplay<Double> trajectoryPlayer = new TrajectoryRecordReplay<>(Double.class, testFilePath, 2); //2 body parts: the hands
      trajectoryPlayer.setDoneReplay(false);
      // start parsing data immedediately, assuming user is moving from beginning of recorded test trajectory
      proMPAssistant.setIsMovingThreshold(0.00001);

      //let's focus on the hands
      List<String> bodyParts = new ArrayList<>();
      bodyParts.add("leftHand");
      bodyParts.add("rightHand");
      TrajectoryRecordReplay<Double> trajectoryRecorder = new TrajectoryRecordReplay<>(Double.class, directoryAbsolutePath, bodyParts.size());
      trajectoryRecorder.setRecordFileName("generatedMotion.csv");
      LogTools.info("Processing trajectory ...");

      while (!proMPAssistant.isCurrentTaskDone())
      {
         for (String bodyPart : bodyParts)
         {
            FramePose3D framePose = new FramePose3D(objectFrame);
            // Read file with stored trajectories: read set point per timestep until file is over
            Double[] dataPoint = trajectoryPlayer.play(true);
            // [0,1,2,3] quaternion of body segment; [4,5,6] position of body segment
            framePose.getOrientation().set(dataPoint[0], dataPoint[1], dataPoint[2], dataPoint[3]);
            framePose.getPosition().set(dataPoint[4], dataPoint[5], dataPoint[6]);
            framePose.changeFrame(ReferenceFrame.getWorldFrame());

            if (proMPAssistant.readyToPack())
            {
               if (!proMPAssistant.isCurrentTaskDone())
                  proMPAssistant.framePoseToPack(framePose, bodyPart);  //change frame according to generated ProMP
            }
            else
            {
               assertTrue(!proMPAssistant.readyToPack());
               FramePose3D observedGoalPose = null; // no observed goal
               //do not change the frame, just observe it in order to generate a prediction later
               proMPAssistant.processFrameAndObjectInformation(framePose, bodyPart,  "Door", objectFrame);
            }
            //record frame and store it in csv file
            framePose.changeFrame(objectFrame);
            Double[] bodyPartTrajectories = new Double[] {framePose.getOrientation().getX(),
                                                          framePose.getOrientation().getY(),
                                                          framePose.getOrientation().getZ(),
                                                          framePose.getOrientation().getS(),
                                                          framePose.getPosition().getX(),
                                                          framePose.getPosition().getY(),
                                                          framePose.getPosition().getZ()};
            if(!proMPAssistant.isCurrentTaskDone())
               trajectoryRecorder.record(bodyPartTrajectories);
         }
      }
      //concatenate each set point of hands in single row
      trajectoryRecorder.concatenateData();
      ArrayList<Double[]> dataConcatenated = trajectoryRecorder.getData();
      assertTrue(dataConcatenated.size() > 0); // check data is not empty
      // save recorded file name
      String recordFile = trajectoryRecorder.getRecordFileName();
      //save recording in csv file
      trajectoryRecorder.saveRecording();

      LogTools.info("Test completed successfully!");
      LogTools.info("You can visualize the ProMPs plots by running the file {}/1Dplots_ProMPAssistantTest.py", directoryAbsolutePath);
      LogTools.info("You can use file {}/{} as a replay file in Kinematics Streaming Mode", directoryAbsolutePath, recordFile);
   }

   private void createObjectFrameFromAruco()
   {
      ObjectInfo arucoInfo = new ObjectInfo();
      ArrayList<OpenCVArUcoMarker> markersToTrack = new ArrayList<>();
      ArUcoMarkerObject objectWithArUco;
      // add markers with their respective info
      for (int id : arucoInfo.getMarkersId())
         markersToTrack.add(new OpenCVArUcoMarker(id, arucoInfo.getMarkerSize(id)));
      // get a marker
      OpenCVArUcoMarker marker = markersToTrack.get(0);
      int objectId = marker.getId();
      // get object with attached aruco marker
      objectWithArUco = new ArUcoMarkerObject(objectId, arucoInfo);
      // get marker pose in world frame, in reality this would be detected by camera
      FramePose3DBasics markerPose = new FramePose3D(ReferenceFrame.getWorldFrame(),
                                                     new Point3D(1.073, -0.146, 1.016),
                                                     new Quaternion(-0.002, 1.000, 0.001, 0.003));
      // create from this pose, the associated transform stored in objectWithArUco
      markerPose.get(objectWithArUco.getMarkerToWorld());
      objectWithArUco.updateFrame(); // update frame of the object
      objectWithArUco.computeObjectPose(markerPose); // compute object pose from marker pose

      objectFrame = objectWithArUco.getObjectFrame();
   }
}
