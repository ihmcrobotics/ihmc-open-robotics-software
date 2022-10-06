package us.ihmc.avatar.sharedControl;

import us.ihmc.euclid.geometry.interfaces.Pose3DBasics;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePose3DBasics;
import us.ihmc.promp.*;
import us.ihmc.promp.presets.PrompInfoMapper;
import us.ihmc.tools.io.WorkspaceDirectory;
import us.ihmc.tools.io.WorkspaceFile;

import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import static us.ihmc.promp.Trajectory.infer_closest_trajectory;
import static us.ihmc.promp.global.promp.EigenMatrixXd;

public class ProMPAssistance
{
   private List<ProMP> learnedProMPs = new ArrayList<>();

   public ProMPAssistance()
   {
      try
      {
         loadLibraries(); //load promp libraries generated from cpp
      }
      catch (IOException e)
      {
         throw new RuntimeException(e);
      }
   }

   public void learnTaskFromDemos(String taskName, int numberDemos, HashMap<String, String> bodyPartsGeometry)
   {
      WorkspaceDirectory demoDir = new WorkspaceDirectory("ihmc-open-robotics-software", "promp/etc/demos");
      String demoDirAbs = demoDir.getDirectoryPath().toAbsolutePath().toString();
      String demoTrainingDirAbs = demoDirAbs + taskName;

      List<String> fileListTraining = new ArrayList<>();
      // The trajectories contained in the demos folders represent different demonstration of a given task
      // Several trajectories of different body parts have been recorded
      for (int i = 0; i < numberDemos; i++) //get training files
         fileListTraining.add(demoTrainingDirAbs + "/pr" + (i + 1) + ".csv");

      List<Long> dofs = new ArrayList<>();
      // 0,1,2,3: left hand quaternion; 4,5,6: left hand X,Y,Z; 7-13: right hand;
      for (String bodyPart : bodyPartsGeometry.keySet()) {
         if (bodyPartsGeometry.get(bodyPart).equals("Orientation")){
            dofs.add(0L);
            dofs.add(1L);
            dofs.add(2L);
            dofs.add(3L);
         } else if (bodyPartsGeometry.get(bodyPart).equals("Position")){
            dofs.add(4L);
            dofs.add(5L);
            dofs.add(6L);
         }
         else if (bodyPartsGeometry.get(bodyPart).equals("Pose")){
            dofs.add(0L);
            dofs.add(1L);
            dofs.add(2L);
            dofs.add(3L);
            dofs.add(4L);
            dofs.add(5L);
            dofs.add(6L);
         }
         if(bodyPart.equals("RightHand")){ // TODO add other possible candidate body parts
            dofs.replaceAll(dof -> dof + 7L);
         }
      }

      TrajectoryGroup trainingTrajectories = new TrajectoryGroup();
      //training filelist
      StringVector fileListStringVectorTraining = new StringVector();
      fileListTraining.forEach(fileListStringVectorTraining::push_back);
      SizeTVector doFsSizeTVector = new SizeTVector();
      dofs.forEach(doFsSizeTVector::push_back);
      // load the training trajectories from the csv files
      trainingTrajectories.load_csv_trajectories(fileListStringVectorTraining, doFsSizeTVector);
      // make all training trajectories have the same length
      int meanLengthTraining = (int) trainingTrajectories.normalize_length();
      learnedProMPs.add(new ProMP(trainingTrajectories, 20)); // default 20 rbf functions seems to generalize well
   }

//   public void updateTaskSpeed(String taskName, List<Pose3DReadOnly> observedFrameTrajectory){
//      EigenMatrixXd observedTrajectory = toEigenMatrix(observedFrameTrajectory);
//      // infer the new speed for the ProMP based on observed (portion of) trajectory
//      int demo = infer_closest_trajectory(observedTrajectory, demoTestTrajectories);
//      System.out.println("Inferred closest demo to current observation: " + (demo + 1));

//      double inferredSpeed = demoTestTrajectories.get(demo).infer_speed(observedTrajectory, 0.25, 4.0, 30);
//      int inferredTimesteps = (int) (demoTestTrajectories.get(demo).timesteps() / inferredSpeed);
//      // generate ProMP mean trajectory with new time modulation
//      PrompInfoMapper.EigenMatrixXd stdDeviationTrajectoryModulated = myProMP.gen_traj_std_dev(inferredTimesteps);
//      PrompInfoMapper.EigenMatrixXd meanTrajectoryModulated = myProMP.generate_trajectory(inferredTimesteps);
//      System.out.println("Inferred speed for demo trajectory: " + inferredSpeed);
//      System.out.println("Inferred timestep: " + inferredTimesteps);
//
//      saveAsCSV(meanTrajectoryModulated, "/meanModulated.csv");
//      saveAsCSV(stdDeviationTrajectoryModulated, "/stdDeviationModulated.csv");
//      // update the time modulation of the ProMP object with estimated value
//      System.out.println("Old ProMp timestep: " + myProMP.get_traj_length());
//      myProMP.update_time_modulation((double) myProMP.get_traj_length() / inferredTimesteps);
//      System.out.println("New ProMp timestep: " + myProMP.get_traj_length());
//   }


//   private EigenMatrixXd toEigenMatrix(List<Pose3DReadOnly> frameList)
//   {
//      EigenMatrixXd matrix = new EigenMatrixXd(frameList.size(),);
//      for (int i=0; i<frameList.size(); i++)
//         for (int j=0; j<matrix.cols(); j++)
//            matrix.apply(i, j).put(row, col);
//      return matrix;
//   }

   private void loadLibraries() throws IOException
   {
      // We need to disable javacpp from trying to automatically load libraries.
      // Otherwise, it will try to load them by name when they aren't in the library path
      // (LD_LIBRARY_PATH on Linux).
      //
      // The approach taken here is to use System.load to load each library by explicit
      // absolute path on disk.
      System.setProperty("org.bytedeco.javacpp.loadlibraries", "false");

      List<String> libraryFiles = new ArrayList<>();
      libraryFiles.add("libpromp.so");
      libraryFiles.add("libjnipromp.so");

      WorkspaceDirectory resourcesDirectory = new WorkspaceDirectory("ihmc-open-robotics-software", "promp/src/main/resources");
      for (String libraryFile : libraryFiles)
      {
         System.load(new WorkspaceFile(resourcesDirectory, libraryFile).getFilePath().toAbsolutePath().normalize().toString());
      }
   }
}
