package us.ihmc.avatar.sharedControl;

import us.ihmc.euclid.geometry.interfaces.Pose3DBasics;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePose3DBasics;
import us.ihmc.promp.*;
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
   private HashMap<String, ProMP> learnedProMPs = new HashMap<>();
   String taskName;
   TrajectoryGroup trainingTrajectories;

   public ProMPAssistance(String taskName)
   {
      this.taskName = taskName;
      ProMPNativeLibrary.load();
   }

   public void learnTaskFromDemos(int numberDemos, HashMap<String, String> bodyPartsGeometry)
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
      // This is how the dofs are stored in the csv training files
      // 0,1,2,3: left hand quaternion; 4,5,6: left hand X,Y,Z;
      // 7,8,9,10: right hand quaternion; 11,12,13: left hand X,Y,Z;
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
         if(bodyPart.equals("RightHand")){
            dofs.replaceAll(dof -> dof + 7L);
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
         learnedProMPs.put(bodyPart+bodyPartsGeometry.get(bodyPart),new ProMP(trainingTrajectories, 20)); // default 20 rbf functions seems to generalize well
      }
   }

   public void updateTaskSpeed(List<HashMap<String, Pose3DReadOnly>> observedFrameTrajectories){
      EigenMatrixXd observedTrajectories = toEigenMatrix(observedFrameTrajectories);
      TrajectoryVector demoTrajectories = trainingTrajectories.trajectories();
      // infer what training demo is the closest to the observed trajectory
      int demo = infer_closest_trajectory(observedTrajectories, demoTrajectories);
      System.out.println("Inferred closest demo to current observation: " + (demo + 1));
      // infer the new speed for the ProMP based on observed (portion of) trajectory
      double inferredSpeed = demoTrajectories.get(demo).infer_speed(observedTrajectories, 0.25, 4.0, 30);
      int inferredTimesteps = (int) (demoTrajectories.get(demo).timesteps() / inferredSpeed);
      // update the time modulation of the learned ProMPs with estimated value
      for (ProMP proMPBodyPart : learnedProMPs.values())
         proMPBodyPart.update_time_modulation((double) proMPBodyPart.get_traj_length() / inferredTimesteps);
   }

   /* transform trajectory from list of frameName-points to matrix */
   private EigenMatrixXd toEigenMatrix(List<HashMap<String, Pose3DReadOnly>> frameList)
   {
      EigenMatrixXd matrix = new EigenMatrixXd(frameList.size(),);
      for (int i=0; i<frameList.size(); i++)
         for (int j=0; j<matrix.cols(); j++)
            matrix.apply(i, j).put(frameList.get(i).);
      return matrix;
   }
}
