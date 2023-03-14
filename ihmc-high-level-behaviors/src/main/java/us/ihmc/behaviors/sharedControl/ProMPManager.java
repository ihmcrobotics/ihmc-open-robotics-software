package us.ihmc.behaviors.sharedControl;

import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.log.LogTools;
import us.ihmc.promp.*;
import us.ihmc.tools.io.WorkspaceDirectory;

import java.io.File;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.atomic.AtomicBoolean;

import static us.ihmc.promp.ProMPUtil.concatenateEigenMatrix;
import static us.ihmc.promp.ProMPUtil.concatenateTrajectoryVector;
import static us.ihmc.promp.Trajectory.infer_closest_trajectory;
import static us.ihmc.promp.presets.ProMPInfoMapper.EigenMatrixXd;
import static us.ihmc.promp.presets.ProMPInfoMapper.EigenVectorXd;

/**
 * Class to learn and use ProMPs for assistance in a given task. Based on promp java-cpp library.
 * A ProMP represents a probabilistic prediction of a multi-dimensional trajectory.
 * The dimension is set in the constructor according to the information you want to retain
 * (e.g, position, orientation, pose of a body part of the robot).
 * The prediction can be updated to match the speed and trend of the current motion.
 * The mean trajectory of the prediction (which can be updated) is always used as the actual predicted trajectory.
 */
public class ProMPManager
{
   private final String taskName;
   private final HashMap<String, String> bodyPartsGeometry;
   // learnedProMPs stores a set of multi-D (e.g., for pose D=6, for position D=3) proMPs. We have a multi-D proMP for each body part
   private final HashMap<String, ProMP> learnedProMPs = new HashMap<>();
   private final HashMap<String, TrajectoryGroup> trainingTrajectories = new HashMap<>();
   private final ProMPLogger logger = new ProMPLogger();
   private boolean logEnabled = false;
   private final AtomicBoolean isLastViaPoint;
   private final int numberBasisFunctions;
   private final long speedFactor;
   private final int numberOfInferredSpeeds;
   private double meanEndValueQS;

   /**
    * Class constructor
    *
    * @param taskName:          name of the task
    * @param bodyPartsGeometry: body part of the robot and geometry (e.g. position, orientation, pose) for which you want to learn the ProMPs
    */
   public ProMPManager(String taskName,
                       HashMap<String, String> bodyPartsGeometry,
                       boolean logEnabled,
                       AtomicBoolean isLastViaPoint,
                       int numberBasisFunctions,
                       long speedFactor,
                       int numberOfInferredSpeeds)
   {
      this.taskName = taskName;
      this.bodyPartsGeometry = bodyPartsGeometry;
      this.logEnabled = logEnabled;
      this.isLastViaPoint = isLastViaPoint;
      this.numberBasisFunctions = numberBasisFunctions; // 20 rbf functions seems to generalize well
      this.speedFactor = speedFactor;
      this.numberOfInferredSpeeds = numberOfInferredSpeeds;

      ProMPNativeLibrary.load();
   }

   /**
    * Learn the ProMPs for the task based on the demo training trajectories stored in .../promp/etc/demos
    * learn a ProMP for each bodyPart specified in the constructor of this class
    */
   public void learnTaskFromDemos()
   {
      WorkspaceDirectory demoDir = new WorkspaceDirectory("ihmc-open-robotics-software", "promp/etc/demos");
      String demoDirAbs = demoDir.getDirectoryPath().toAbsolutePath().toString();
      String demoTrainingDirAbs = demoDirAbs + "/" + taskName;
      File demoFolder = new File(demoTrainingDirAbs);
      File[] listOfFiles = demoFolder.listFiles();
      List<String> fileListTraining = new ArrayList<>();
      // The trajectories contained in the demos folders represent different demonstration of a given task
      // Several trajectories of different body parts have been recorded
      for (int i = 0; i < listOfFiles.length; i++) //get training files
         fileListTraining.add(demoTrainingDirAbs + "/" + (i + 1) + ".csv");

      // This is how the dofs are stored in the csv training files (generated using KinematicsRecordReplay in GDXVRKinematicsStreaming)
      // 0,1,2,3: left hand quaternion; 4,5,6: left hand X,Y,Z;
      // 7,8,9,10: right hand quaternion; 11,12,13: right hand X,Y,Z;
      for (Map.Entry<String, String> entry : bodyPartsGeometry.entrySet())
      {
         List<Long> dofs = new ArrayList<>();
         if (entry.getValue().equals("Orientation"))
         {
            dofs.add(0L);
            dofs.add(1L);
            dofs.add(2L);
            dofs.add(3L);
         }
         else if (entry.getValue().equals("Position"))
         {
            dofs.add(4L);
            dofs.add(5L);
            dofs.add(6L);
         }
         else if (entry.getValue().equals("Pose"))
         {
            dofs.add(0L);
            dofs.add(1L);
            dofs.add(2L);
            dofs.add(3L);
            dofs.add(4L);
            dofs.add(5L);
            dofs.add(6L);
         }
         if (entry.getKey().equals("rightHand"))
         {
            dofs.replaceAll(dof -> dof + 7L);
         }
         TrajectoryGroup trainingTrajectory = new TrajectoryGroup();
         // training filelist
         StringVector fileListStringVectorTraining = new StringVector();
         fileListTraining.forEach(fileListStringVectorTraining::push_back);
         SizeTVector doFsSizeTVector = new SizeTVector();
         dofs.forEach(doFsSizeTVector::push_back);
         // load the training trajectories from the csv files
         trainingTrajectory.load_csv_trajectories(fileListStringVectorTraining, doFsSizeTVector);
         // make all training trajectories have the same length (= mean length)
         int meanLengthTraining = (int) trainingTrajectory.normalize_length();
         // get mean end value of quaternion S, will be used to check and eventually change sign of observed goal quaternion
         if (dofs.size() != 3)
            meanEndValueQS = trainingTrajectory.get_mean_end_value(3);
         trainingTrajectories.put(entry.getKey(), trainingTrajectory);
         learnedProMPs.put(entry.getKey(), new ProMP(trainingTrajectory, numberBasisFunctions));

         if (logEnabled)
         {
            logger.saveDemosAndLearnedTrajectories(entry.getKey(), learnedProMPs.get(entry.getKey()), trainingTrajectory);
         }
      }
   }

   public void resetTask()
   {
      for (Map.Entry<String, String> partGeometry : bodyPartsGeometry.entrySet())
         learnedProMPs.replace(partGeometry.getKey(), new ProMP(trainingTrajectories.get(partGeometry.getKey()), numberBasisFunctions));
   }

   /**
    * Update the speed of the ProMPs of the task based on observation of a body part trajectory (e.g., RightHand or LeftHand)
    */
   public void updateTaskSpeed(List<FramePose3D> observedFrameTrajectory, String bodyPart)
   {
      EigenMatrixXd observedTrajectory = toEigenMatrix(observedFrameTrajectory, bodyPart);
      TrajectoryVector demoTrajectories = trainingTrajectories.get(bodyPart).trajectories();
      // infer what training demo is the closest to the observed trajectory
      int demo = infer_closest_trajectory(observedTrajectory, demoTrajectories);
      // infer the new speed for the demo trajectory based on observed (portion of) trajectory
      double inferredSpeed = demoTrajectories.get(demo).infer_speed(observedTrajectory, 1.0 / speedFactor, speedFactor, numberOfInferredSpeeds);
      // find equivalent timesteps
      int inferredTimesteps = (int) (demoTrajectories.get(demo).timesteps() / inferredSpeed);
      if (logEnabled)
         LogTools.info("Inferred Timesteps: {}", inferredTimesteps);
      // update the time modulation of the learned ProMPs with estimated value
      for (Map.Entry<String, ProMP> partProMP : learnedProMPs.entrySet())
      {
         (partProMP.getValue()).update_time_modulation((double) (partProMP.getValue()).get_traj_length() / inferredTimesteps);
         if (logEnabled)
         {
            logger.saveUpdatedTrajectories(partProMP.getKey(), partProMP.getValue(), "Modulated");
         }
      }
   }

   /**
    * Update the speed of the ProMPs of the task based on observation of multiple body parts trajectories
    */
   public void updateTaskSpeed(List<List<FramePose3D>> observedFrameTrajectories, List<String> bodyParts)
   {
      List<EigenMatrixXd> observedTrajectories = new ArrayList<>();
      List<TrajectoryVector> demoTrajectories = new ArrayList<>();
      for (int i = 0; i < observedFrameTrajectories.size(); i++)
      {
         observedTrajectories.add(toEigenMatrix(observedFrameTrajectories.get(i), bodyParts.get(i)));
         demoTrajectories.add(trainingTrajectories.get(bodyParts.get(i)).trajectories());
      }
      // concatenate observed trajectories of bodyParts in a single EigenMatrix object
      EigenMatrixXd observedTrajectory = concatenateEigenMatrix(observedTrajectories.get(0), observedTrajectories.get(1));
      for (int i = 2; i < observedFrameTrajectories.size(); i++)
      {
         observedTrajectory = concatenateEigenMatrix(observedTrajectory, observedTrajectories.get(i));
      }
      // concatenate demo trajectories of bodyParts in a single Trajectory Vector object
      TrajectoryVector demoTrajectory = concatenateTrajectoryVector(demoTrajectories.get(0), demoTrajectories.get(1));
      for (int i = 2; i < demoTrajectories.size(); i++)
      {
         demoTrajectory = concatenateTrajectoryVector(demoTrajectory, demoTrajectories.get(i));
      }
      // infer what training demo is the closest to the observed trajectory
      int demo = infer_closest_trajectory(observedTrajectory, demoTrajectory);
      // infer the new speed for the demo trajectory based on observed (portion of) trajectory
      double inferredSpeed = demoTrajectory.get(demo).infer_speed(observedTrajectory, 1.0 / speedFactor, speedFactor, numberOfInferredSpeeds);
      // find equivalent timesteps
      int inferredTimesteps = (int) (demoTrajectory.get(demo).timesteps() / inferredSpeed);
      if (logEnabled)
         LogTools.info("Inferred Timesteps: {}", inferredTimesteps);
      // update the time modulation of the learned ProMPs with estimated value
      for (Map.Entry<String, ProMP> partProMP : learnedProMPs.entrySet())
      {
         (partProMP.getValue()).update_time_modulation((double) (partProMP.getValue()).get_traj_length() / inferredTimesteps);

         if (logEnabled)
         {
            logger.saveUpdatedTrajectories(partProMP.getKey(), partProMP.getValue(), "Modulated");
         }
      }
   }

   /**
    * Update the speed of the ProMPs of the task based on observation of a body part trajectory and goal (e.g., RightHand or LeftHand)
    * more accurate but much slower
    */
   public void updateTaskSpeed(List<FramePose3D> observedFrameTrajectory, Pose3DReadOnly observedGoal, String bodyPart)
   {
      EigenMatrixXd observedTrajectory = toEigenMatrix(observedFrameTrajectory, bodyPart);
      // create a copy of proMP for current task
      // NOTE. we do not want to condition a proMP that will be modulated afterwards, this will likely corrupt the model
      ProMP copyProMPCurrentTask = new ProMP(trainingTrajectories.get(bodyPart), numberBasisFunctions);
      // condition proMP to reach observed goal
      updateTrajectoryGoal(copyProMPCurrentTask, bodyPart, observedGoal);
      Trajectory meanTrajectoryProMPCurrentTask = new Trajectory(copyProMPCurrentTask.generate_trajectory(), 1.0);
      // infer the new speed for the mean trajectory based on observed (portion of) trajectory
      double inferredSpeed = meanTrajectoryProMPCurrentTask.infer_speed(observedTrajectory, 1.0 / speedFactor, speedFactor, numberOfInferredSpeeds);
      // find equivalent timesteps
      int inferredTimesteps = (int) (meanTrajectoryProMPCurrentTask.timesteps() / inferredSpeed);
      if (logEnabled)
         LogTools.info("Inferred Timesteps: {}", inferredTimesteps);
      // update the time modulation of the learned ProMPs with estimated value
      for (Map.Entry<String, ProMP> partProMP : learnedProMPs.entrySet())
      {
         (partProMP.getValue()).update_time_modulation((double) (partProMP.getValue()).get_traj_length() / inferredTimesteps);
         if (logEnabled)
         {
            logger.saveUpdatedTrajectories(partProMP.getKey(), partProMP.getValue(), "Modulated");
         }
      }
   }

   /**
    * Transform trajectory from list of set poses to EigenMatrixXd
    */
   private EigenMatrixXd toEigenMatrix(List<FramePose3D> frameList, String bodyPart)
   {
      EigenMatrixXd matrix = null;
      if (bodyPartsGeometry.get(bodyPart).equals("Orientation"))
      {
         matrix = new EigenMatrixXd(frameList.size(), 4);
         for (int i = 0; i < matrix.rows(); i++)
         {
            matrix.apply(i, 0).put(frameList.get(i).getOrientation().getX());
            matrix.apply(i, 1).put(frameList.get(i).getOrientation().getY());
            matrix.apply(i, 2).put(frameList.get(i).getOrientation().getZ());
            matrix.apply(i, 3).put(frameList.get(i).getOrientation().getS());
         }
      }
      else if (bodyPartsGeometry.get(bodyPart).equals("Position"))
      {
         matrix = new EigenMatrixXd(frameList.size(), 3);
         for (int i = 0; i < matrix.rows(); i++)
         {
            matrix.apply(i, 0).put(frameList.get(i).getPosition().getX());
            matrix.apply(i, 1).put(frameList.get(i).getPosition().getY());
            matrix.apply(i, 2).put(frameList.get(i).getPosition().getZ());
         }
      }
      else if (bodyPartsGeometry.get(bodyPart).equals("Pose"))
      {
         matrix = new EigenMatrixXd(frameList.size(), 7);
         for (int i = 0; i < matrix.rows(); i++)
         {
            matrix.apply(i, 0).put(frameList.get(i).getOrientation().getX());
            matrix.apply(i, 1).put(frameList.get(i).getOrientation().getY());
            matrix.apply(i, 2).put(frameList.get(i).getOrientation().getZ());
            matrix.apply(i, 3).put(frameList.get(i).getOrientation().getS());
            matrix.apply(i, 4).put(frameList.get(i).getPosition().getX());
            matrix.apply(i, 5).put(frameList.get(i).getPosition().getY());
            matrix.apply(i, 6).put(frameList.get(i).getPosition().getZ());
         }
      }
      return matrix;
   }

   /**
    * Transform framePose to EigenVectorXd
    */
   private EigenVectorXd toEigenVector(FramePose3D framePose, String bodyPart)
   {
      EigenVectorXd vector = null;
      if (bodyPartsGeometry.get(bodyPart).equals("Orientation"))
      {
         vector = new EigenVectorXd(4);
         for (int i = 0; i < vector.size(); i++)
         {
            vector.apply(0).put(framePose.getOrientation().getX());
            vector.apply(1).put(framePose.getOrientation().getY());
            vector.apply(2).put(framePose.getOrientation().getZ());
            vector.apply(3).put(framePose.getOrientation().getS());
         }
      }
      else if (bodyPartsGeometry.get(bodyPart).equals("Position"))
      {
         vector = new EigenVectorXd(3);
         for (int i = 0; i < vector.size(); i++)
         {
            vector.apply(4).put(framePose.getPosition().getX());
            vector.apply(5).put(framePose.getPosition().getY());
            vector.apply(6).put(framePose.getPosition().getZ());
         }
      }
      else if (bodyPartsGeometry.get(bodyPart).equals("Pose"))
      {
         vector = new EigenVectorXd(7);
         for (int i = 0; i < vector.size(); i++)
         {
            vector.apply(0).put(framePose.getOrientation().getX());
            vector.apply(1).put(framePose.getOrientation().getY());
            vector.apply(2).put(framePose.getOrientation().getZ());
            vector.apply(3).put(framePose.getOrientation().getS());
            vector.apply(4).put(framePose.getPosition().getX());
            vector.apply(5).put(framePose.getPosition().getY());
            vector.apply(6).put(framePose.getPosition().getZ());
         }
      }

      return vector;
   }

   public double computeInitialDistance(FramePose3D observedFramePose, String bodyPart)
   {
      double distance = 0.0;
      EigenVectorXd observedPose = toEigenVector(observedFramePose, bodyPart);
      for (int i = 0; i < observedPose.size(); i++)
      {
         distance += Math.abs(observedPose.coeff(i) - trainingTrajectories.get(bodyPart).get_mean_start_value(i));
      }

      return distance;
   }

   /**
    * Update the predicted trajectories based on observed set poses
    */
   public void updateTaskTrajectories(HashMap<String, Pose3DReadOnly> bodyPartObservedPose, int conditioningTimestep)
   {
      // condition ProMP to reach point at given timestep
      learnedProMPs.keySet()
                   .forEach(bodyPart -> updateTaskTrajectory(bodyPart, bodyPartObservedPose.get(bodyPart), conditioningTimestep));
   }

   /**
    * Update the predicted trajectory based on observed set pose
    */
   public void updateTaskTrajectory(String bodyPart, Pose3DReadOnly observedPose, int conditioningTimestep)
   {
      updateTrajectory(learnedProMPs.get(bodyPart), bodyPart, observedPose, conditioningTimestep);
   }

   private void updateTrajectory(ProMP myProMP, String bodyPart, Pose3DReadOnly observedPose, int conditioningTimestep)
   {
      EigenVectorXd viaPoint = new EigenVectorXd((int) myProMP.get_dims());
      setViaPoint(viaPoint, bodyPart, observedPose);
      myProMP.condition_via_point(conditioningTimestep, viaPoint);
      if (logEnabled)
      {
         if (isLastViaPoint.get())
         {
            logger.saveUpdatedTrajectories(bodyPart, myProMP, "Conditioned");
            isLastViaPoint.set(false);
         }
      }
   }

   /**
    * Update the predicted trajectory based on observed goal
    */
   public void updateTaskTrajectoriesGoal(HashMap<String, Pose3DReadOnly> bodyPartObservedPose)
   {
      // condition ProMP to reach end point
      learnedProMPs.keySet()
                   .forEach(bodyPart -> updateTaskTrajectoryGoal(bodyPart, bodyPartObservedPose.get(bodyPart)));
   }

   /**
    * Update the predicted trajectory based on observed goal
    */
   public void updateTaskTrajectoryGoal(String bodyPart, Pose3DReadOnly observedPose)
   {
      updateTrajectoryGoal(learnedProMPs.get(bodyPart), bodyPart, observedPose);
   }

   private void updateTrajectoryGoal(ProMP myProMP, String bodyPart, Pose3DReadOnly observedPose)
   {
      // condition ProMP to reach end point
      EigenVectorXd viaPoint = new EigenVectorXd((int) myProMP.get_dims());
      setViaPoint(viaPoint, bodyPart, observedPose);
      myProMP.condition_goal(viaPoint);
      if (logEnabled)
      {
         LogTools.info("Logging goal conditioning ...");
         logger.saveUpdatedTrajectories(bodyPart, myProMP, "Conditioned");
      }
   }

   private void setViaPoint(EigenVectorXd viaPoint, String bodyPart, Pose3DReadOnly observedPose)
   {
      if (bodyPartsGeometry.get(bodyPart).equals("Position"))
      {
         viaPoint.apply(0).put(observedPose.getPosition().getX());
         viaPoint.apply(1).put(observedPose.getPosition().getY());
         viaPoint.apply(2).put(observedPose.getPosition().getZ());
      }
      else if (bodyPartsGeometry.get(bodyPart).equals("Orientation"))
      {
         viaPoint.apply(0).put(observedPose.getOrientation().getX());
         viaPoint.apply(1).put(observedPose.getOrientation().getY());
         viaPoint.apply(2).put(observedPose.getOrientation().getZ());
         viaPoint.apply(3).put(observedPose.getOrientation().getS());
      }
      else if (bodyPartsGeometry.get(bodyPart).equals("Pose"))
      {
         viaPoint.apply(0).put(observedPose.getOrientation().getX());
         viaPoint.apply(1).put(observedPose.getOrientation().getY());
         viaPoint.apply(2).put(observedPose.getOrientation().getZ());
         viaPoint.apply(3).put(observedPose.getOrientation().getS());
         viaPoint.apply(4).put(observedPose.getPosition().getX());
         viaPoint.apply(5).put(observedPose.getPosition().getY());
         viaPoint.apply(6).put(observedPose.getPosition().getZ());
      }
   }

   /**
    * Generate mean of predicted trajectory as a list of frame poses
    */
   public List<FramePose3D> generateTaskTrajectory(String bodyPart, ReferenceFrame frame)
   {
      EigenMatrixXd meanTrajectoryConditioned = learnedProMPs.get(bodyPart).generate_trajectory();
      return toFrameList(meanTrajectoryConditioned, bodyPart, frame);
   }

   /**
    * Transform trajectory from list of frame poses to EigenMatrixXd
    */
   private List<FramePose3D> toFrameList(EigenMatrixXd matrix, String bodyPart, ReferenceFrame frame)
   {
      List<FramePose3D> frameList = new ArrayList<>();
      for (int i = 0; i < matrix.rows(); i++)
      {
         FramePose3D setPose = new FramePose3D(frame);
         if (bodyPartsGeometry.get(bodyPart).equals("Position"))
         {
            setPose.getPosition().set(matrix.coeff(i, 0), matrix.coeff(i, 1), matrix.coeff(i, 2));
         }
         else if (bodyPartsGeometry.get(bodyPart).equals("Orientation"))
         {
            setPose.getOrientation().set(matrix.coeff(i, 0), matrix.coeff(i, 1), matrix.coeff(i, 2), matrix.coeff(i, 3));
         }
         else if (bodyPartsGeometry.get(bodyPart).equals("Pose"))
         {
            setPose.getOrientation().set(matrix.coeff(i, 0), matrix.coeff(i, 1), matrix.coeff(i, 2), matrix.coeff(i, 3));
            setPose.getPosition().set(matrix.coeff(i, 4), matrix.coeff(i, 5), matrix.coeff(i, 6));
         }
         frameList.add(setPose);
      }
      return frameList;
   }

   public TrajectoryGroup getTrainingTrajectories(String bodyPart)
   {
      return trainingTrajectories.get(bodyPart);
   }

   public HashMap<String, ProMP> getLearnedProMPs()
   {
      return learnedProMPs;
   }

   public ProMP getLearnedProMP(String bodyPart)
   {
      return learnedProMPs.get(bodyPart);
   }

   public String getTaskName()
   {
      return taskName;
   }

   public HashMap<String, String> getBodyPartsGeometry()
   {
      return bodyPartsGeometry;
   }

   public double getMeanEndValueQS()
   {
      return meanEndValueQS;
   }
}
