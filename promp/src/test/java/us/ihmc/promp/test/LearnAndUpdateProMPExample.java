package us.ihmc.promp.test;

import us.ihmc.promp.*;
import us.ihmc.tools.io.WorkspaceDirectory;
import us.ihmc.tools.io.WorkspaceFile;

import java.io.File;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import static us.ihmc.promp.Trajectory.infer_closest_trajectory;
import static us.ihmc.promp.global.promp.EigenMatrixXd;
import static us.ihmc.promp.presets.PrompInfoMapper.EigenVectorXd;

public class LearnAndUpdateProMPExample
{
   private static void loadLibraries() throws IOException
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

   private static void saveAsCSV(EigenMatrixXd dataMatrix, String fileName)
   {
      List<String[]> dataLines = new ArrayList<>();
      for (int i = 0; i < dataMatrix.rows(); i++)
      {
         String[] stringLine = new String[(int) dataMatrix.cols()];
         for (int j = 0; j < dataMatrix.cols(); j++)
            stringLine[j] = "" + dataMatrix.coeff(i, j);
         dataLines.add(stringLine);
      }
      WorkspaceDirectory fileDirectory = new WorkspaceDirectory("ihmc-open-robotics-software", "promp/etc");
      String fileDirAbs = fileDirectory.getDirectoryPath().toAbsolutePath().toString();
      File csvFile = new File(fileDirAbs + fileName);
      try (PrintWriter writer = new PrintWriter(csvFile))
      {
         dataLines.stream().map(s -> convertToCSV(s)).forEach(writer::println);
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }
   }

   private static String convertToCSV(String[] data)
   {
      return Stream.of(data).collect(Collectors.joining(","));
   }

   private static void printMatrix(EigenMatrixXd matrix, String name)
   {
      System.out.println(name);
      for (int row = 0; row < matrix.rows(); row++)
      {
         for (int col = 0; col < matrix.cols(); col++)
         {
            System.out.print(matrix.coeff(row, col) + " ");
         }
         System.out.println();
      }
      System.out.println();
   }

   public static void main(String[] args)
   {
      try
      {
         loadLibraries();
      }
      catch (IOException e)
      {
         throw new RuntimeException(e);
      }
      /*
       * Load training and test trajectories
       */
      WorkspaceDirectory demoDir = new WorkspaceDirectory("ihmc-open-robotics-software", "promp/etc/demos");
      String demoDirAbs = demoDir.getDirectoryPath().toAbsolutePath().toString();
      String demoTrainingDirAbs = demoDirAbs + "/Reaching1";
      String demoTestingDirAbs = demoDirAbs + "/Reaching2";

      List<String> fileListTraining = new ArrayList<>();
      // The trajectories contained in the Reaching1,2 folders represent different demonstration of a given task
      // Several trajectories of different body parts have been recorded
      // 0: waist Z; 1,2,3: right hand X,Y,Z; 5,6,7: left hand X,Y,Z
      for (int i = 0; i < 10; i++) //get training files
         fileListTraining.add(demoTrainingDirAbs + "/pr" + (i + 1) + ".csv");

      List<String> fileListTesting = new ArrayList<>();
      for (int i = 0; i < 6; i++) //get testing files
         fileListTesting.add(demoTestingDirAbs + "/pr" + (i + 1) + ".csv");

      // consider only right hand trajectories
      List<Long> dofs = List.of(1L, 2L, 3L);
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

      //testing filelist
      StringVector fileListStringVectorTesting = new StringVector();
      fileListTesting.forEach(fileListStringVectorTesting::push_back);
      TrajectoryGroup testingTrajectories = new TrajectoryGroup();
      // load the testing trajectories from the csv files
      testingTrajectories.load_csv_trajectories(fileListStringVectorTesting, doFsSizeTVector);

      /*
       * Learn ProMP
       */
      int n_rbf = 20;
      ProMP myProMP = new ProMP(trainingTrajectories, n_rbf);
      EigenMatrixXd meanTrajectory = myProMP.generate_trajectory();
      EigenMatrixXd stdDeviationTrajectory = myProMP.gen_traj_std_dev();
      EigenMatrixXd covarianceTrajectory = myProMP.generate_trajectory_covariance();

      /*
       * Create training/demo trajectories vectors for logging and later usage
       */
      TrajectoryVector demoTrajectories = trainingTrajectories.trajectories();
      TrajectoryVector demoTestTrajectories = testingTrajectories.trajectories();
      for (int i = 0; i < demoTrajectories.size(); i++)
      {
         saveAsCSV(demoTrajectories.get(i).matrix(), ("/demo" + (i + 1) + ".csv"));
      }
      for (int i = 0; i < demoTestTrajectories.size(); i++)
      {
         saveAsCSV(demoTestTrajectories.get(i).matrix(), ("/test" + (i + 1) + ".csv"));
      }
      saveAsCSV(meanTrajectory, "/mean.csv");
      saveAsCSV(stdDeviationTrajectory, "/stdDeviation.csv");
      saveAsCSV(covarianceTrajectory, "/covariance.csv");

      /*
       * Select a demo trajectory and infer modulation of ProMP based on that demo
       * then update ProMP modulation accordingly
       */
      // see timesteps of selected demo trajectory
      long timestepDemo = demoTestTrajectories.get(0).timesteps();
      System.out.println("timestepDemo: " + timestepDemo);

      // infer the new speed for the ProMP based on observed portion of demo trajectory
      int observedTimesteps = (int) timestepDemo / 3;
      // build observed matrix from demo test 1
      EigenMatrixXd observedTrajectory = new EigenMatrixXd(observedTimesteps, (int) meanTrajectory.cols());
      for (int i = 0; i < observedTrajectory.rows(); i++)
         for (int j = 0; j < observedTrajectory.cols(); j++)
            observedTrajectory.apply(i, j).put(demoTestTrajectories.get(0).matrix().coeff(i, j));

      int demo = infer_closest_trajectory(observedTrajectory, demoTestTrajectories);
      System.out.println("Inferred closest demo to current observation: " + (demo + 1));

      double inferredSpeed = demoTestTrajectories.get(demo).infer_speed(observedTrajectory, 0.25, 4.0, 30);
      int inferredTimesteps = (int) (demoTestTrajectories.get(demo).timesteps() / inferredSpeed);
      // generate ProMP mean trajectory with new time modulation
      EigenMatrixXd stdDeviationTrajectoryModulated = myProMP.gen_traj_std_dev(inferredTimesteps);
      EigenMatrixXd meanTrajectoryModulated = myProMP.generate_trajectory(inferredTimesteps);
      System.out.println("Inferred speed for demo trajectory: " + inferredSpeed);
      System.out.println("Inferred timestep: " + inferredTimesteps);

      saveAsCSV(meanTrajectoryModulated, "/meanModulated.csv");
      saveAsCSV(stdDeviationTrajectoryModulated, "/stdDeviationModulated.csv");
      // update the time modulation of the ProMP object with estimated value
      System.out.println("Old ProMp timestep: " + myProMP.get_traj_length());
      myProMP.update_time_modulation((double) myProMP.get_traj_length() / inferredTimesteps);
      System.out.println("New ProMp timestep: " + myProMP.get_traj_length());

      /*
       * Update ProMP with conditioning operation according to observations from demo
       */
      EigenMatrixXd viaPointStdDeviation = new EigenMatrixXd(dofs.size(), dofs.size());
      for (int i = 0; i < viaPointStdDeviation.rows(); i++)
      {
         for (int j = 0; j < viaPointStdDeviation.cols(); j++)
         {
            if (i == j)
               viaPointStdDeviation.apply(i, j).put(0); // Preferably keep std low. Lower std -> higher precision but less damping
            else
               viaPointStdDeviation.apply(i, j).put(0);
         }
      }
      // condition point at a general timestep
      EigenVectorXd viaPoint = new EigenVectorXd(dofs.size());
      int conditioningTimestep = 10;
      //      for (int i=0; i<viaPoint.size(); i++)
      //         viaPoint.apply(i).put(demoTestTrajectories.get(0).matrix().coeff(conditioningTimestep,i));
      //      myProMP.condition_via_point(conditioningTimestep,viaPoint,viaPointStdDeviation);
      // condition goal
      conditioningTimestep = (int) demoTestTrajectories.get(0).timesteps()-1;
      System.out.print("Conditioning timestep: " + conditioningTimestep);
      System.out.println("; Via point: " + demoTestTrajectories.get(0).matrix().coeff(conditioningTimestep, 0) + " " + demoTestTrajectories.get(0).matrix().coeff(conditioningTimestep, 1) + " " + demoTestTrajectories.get(0).matrix().coeff(conditioningTimestep, 2));
      myProMP.set_conditioning_ridge_factor(0.01);
      for (int i = 0; i < viaPoint.size(); i++)
         viaPoint.apply(i).put(demoTestTrajectories.get(0).matrix().coeff(conditioningTimestep, i));
      myProMP.condition_goal(viaPoint, viaPointStdDeviation);
      //generate updated mean trajectory
      EigenMatrixXd meanTrajectoryConditioned = myProMP.generate_trajectory();
      EigenMatrixXd stdDeviationTrajectoryConditioned = myProMP.gen_traj_std_dev(inferredTimesteps);

      saveAsCSV(meanTrajectoryConditioned, "/meanConditioned.csv");
      saveAsCSV(stdDeviationTrajectoryConditioned, "/stdDeviationConditioned.csv");
   }
}
