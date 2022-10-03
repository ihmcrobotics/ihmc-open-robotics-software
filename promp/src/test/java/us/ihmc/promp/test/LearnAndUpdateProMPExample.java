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

import static us.ihmc.promp.global.promp.EigenMatrixXd;

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
      /* Load trajectories */
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

      /* Learn ProMP */
      int n_rbf = 20;
      ProMP myProMP = new ProMP(trainingTrajectories, n_rbf);
      EigenMatrixXd meanTrajectory = myProMP.generate_trajectory();
      EigenMatrixXd stdTrajectory = myProMP.gen_traj_std_dev();
      EigenMatrixXd covarianceTrajectory = myProMP.generate_trajectory_covariance();

      /* Create training/demo trajectories vectors for logging and later usage */
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
      saveAsCSV(stdTrajectory, "/variance.csv");
      saveAsCSV(covarianceTrajectory, "/covariance.csv");

      /* Select a demo trajectory and infer modulation of ProMP */
      // create trajectory object for the meanTrajectory of the ProMP
//      EigenMatrixXd meanModulatedTrajectory = myProMP.generate_trajectory_with_speed();
      Trajectory trajectoryOriginal = new Trajectory(meanTrajectory, 1.0);
      // see current timesteps of ProMP
      long timestepOriginal = trajectoryOriginal.timesteps();
      // current speed is 1
      double speedOriginal = trajectoryOriginal.speed();
      System.out.println("speedOriginal: " + speedOriginal);
      System.out.println("timestepOriginal: " + timestepOriginal);
      // see timesteps of selected demo trajectory
      long timestepDemo = demoTestTrajectories.get(0).timesteps();
      System.out.println("timestepDemo: " + timestepDemo);

      // infer the new speed for the ProMP based on observed portion of demo trajectory
      int observedTimesteps = (int) timestepDemo/4;
      // build observed matrix
      EigenMatrixXd observedTrajectory= new EigenMatrixXd(observedTimesteps, (int) meanTrajectory.cols());
      for (int i=0; i<observedTrajectory.rows(); i++){
         for (int j=0; i<observedTrajectory.cols(); j++){
            observedTrajectory.coeff(i,j) = demoTestTrajectories.get(0).matrix().coeff(i,j);
         }
      }
//      double inferredSpeed = trajectoryOriginal.infer_speed(demoTestTrajectories.get(0).matrix(), 0.25, 4.0, 20);
      double inferredSpeed = trajectoryOriginal.infer_speed(observedTrajectory, 0.25, 4.0, 20);
      trajectoryOriginal.modulate((long) (timestepOriginal / inferredSpeed));
      timestepOriginal = trajectoryOriginal.timesteps();
      System.out.println("Inferred speed for demo trajectory: " + inferredSpeed);
      System.out.println("New timestepOriginal: " + timestepOriginal);


   }
}
