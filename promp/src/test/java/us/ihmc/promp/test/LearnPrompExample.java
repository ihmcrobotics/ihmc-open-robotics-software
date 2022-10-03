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

/**
 * This example shows you how to load training trajectories and learn a multidimensional ProMP for the specified dofs.
 * The ProMP can be intuitively analyzed by plotting its mean trajectory and std deviation.
 */
public class LearnPrompExample
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

      WorkspaceDirectory demoDir = new WorkspaceDirectory("ihmc-open-robotics-software", "promp/etc/demos/Reaching1");
      String demoDirAbs = demoDir.getDirectoryPath().toAbsolutePath().toString();

      List<String> fileList = new ArrayList<>();
      // The trajectories contained in the Reaching1 folder represent different demonstration of a given task
      // Several trajectories of different body parts have been recorded
      // 0: waist Z; 1,2,3: right hand X,Y,Z; 5,6,7: left hand X,Y,Z
      for (int i=0; i<10; i++) //get training files
         fileList.add(demoDirAbs + "/pr"+(i+1)+".csv");
      // consider only right hand trajectories
      List<Long> dofs = List.of(1L, 2L, 3L);

      TrajectoryGroup trajectoryGroup = new TrajectoryGroup();

      StringVector fileListStringVector = new StringVector();
      fileList.forEach(fileListStringVector::push_back);

      SizeTVector doFsSizeTVector = new SizeTVector();
      dofs.forEach(doFsSizeTVector::push_back);

      trajectoryGroup.load_csv_trajectories(fileListStringVector, doFsSizeTVector);
      long t_len = trajectoryGroup.normalize_length();

      int n_rbf = 20;

      ProMP m_promp = new ProMP(trajectoryGroup, n_rbf);
      EigenMatrixXd meanTrajectory = m_promp.generate_trajectory();
      EigenMatrixXd stdTrajectory = m_promp.gen_traj_std_dev();
      EigenMatrixXd covarianceTrajectory = m_promp.generate_trajectory_covariance();

      TrajectoryVector demoTrajectories = trajectoryGroup.trajectories();
      for (int i = 0; i < demoTrajectories.size(); i++)
      {
         saveAsCSV(demoTrajectories.get(i).matrix(), ("/demo" + (i + 1) + ".csv"));
      }
      saveAsCSV(meanTrajectory, "/mean.csv");
      saveAsCSV(stdTrajectory, "/variance.csv");
      saveAsCSV(covarianceTrajectory, "/covariance.csv");

      printMatrix(meanTrajectory, "Mean");
      printMatrix(stdTrajectory, "Std Deviation");
      printMatrix(covarianceTrajectory, "Covariance");
   }
}
