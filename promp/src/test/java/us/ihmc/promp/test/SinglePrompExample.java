package us.ihmc.promp.test;

import org.bytedeco.javacpp.DoublePointer;
import us.ihmc.promp.ProMP;
import us.ihmc.promp.SizeTVector;
import us.ihmc.promp.StringVector;
import us.ihmc.promp.TrajectoryGroup;
import us.ihmc.promp.global.promp;
import us.ihmc.tools.io.WorkspaceDirectory;
import us.ihmc.tools.io.WorkspaceFile;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

public class SinglePrompExample
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
      libraryFiles.add("libjnipromp.so");
      libraryFiles.add("libpromp.so");

      WorkspaceDirectory resourcesDirectory = new WorkspaceDirectory("ihmc-open-robotics-software", "promp/src/main/resources");
      for (String libraryFile : libraryFiles)
      {
         System.load(new WorkspaceFile(resourcesDirectory, libraryFile).getFilePath().toAbsolutePath().normalize().toString());
      }
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

      WorkspaceDirectory putasideBoxDemoDir = new WorkspaceDirectory("ihmc-open-robotics-software", "promp/etc/demos/PutasideBox");
      String putasideBoxDemoDirAbs = putasideBoxDemoDir.getDirectoryPath().toAbsolutePath().toString();

      List<String> fileList = new ArrayList<>();

      fileList.add(putasideBoxDemoDirAbs + "/p1.csv");
      fileList.add(putasideBoxDemoDirAbs + "/p2.csv");
      fileList.add(putasideBoxDemoDirAbs + "/p3.csv");
      fileList.add(putasideBoxDemoDirAbs + "/p4.csv");
      fileList.add(putasideBoxDemoDirAbs + "/p5.csv");
      fileList.add(putasideBoxDemoDirAbs + "/p6.csv");
      fileList.add(putasideBoxDemoDirAbs + "/p7.csv");
      fileList.add(putasideBoxDemoDirAbs + "/p8.csv");
      fileList.add(putasideBoxDemoDirAbs + "/p9.csv");
      fileList.add(putasideBoxDemoDirAbs + "/p10.csv");

      List<Long> dofs = List.of(0L, 1L, 2L);

      TrajectoryGroup trajectoryGroup = new TrajectoryGroup();

      StringVector fileListStringVector = new StringVector();
      fileList.forEach(fileListStringVector::push_back);

      SizeTVector doFsSizeTVector = new SizeTVector();
      dofs.forEach(doFsSizeTVector::push_back);

      trajectoryGroup.load_csv_trajectories(fileListStringVector, doFsSizeTVector);
      long t_len = trajectoryGroup.normalize_length();

      System.out.println("t_len " + t_len);

      int n_rbf = 20;

      ProMP m_promp = new ProMP(trajectoryGroup, n_rbf);
      promp.EigenMatrixXd vect = m_promp.generate_trajectory();
      promp.EigenMatrixXd stdTraj = m_promp.gen_traj_std_dev();
      promp.EigenMatrixXd traj_covariance = m_promp.generate_trajectory_covariance();

      DoublePointer vectData = vect.data();
      DoublePointer stdTrajData = stdTraj.data();
      DoublePointer traj_covarianceData = traj_covariance.data();

      System.out.println("vect");
      for (int i = 0; i < t_len; i++)
      {
         System.out.println(vectData.get(i));
      }
      System.out.println();

      System.out.println("stdTraj");
      for (int i = 0; i < t_len; i++)
      {
         System.out.println(stdTrajData.get(i));
      }
      System.out.println();

      System.out.println("traj_covariance");
      for (int i = 0; i < t_len; i++)
      {
         System.out.println(traj_covarianceData.get(i));
      }
      System.out.println();
   }
}
