package us.ihmc.promp.test;

import us.ihmc.promp.*;

import java.io.File;
import java.net.URISyntaxException;
import java.util.ArrayList;
import java.util.List;
import java.util.Objects;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;
import static us.ihmc.promp.presets.ProMPInfoMapper.EigenMatrixXd;

/**
 * This example shows you how to load training trajectories and learn a multidimensional ProMP for the specified dofs.
 * The ProMP can be intuitively analyzed by plotting its mean trajectory and std deviation.
 */
public class LearnProMPExampleTest
{
   private final boolean SAVEFILES = false;

   @Test
   public void testLearningProMP() throws URISyntaxException
   {
      ProMPNativeLibrary.load();

      File file = new File(Objects.requireNonNull(getClass().getClassLoader().getResource("promp/cppLibraryTestData/Reaching")).toURI());
      String demoDirectory = file.getPath();

      System.out.println("Current dir using System: " + demoDirectory);

      List<String> fileList = new ArrayList<>();
      // The trajectories contained in the Reaching1 folder represent different demonstration of a given task
      // Several trajectories of different body parts have been recorded
      // 0: waist Z; 1,2,3: right hand X,Y,Z; 5,6,7: left hand X,Y,Z
      for (int i = 0; i < 10; i++) //get training files
         fileList.add(demoDirectory + "/pr" + (i + 1) + ".csv");
      assertTrue(fileList.size() > 0);
      // consider only right hand trajectories
      List<Long> dofs = List.of(1L, 2L, 3L);

      TrajectoryGroup trajectoryGroup = new TrajectoryGroup();

      StringVector fileListStringVector = new StringVector();
      fileList.forEach(fileListStringVector::push_back);

      SizeTVector doFsSizeTVector = new SizeTVector();
      dofs.forEach(doFsSizeTVector::push_back);

      trajectoryGroup.load_csv_trajectories(fileListStringVector, doFsSizeTVector);
      long t_len = trajectoryGroup.normalize_length();
      assertTrue(t_len > 0);

      int n_rbf = 20;
      ProMP myProMP = new ProMP(trajectoryGroup, n_rbf);
      EigenMatrixXd meanTrajectory = myProMP.generate_trajectory();
      assertEquals(meanTrajectory.cols(), dofs.size());
      assertEquals(meanTrajectory.rows(), t_len);
      EigenMatrixXd stdDeviationTrajectory = myProMP.gen_traj_std_dev();
      assertEquals(stdDeviationTrajectory.cols(), dofs.size());
      assertEquals(stdDeviationTrajectory.rows(), t_len);
      EigenMatrixXd covarianceTrajectory = myProMP.generate_trajectory_covariance();


      if (SAVEFILES)
      {
         TrajectoryVector demoTrajectories = trajectoryGroup.trajectories();
         for (int i = 0; i < demoTrajectories.size(); i++)
         {
            ProMPUtil.saveAsCSV(demoTrajectories.get(i).matrix(), ("/demo" + (i + 1) + ".csv"));
         }
         ProMPUtil.saveAsCSV(meanTrajectory, "/mean.csv");
         ProMPUtil.saveAsCSV(stdDeviationTrajectory, "/stdDeviation.csv");
         ProMPUtil.saveAsCSV(covarianceTrajectory, "/covariance.csv");
      }

      meanTrajectory.debugPrintMatrix("meanTrajectory");
      stdDeviationTrajectory.debugPrintMatrix("stdDeviationTrajectory");
      covarianceTrajectory.debugPrintMatrix("covarianceTrajectory");
   }
}
