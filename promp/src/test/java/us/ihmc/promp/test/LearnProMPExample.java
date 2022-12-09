package us.ihmc.promp.test;

import us.ihmc.promp.*;
import us.ihmc.tools.io.WorkspaceDirectory;

import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;
import static us.ihmc.promp.global.promp.EigenMatrixXd;

/**
 * This example shows you how to load training trajectories and learn a multidimensional ProMP for the specified dofs.
 * The ProMP can be intuitively analyzed by plotting its mean trajectory and std deviation.
 */
public class LearnProMPExample
{
   @Test
   public void testLearningProMP()
   {
      ProMPNativeLibrary.load();

      WorkspaceDirectory demoDir = new WorkspaceDirectory("ihmc-open-robotics-software", "promp/etc/test/cppLibraryTestData/Reaching");
      String demoDirAbs = demoDir.getDirectoryPath().toAbsolutePath().toString();

      List<String> fileList = new ArrayList<>();
      // The trajectories contained in the Reaching1 folder represent different demonstration of a given task
      // Several trajectories of different body parts have been recorded
      // 0: waist Z; 1,2,3: right hand X,Y,Z; 5,6,7: left hand X,Y,Z
      for (int i = 0; i < 10; i++) //get training files
         fileList.add(demoDirAbs + "/pr" + (i + 1) + ".csv");
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
      assertTrue(meanTrajectory.cols() == dofs.size());
      assertTrue(meanTrajectory.rows() == t_len);
      EigenMatrixXd stdDeviationTrajectory = myProMP.gen_traj_std_dev();
      assertTrue(stdDeviationTrajectory.cols() == dofs.size());
      assertTrue(stdDeviationTrajectory.rows() == t_len);
      EigenMatrixXd covarianceTrajectory = myProMP.generate_trajectory_covariance();

      TrajectoryVector demoTrajectories = trajectoryGroup.trajectories();
      for (int i = 0; i < demoTrajectories.size(); i++)
      {
         ProMPUtil.saveAsCSV(demoTrajectories.get(i).matrix(), ("/demo" + (i + 1) + ".csv"));
      }
      ProMPUtil.saveAsCSV(meanTrajectory, "/mean.csv");
      ProMPUtil.saveAsCSV(stdDeviationTrajectory, "/stdDeviation.csv");
      ProMPUtil.saveAsCSV(covarianceTrajectory, "/covariance.csv");

      meanTrajectory.debugPrintMatrix("meanTrajectory");
      stdDeviationTrajectory.debugPrintMatrix("stdDeviationTrajectory");
      covarianceTrajectory.debugPrintMatrix("covarianceTrajectory");
   }
}
