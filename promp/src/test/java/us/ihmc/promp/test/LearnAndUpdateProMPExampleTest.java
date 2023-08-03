package us.ihmc.promp.test;

import us.ihmc.promp.*;

import java.io.File;
import java.net.URISyntaxException;
import java.util.ArrayList;
import java.util.List;
import java.util.Objects;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;
import static us.ihmc.promp.Trajectory.infer_closest_trajectory;
import static us.ihmc.promp.presets.ProMPInfoMapper.EigenMatrixXd;
import static us.ihmc.promp.presets.ProMPInfoMapper.EigenVectorXd;

public class LearnAndUpdateProMPExampleTest
{
   private final boolean SAVEFILES = false;

   @Test
   public void testUpdateProMP() throws URISyntaxException
   {
      ProMPNativeLibrary.load();
      /*
       * Load training and test trajectories
       */

      File fileTraining = new File(Objects.requireNonNull(getClass().getClassLoader().getResource("promp/cppLibraryTestData/Reaching")).toURI());
      String demoDirectoryTraining = fileTraining.getPath();

      File fileTesting = new File(Objects.requireNonNull(getClass().getClassLoader().getResource("promp/cppLibraryTestData/ReachingTest")).toURI());
      String demoDirectoryTesting = fileTesting.getPath();

      List<String> fileListTraining = new ArrayList<>();
      // The trajectories contained in the Reaching1,2 folders represent different demonstration of a given task
      // Several trajectories of different body parts have been recorded
      // 0: waist Z; 1,2,3: right hand X,Y,Z; 5,6,7: left hand X,Y,Z
      for (int i = 0; i < 10; i++) //get training files
         fileListTraining.add(demoDirectoryTraining + "/pr" + (i + 1) + ".csv");
      assertTrue(fileListTraining.size() > 0);

      List<String> fileListTesting = new ArrayList<>();
      for (int i = 0; i < 6; i++) //get testing files
         fileListTesting.add(demoDirectoryTesting + "/pr" + (i + 1) + ".csv");
      assertTrue(fileListTesting.size() > 0);

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
      assertTrue(meanLengthTraining > 0);

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
      assertEquals(meanTrajectory.cols(), dofs.size());
      assertEquals(meanTrajectory.rows(), meanLengthTraining);
      EigenMatrixXd stdDeviationTrajectory = myProMP.gen_traj_std_dev();
      assertEquals(stdDeviationTrajectory.cols(), dofs.size());
      assertEquals(stdDeviationTrajectory.rows(), meanLengthTraining);
      EigenMatrixXd covarianceTrajectory = myProMP.generate_trajectory_covariance();


      if (SAVEFILES)
      {
         /*
          * Create training/demo trajectories vectors for logging and later usage
          */
         TrajectoryVector demoTrajectories = trainingTrajectories.trajectories();
         TrajectoryVector demoTestTrajectories = testingTrajectories.trajectories();
         for (int i = 0; i < demoTrajectories.size(); i++)
         {
            ProMPUtil.saveAsCSV(demoTrajectories.get(i).matrix(), ("/demo" + (i + 1) + ".csv"));
         }
         for (int i = 0; i < demoTestTrajectories.size(); i++)
         {
            ProMPUtil.saveAsCSV(demoTestTrajectories.get(i).matrix(), ("/test" + (i + 1) + ".csv"));
         }
         ProMPUtil.saveAsCSV(meanTrajectory, "/mean.csv");
         ProMPUtil.saveAsCSV(stdDeviationTrajectory, "/stdDeviation.csv");
         ProMPUtil.saveAsCSV(covarianceTrajectory, "/covariance.csv");

         /*
          * Select a demo trajectory and infer modulation of ProMP based on that demo
          * then update ProMP modulation accordingly
          */
         // see timesteps of selected demo trajectory
         long timestepDemo = demoTestTrajectories.get(0).timesteps();
         assertTrue(timestepDemo > 0);
         System.out.println("timestepDemo: " + timestepDemo);
         // infer the new speed for the ProMP based on observed portion of demo trajectory
         //----------------------------- Default way --------------------------------------------------
         int observedTimesteps = (int) timestepDemo / 3;
         // build observed matrix from demo test 1
         EigenMatrixXd observedTrajectory = new EigenMatrixXd(observedTimesteps, (int) meanTrajectory.cols());
         for (int i = 0; i < observedTrajectory.rows(); i++)
            for (int j = 0; j < observedTrajectory.cols(); j++)
               observedTrajectory.apply(i, j).put(demoTestTrajectories.get(0).matrix().coeff(i, j));

         //estimate the demo training trajectory closest to observed data
         int demo = infer_closest_trajectory(observedTrajectory, demoTrajectories);
         System.out.println("Inferred closest demo to current observation: " + (demo + 1));
         assertEquals(8, demo);
         // infer the speed of the estimated demo training trajectory that best matches the observed data
         double inferredSpeed = demoTrajectories.get(demo).infer_speed(observedTrajectory, 0.25, 4.0, 30);
         assertTrue((inferredSpeed <= 1.5) && (inferredSpeed >= 0.75));
         System.out.println("Inferred speed for demo trajectory: " + inferredSpeed);
         // The desired timesteps are given by the actual timesteps of the demo trajectory and the inferred speed
         int inferredTimesteps = (int) (demoTrajectories.get(demo).timesteps() / inferredSpeed);
         // generate ProMP mean trajectory with new time modulation
         EigenMatrixXd stdDeviationTrajectoryModulated = myProMP.gen_traj_std_dev(inferredTimesteps);
         assertEquals(stdDeviationTrajectoryModulated.rows(), inferredTimesteps);
         EigenMatrixXd meanTrajectoryModulated = myProMP.generate_trajectory(inferredTimesteps);
         assertEquals(meanTrajectoryModulated.rows(), inferredTimesteps);
         System.out.println("Inferred timestep: " + inferredTimesteps);
         //--------------------------------!Alternative way ------------------------------------------
         //      // infer the new speed for the ProMP based on observed portion of demo trajectory (goal available)
         //      int observedTimesteps = (int) timestepDemo / 3;
         //      // build observed matrix from demo test 1
         //      EigenMatrixXd observedTrajectory = new EigenMatrixXd(observedTimesteps, (int) meanTrajectory.cols());
         //      for (int i = 0; i < observedTrajectory.rows(); i++)
         //         for (int j = 0; j < observedTrajectory.cols(); j++)
         //            observedTrajectory.apply(i, j).put(demoTestTrajectories.get(0).matrix().coeff(i, j));
         //      //create copy of promp
         //      // NOTE. we do not want to condition a ProMP that will be modulated afterwards, this will likely corrupt the model
         //      ProMP myCopyProMP = new ProMP(trainingTrajectories, n_rbf);
         //     //update copy ProMP with conditioning operation according to oberseved goal
         //      EigenMatrixXd viaPointStdDeviation = new EigenMatrixXd(dofs.size(), dofs.size());
         //      for (int i = 0; i < viaPointStdDeviation.rows(); i++)
         //      {
         //         for (int j = 0; j < viaPointStdDeviation.cols(); j++)
         //         {
         //            if (i == j)
         //               viaPointStdDeviation.apply(i, j).put(0.00001); // Preferably keep std low. Lower std -> higher precision but less damping
         //            else
         //               viaPointStdDeviation.apply(i, j).put(0);
         //         }
         //      }
         //      // condition goal of demo 1
         //      myCopyProMP.set_conditioning_ridge_factor(0.0001);
         //      EigenVectorXd viaPoint = new EigenVectorXd(dofs.size());
         //      int conditioningTimestep = (int) demoTestTrajectories.get(0).timesteps() - 1;
         //      for (int i = 0; i < viaPoint.size(); i++)
         //         viaPoint.apply(i).put(demoTestTrajectories.get(0).matrix().coeff(conditioningTimestep, i));
         //      myCopyProMP.condition_goal(viaPoint, viaPointStdDeviation);
         //      EigenMatrixXd meanTrajectoryCopyConditioned = myCopyProMP.generate_trajectory();
         //      Trajectory trajectoryMeanCopyConditioned = new Trajectory(meanTrajectoryCopyConditioned, 1.0);
         //      // infer the speed of the estimated demo training trajectory that best matches the observed data
         //      double inferredSpeed = trajectoryMeanCopyConditioned.infer_speed(observedTrajectory, 0.25, 4.0, 30);
         //      System.out.println("Inferred speed for demo trajectory: " + inferredSpeed);
         //      // The desired timesteps are given by the actual timesteps of the demo trajectory and the inferred speed
         //      int inferredTimesteps = (int) (trajectoryMeanCopyConditioned.timesteps() / inferredSpeed);
         //      // generate ProMP mean trajectory with new time modulation
         //      EigenMatrixXd stdDeviationTrajectoryModulated = myProMP.gen_traj_std_dev(inferredTimesteps);
         //      EigenMatrixXd meanTrajectoryModulated = myProMP.generate_trajectory(inferredTimesteps);
         //      System.out.println("Inferred timestep: " + inferredTimesteps);
         //      ProMPUtil.saveAsCSV(meanTrajectoryCopyConditioned, "/meanPreModulated.csv");
         //-----------------------------------------------------------------------------------
         ProMPUtil.saveAsCSV(meanTrajectoryModulated, "/meanModulated.csv");
         ProMPUtil.saveAsCSV(stdDeviationTrajectoryModulated, "/stdDeviationModulated.csv");
         // update the time modulation of the ProMP object with estimated value
         System.out.println("Old ProMp timestep: " + myProMP.get_traj_length());
         myProMP.update_time_modulation((double) myProMP.get_traj_length() / inferredTimesteps);
         assertEquals(myProMP.get_traj_length(), inferredTimesteps);
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
                  viaPointStdDeviation.apply(i, j).put(0.00001); // Preferably keep std low. Lower std -> higher precision but less damping
               else
                  viaPointStdDeviation.apply(i, j).put(0);
            }
         }
         // condition point at a general timestep
         myProMP.set_conditioning_ridge_factor(0.0001);
         EigenVectorXd viaPoint = new EigenVectorXd(dofs.size());
         int conditioningTimestep = 350;
         for (int i = 0; i < viaPoint.size(); i++)
            viaPoint.apply(i).put(demoTestTrajectories.get(0).matrix().coeff(conditioningTimestep, i));
         myProMP.condition_via_point(conditioningTimestep, viaPoint, viaPointStdDeviation);
         EigenMatrixXd meanTrajectoryConditioned = myProMP.generate_trajectory();
         for (int i = 0; i < viaPoint.size(); i++)
            assertTrue(meanTrajectoryConditioned.coeff(conditioningTimestep, i) >= viaPoint.coeff(i) - 0.01
                       && meanTrajectoryConditioned.coeff(conditioningTimestep, i) <= viaPoint.coeff(i) + 0.01);
         // condition goal
         conditioningTimestep = myProMP.get_traj_length();
         System.out.print("Conditioning timestep: " + conditioningTimestep);
         System.out.println("; Via point: " + demoTestTrajectories.get(0).matrix().coeff(conditioningTimestep, 0) + " " + demoTestTrajectories.get(0)
                                                                                                                                              .matrix()
                                                                                                                                              .coeff(
                                                                                                                                                    conditioningTimestep,
                                                                                                                                                    1) + " " + demoTestTrajectories.get(0).matrix().coeff(conditioningTimestep, 2));
         for (int i = 0; i < viaPoint.size(); i++)
            viaPoint.apply(i).put(demoTestTrajectories.get(0).matrix().coeff(conditioningTimestep, i));
         myProMP.condition_goal(viaPoint, viaPointStdDeviation);
         //generate updated mean trajectory
         meanTrajectoryConditioned = myProMP.generate_trajectory();
         assertEquals(meanTrajectoryConditioned.rows(), conditioningTimestep);
         for (int i = 0; i < viaPoint.size(); i++)
            assertTrue(meanTrajectoryConditioned.coeff(conditioningTimestep - 1, i) >= viaPoint.coeff(i) - 0.01
                       && meanTrajectoryConditioned.coeff(conditioningTimestep - 1, i) <= viaPoint.coeff(i) + 0.01);
         EigenMatrixXd stdDeviationTrajectoryConditioned = myProMP.gen_traj_std_dev(inferredTimesteps);

         ProMPUtil.saveAsCSV(meanTrajectoryConditioned, "/meanConditioned.csv");
         ProMPUtil.saveAsCSV(stdDeviationTrajectoryConditioned, "/stdDeviationConditioned.csv");
      }
   }
}
