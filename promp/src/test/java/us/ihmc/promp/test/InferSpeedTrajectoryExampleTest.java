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
 * It then modulates (changes the speed of) the ProMP mean trajectory and infers the new time modulation (= 1/speed) by observing the modulated trajectory.
 */
public class InferSpeedTrajectoryExampleTest
{
   @Test
   public void testSpeedInference() throws URISyntaxException
   {
      ProMPNativeLibrary.load();

      File fileTraining = new File(Objects.requireNonNull(getClass().getClassLoader().getResource("promp/cppLibraryTestData/Reaching")).toURI());
      String demoDirectory = fileTraining.getPath();

      List<String> fileListTraining = new ArrayList<>();
      // The trajectories contained in the Reaching1,2 folders represent different demonstration of a given task
      // Several trajectories of different body parts have been recorded
      // 0: waist Z; 1,2,3: right hand X,Y,Z; 5,6,7: left hand X,Y,Z
      for (int i = 0; i < 10; i++) //get training files
         fileListTraining.add(demoDirectory + "/pr" + (i + 1) + ".csv");
      assertTrue(fileListTraining.size() > 0);
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
      long meanLengthTraining = trainingTrajectories.normalize_length();
      assertTrue(meanLengthTraining > 0);

      int n_rbf = 20;

      ProMP myProMP = new ProMP(trainingTrajectories, n_rbf);
      EigenMatrixXd meanTrajectory = myProMP.generate_trajectory();
      assertEquals(meanTrajectory.cols(), dofs.size());
      assertEquals(meanTrajectory.rows(), meanLengthTraining);

      // Test speed inference on a trajectory
      Trajectory trajectoryOriginal = new Trajectory(meanTrajectory, 1.0);
      // set desired speed
      long newSpeed = 2;
      System.out.println("newSpeed to guess: " + newSpeed);
      // see current timesteps
      long timestepOriginal = trajectoryOriginal.timesteps();
      // current speed is 1, as set in the trajetcory constructor
      double speedOriginal = trajectoryOriginal.speed();
      System.out.println("speedOriginal: " + speedOriginal);
      System.out.println("timestepOriginal: " + timestepOriginal);
      // Modulate trajectory according to new speed
      // newTimesteps/timestepsOriginal = NewModulation/modulationOriginal = speedOriginal/newSpeed
      Trajectory trajectoryModulated = trajectoryOriginal.modulate(timestepOriginal / newSpeed); //you can modulate the trajectory object
      //      EigenMatrixXd meanModulatedTrajectory = myProMP.generate_trajectory_with_speed(newSpeed); //or directly modulate the mean trajectory of the proMP
      long timestepNew = trajectoryModulated.timesteps();
      assertEquals(timestepNew, timestepOriginal / 2);
      System.out.println("timestepNew: " + timestepNew);
      //      System.out.println("timestepNew: " + meanModulatedTrajectory.rows());

      // infer the new speed for the modulated trajectory
      double inferredSpeed = trajectoryOriginal.infer_speed(trajectoryModulated.matrix(), 0.25, 4.0, 20); //new speed in range 0.25-4
      assertTrue( (inferredSpeed <= newSpeed + 0.1) &&  (inferredSpeed >= newSpeed - 0.1));
      //      double inferredSpeed = trajectoryOriginal.infer_speed(meanModulatedTrajectory, 0.25, 4.0, 20); //new speed in range 0.25-4
      System.out.println("Inferred speed for modulated trajectory: " + inferredSpeed);
   }
}
