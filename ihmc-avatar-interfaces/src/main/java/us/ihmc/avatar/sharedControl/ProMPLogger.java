package us.ihmc.avatar.sharedControl;

import us.ihmc.promp.*;
import us.ihmc.promp.presets.ProMPInfoMapper;

public class ProMPLogger
{
   public ProMPLogger()
   {
      ProMPNativeLibrary.load();
   }

   public void saveDemosAndLearnedTrajectories(String bodyPart, ProMP learnedProMP, TrajectoryGroup trainingTrajectories)
   {
      TrajectoryVector demoTrajectories = trainingTrajectories.trajectories();
      for (int i = 0; i < demoTrajectories.size(); i++)
      {
         ProMPUtil.saveAsCSV(demoTrajectories.get(i).matrix(), ("/" + bodyPart + "demo" + (i + 1) + ".csv"));
      }
      ProMPInfoMapper.EigenMatrixXd meanTrajectory = learnedProMP.generate_trajectory();
      ProMPInfoMapper.EigenMatrixXd stdDeviationTrajectory = learnedProMP.gen_traj_std_dev();
      ProMPInfoMapper.EigenMatrixXd covarianceTrajectory = learnedProMP.generate_trajectory_covariance();
      ProMPUtil.saveAsCSV(meanTrajectory, "/" + bodyPart + "mean.csv");
      ProMPUtil.saveAsCSV(stdDeviationTrajectory, "/" + bodyPart + "stdDeviation.csv");
      ProMPUtil.saveAsCSV(covarianceTrajectory, "/" + bodyPart + "covariance.csv");
   }

   public void saveModulatedTrajectories(String bodyPart, ProMP modulatedProMP)
   {
      ProMPInfoMapper.EigenMatrixXd meanTrajectory = modulatedProMP.generate_trajectory();
      ProMPInfoMapper.EigenMatrixXd stdDeviationTrajectory = modulatedProMP.gen_traj_std_dev(meanTrajectory.rows());
      ProMPUtil.saveAsCSV(meanTrajectory, "/" + bodyPart + "meanModulated.csv");
      ProMPUtil.saveAsCSV(stdDeviationTrajectory, "/" + bodyPart + "stdDeviationModulated.csv");
   }

   public void saveConditionedTrajectories(String bodyPart, ProMP conditionedProMP, int timestep)
   {
      ProMPInfoMapper.EigenMatrixXd meanTrajectory = conditionedProMP.generate_trajectory();
      ProMPInfoMapper.EigenMatrixXd stdDeviationTrajectory = conditionedProMP.gen_traj_std_dev(meanTrajectory.rows());
      ProMPUtil.saveAsCSV(meanTrajectory, "/" + bodyPart + "meanConditioned" + timestep + ".csv");
      ProMPUtil.saveAsCSV(stdDeviationTrajectory, "/" + bodyPart + "stdDeviationConditioned" + timestep + ".csv");
   }

   public void saveConditionedGoalTrajectories(String bodyPart, ProMP conditionedProMP)
   {
      ProMPInfoMapper.EigenMatrixXd meanTrajectory = conditionedProMP.generate_trajectory();
      ProMPInfoMapper.EigenMatrixXd stdDeviationTrajectory = conditionedProMP.gen_traj_std_dev(meanTrajectory.rows());
      ProMPUtil.saveAsCSV(meanTrajectory, "/" + bodyPart + "meanConditionedGoal.csv");
      ProMPUtil.saveAsCSV(stdDeviationTrajectory, "/" + bodyPart + "stdDeviationConditionedGoal.csv");
   }
}
