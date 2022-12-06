package us.ihmc.behaviors.sharedControl;

import us.ihmc.promp.*;
import us.ihmc.promp.presets.ProMPInfoMapper;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

public class ProMPLogger
{
   private final HashMap<String, List<ProMPInfoMapper.EigenVectorXd>> bodyPartViaPoints = new HashMap<>();

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

   public void saveUpdatedTrajectories(String bodyPart, ProMP myProMP, String updateName)
   {
      ProMPInfoMapper.EigenMatrixXd meanTrajectory = myProMP.generate_trajectory();
      ProMPInfoMapper.EigenMatrixXd stdDeviationTrajectory = myProMP.gen_traj_std_dev(meanTrajectory.rows());
      ProMPUtil.saveAsCSV(meanTrajectory, "/" + bodyPart + "mean" + updateName + ".csv");
      ProMPUtil.saveAsCSV(stdDeviationTrajectory, "/" + bodyPart + "stdDeviation" + updateName + ".csv");
   }

   public void addViaPoint(String bodyPart, ProMPInfoMapper.EigenVectorXd viaPoint)
   {
      if (!bodyPartViaPoints.containsKey(bodyPart))
         bodyPartViaPoints.put(bodyPart, new ArrayList<>());
      bodyPartViaPoints.get(bodyPart).add(viaPoint);
   }

   public void saveViaPoints(String bodyPart)
   {
      ProMPUtil.saveAsCSV(buildViaPointsMatrix(bodyPart), "/" + bodyPart + "viaPoints.csv");
   }

   private ProMPInfoMapper.EigenMatrixXd buildViaPointsMatrix(String bodyPart)
   {
      // build list of viaPoints as matrix: rows = size of List, cols = size of viaPoint EigenVector
      ProMPInfoMapper.EigenMatrixXd viaPointsMatrix = new ProMPInfoMapper.EigenMatrixXd(bodyPartViaPoints.get(bodyPart).size(),
                                                                                        (int) bodyPartViaPoints.get(bodyPart).get(0).size());
      for (int i = 0; i < viaPointsMatrix.rows(); i++)
         for (int j = 0; j < viaPointsMatrix.cols(); j++)
            viaPointsMatrix.apply(i, j).put(bodyPartViaPoints.get(bodyPart).get(i).coeff(j));
      // clean in case you want to store and log other data again in the future
      bodyPartViaPoints.remove(bodyPart);
      return viaPointsMatrix;
   }
}
