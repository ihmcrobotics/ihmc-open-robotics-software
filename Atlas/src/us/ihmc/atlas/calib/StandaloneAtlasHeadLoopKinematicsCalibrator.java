package us.ihmc.atlas.calib;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import boofcv.abst.fiducial.calib.CalibrationDetectorChessboard;
import boofcv.abst.fiducial.calib.ConfigChessboard;
import boofcv.factory.calib.FactoryCalibrationTarget;
import org.ddogleg.optimization.FactoryOptimization;
import org.ddogleg.optimization.UnconstrainedLeastSquares;
import org.ddogleg.optimization.UtilOptimize;

import boofcv.io.UtilIO;
import boofcv.struct.calib.IntrinsicParameters;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.OneDoFJoint;

/**
 * @author Peter Abeles
 */
public class StandaloneAtlasHeadLoopKinematicsCalibrator
{
   public static final String DATA_NAME = "DATA_NAME";
   public static final boolean useQOut = false;
   public static final boolean useLeftArm = false;

   private final ArrayList<Map<String, Object>> metaData;
   final ReferenceFrame cameraFrame;

   protected final Map<String, Double> qbias = new HashMap<>();
   protected final FullHumanoidRobotModel fullRobotModel;

   protected final OneDoFJoint[] joints;
   protected final ArrayList<Map<String, Double>> q = new ArrayList<>();
   protected final ArrayList<Map<String, Double>> qout = new ArrayList<>();


   private IntrinsicParameters intrinsic;
   private CalibrationDetectorChessboard calibGrid = FactoryCalibrationTarget.detectorChessboard(new ConfigChessboard(DetectChessboardInKinematicsData.boardWidth, DetectChessboardInKinematicsData.boardHeight, 0.03));

   public StandaloneAtlasHeadLoopKinematicsCalibrator(AtlasRobotVersion atlasVersion)
   {
      //load robot
	  DRCRobotModel robotModel = new AtlasRobotModel(atlasVersion, DRCRobotModel.RobotTarget.SCS, false);
      fullRobotModel = robotModel.createFullRobotModel();
      joints = fullRobotModel.getOneDoFJoints();

      cameraFrame = fullRobotModel.getCameraFrame("stereo_camera_left");
      metaData = new ArrayList<>();
   }

   private void computePerImageError(double output[])
   {
      int numImages = output.length / (2 * calibGrid.getLayout().size());


      int index = 0;
      for (int i = 0; i < numImages; i++)
      {
         double averageError = 0;
         for (int j = 0; j < calibGrid.getLayout().size(); j++)
         {
            double x = output[index++];
            double y = output[index++];

            double r = Math.sqrt(x * x + y * y);

            averageError += r;
         }

         averageError /= calibGrid.getLayout().size();

         String dataName = (String) metaData.get(i).get(DATA_NAME);

         System.out.printf("%5d  Image %20s error = %f%n", i, dataName, averageError);
      }
   }

   private void computeErrorStatistics(double[] output, double[] found)
   {
      double averageError = 0;
      double errors[] = new double[output.length / 2];
      for (int i = 0; i < output.length; i += 2)
      {
         double x = output[i];
         double y = output[i + 1];

         double r = Math.sqrt(x * x + y * y);

         averageError += r;
         errors[i / 2] = r;
      }
      averageError /= errors.length;
      Arrays.sort(errors);
      System.out.println();
      System.out.println();
      System.out.println("Average pixel error " + averageError);
      System.out.println("25% pixel error     " + errors[errors.length / 4]);
      System.out.println("50% pixel error     " + errors[errors.length / 2]);
      System.out.println("75% pixel error     " + errors[(int) (errors.length * 0.75)]);
      System.out.println("95% pixel error     " + errors[(int) (errors.length * 0.95)]);
      System.out.println();
      System.out.println();
   }

   private void printCode(java.util.List<String> jointNames, double found[])
   {
      for (int i = 0; i < jointNames.size(); i++)
      {
         System.out.println("jointAngleOffsetPreTransmission.put(AtlasJointId.JOINT_" + jointNames.get(i).toUpperCase() + ", " + found[i] + ");");
      }
   }

   private void initializeWithQ(double input[], KinematicCalibrationHeadLoopResidual function)
   {
      Map<String, Double> dataQ = q.get(0);
      Map<String, Double> dataQout = qout.get(0);

      List<String> keys = function.getCalJointNames();

      for (int i = 0; i < keys.size(); i++)
      {
         String s = keys.get(i);
         double valQ = dataQ.get(s);
         double valQout = dataQout.get(s);

         double difference = valQ - valQout;
         input[i] = difference;
      }
   }

   public void optimizeData()
   {

      ArrayList<Map<String, Double>> jointMeas = useQOut ? qout : q;

      KinematicCalibrationHeadLoopResidual function = new KinematicCalibrationHeadLoopResidual(fullRobotModel, useLeftArm, intrinsic, calibGrid, metaData, jointMeas);

      UnconstrainedLeastSquares optimizer = FactoryOptimization.leastSquaresLM(1e-3, true);

      double input[] = new double[function.getNumOfInputsN()];

      // if using qout you need to give it an initial estimate
      if (useQOut)
      {
         initializeWithQ(input, function);
      }

      optimizer.setFunction(function, null);
      optimizer.initialize(input, 1e-12, 1e-12);

      for (int i = 0; i < 500; i++)
      {
         System.out.println("  optimization step " + i + " error = " + optimizer.getFunctionValue());
         boolean converged = UtilOptimize.step(optimizer);
         if (converged)
         {
            break;
         }
      }

      double found[] = optimizer.getParameters();
      double output[] = new double[function.getNumOfOutputsM()];

      function.process(found, output);

      computeErrorStatistics(output, found);
//      computePerImageError(output);

      java.util.List<String> jointNames = function.getCalJointNames();

      KinematicCalibrationHeadLoopResidual.computeTargetToEE(found, jointNames.size(),useLeftArm);

      for (int i = 0; i < jointNames.size(); i++)
      {
         qbias.put(jointNames.get(i), found[i]);
         System.out.println(jointNames.get(i) + " bias: " + Math.toDegrees(found[i]) + " deg");
      }
      System.out.println("board to wrist tranX: " + found[found.length - 4]);
      System.out.println("board to wrist tranY: " + found[found.length - 3]);
      System.out.println("board to wrist tranZ: " + found[found.length - 2]);
      System.out.println("board to wrist  rotY: " + Math.toDegrees(found[found.length - 1]) + " deg");

      printCode(jointNames, found);
   }

   public void loadData(String directory) throws IOException
   {
      System.out.println("Loading... ");
      intrinsic = UtilIO.loadXML("../DarpaRoboticsChallenge/data/calibration_images/intrinsic_ros.xml");

      File[] files = new File(directory).listFiles();
      if (files == null)
      {
         System.out.println("Cannot list files in " + directory);
         return;
      }

      Arrays.sort(files);

      for (File f : files)
      {
         if (!f.isDirectory())
            continue;
         System.out.println("datafolder:" + f.toString());

         Map<String, Object> mEntry = new HashMap<>();
         Map<String, Double> qEntry = new HashMap<>();
         Map<String, Double> qoutEntry = new HashMap<>();

         if (!AtlasHeadLoopKinematicCalibrator.loadData(f, mEntry, qEntry, qoutEntry, true))
            continue;
         mEntry.put(DATA_NAME, f.getName());

         metaData.add(mEntry);
         q.add(qEntry);
         qout.add(qoutEntry);
      }

      System.out.println("loaded " + q.size() + " data files");
   }

   public static void main(String[] arg) throws InterruptedException, IOException
   {
	  final AtlasRobotVersion ATLAS_ROBOT_MODEL = AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS;
	  
      StandaloneAtlasHeadLoopKinematicsCalibrator calib = new StandaloneAtlasHeadLoopKinematicsCalibrator(ATLAS_ROBOT_MODEL);
//      calib.loadData("data/calibration20131208");
      calib.loadData("data/armCalibratoin20131209/calibration_right");
//      calib.loadData("data/chessboard_joints_20131204");
      calib.optimizeData();
   }
}
