package us.ihmc.atlas.calib;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;
import georegression.struct.point.Point2D_F64;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Random;

import javax.vecmath.AxisAngle4d;
import javax.vecmath.Matrix3d;
import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import org.ddogleg.optimization.FactoryOptimization;
import org.ddogleg.optimization.UnconstrainedLeastSquares;
import org.ddogleg.optimization.UtilOptimize;
import org.junit.Ignore;
import org.junit.Test;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.imageProcessing.configuration.ConfigurationLoader;
import us.ihmc.utilities.code.unitTesting.BambooAnnotations.AverageDuration;
import us.ihmc.utilities.code.unitTesting.BambooAnnotations.QuarantinedTest;
import us.ihmc.utilities.humanoidRobot.partNames.LimbName;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.math.geometry.RigidBodyTransform;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.wholeBodyController.DRCRobotJointMap;
import boofcv.alg.geo.PerspectiveOps;
import boofcv.alg.geo.calibration.PlanarCalibrationTarget;
import boofcv.factory.calib.FactoryPlanarCalibrationTarget;
import boofcv.struct.calib.IntrinsicParameters;

/**
 * @author Peter Abeles
 */
public class KinematicCalibrationHeadLoopResidualTest
{
   Random random = new Random(23234);

   IntrinsicParameters intrinsic;
   PlanarCalibrationTarget calibGrid = FactoryPlanarCalibrationTarget.gridChess(
         DetectChessboardInKinematicsData.boardWidth, DetectChessboardInKinematicsData.boardHeight, 0.03);

   private static final DRCRobotModel robotModel = new AtlasRobotModel(AtlasRobotVersion.DRC_NO_HANDS, AtlasRobotModel.AtlasTarget.SIM, false);
   DRCRobotJointMap jointMap = robotModel.getJointMap();
   SDFFullRobotModel fullRobotModel = robotModel.createFullRobotModel();

   RigidBodyTransform imageToCamera = new RigidBodyTransform(new double[]{0, 0, 1, 0, -1, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 1});

   ReferenceFrame cameraFrame = fullRobotModel.getCameraFrame("stereo_camera_left");

   RigidBodyTransform targetToEE;

   ReferenceFrame cameraImageFrame = ReferenceFrame.
         constructBodyFrameWithUnchangingTransformToParent("cameraImage", cameraFrame, imageToCamera);

   boolean isleft = true;
   List<String> calJointNames = KinematicCalibrationHeadLoopResidual.getOrderedArmJointsNames(fullRobotModel, isleft);

   double targetToEE_param[] = new double[]{0.1, -0.1, 0.05, 0.05};

   public KinematicCalibrationHeadLoopResidualTest()
   {
      intrinsic = ConfigurationLoader.loadXML("/us/ihmc/atlas/calib/intrinsic_ros.xml");

      Vector3d tran = new Vector3d(targetToEE_param[0], targetToEE_param[1], targetToEE_param[2]);
      AxisAngle4d axisY = new AxisAngle4d(new Vector3d(0, 1, 0), targetToEE_param[3]);
      Matrix3d matAxisY = new Matrix3d();
      matAxisY.set(axisY);

      Matrix3d rotFull = new Matrix3d();
      rotFull.mul(matAxisY, KinematicCalibrationHeadLoopResidual.TARGET_LEFT_ROT);

      targetToEE = new RigidBodyTransform();
      targetToEE.setTranslation(tran);
      targetToEE.setRotation(rotFull);
   }

   @Ignore
   @QuarantinedTest("Should get this working again.")
	@AverageDuration(duration = 50.0)
	@Test(timeout=300000)
   public void perfect() throws IOException
   {
      // No offsets to make things easy
      Map<String, Double> qoffset = new HashMap<>();

      for (String s : calJointNames)
      {
         qoffset.put(s, 0.0);
      }

      ArrayList<Map<String, Object>> qdata = new ArrayList<>();
      ArrayList<Map<String, Double>> q = new ArrayList<>();

      int numPoses = 6;
      generateData(qoffset, qdata, q, numPoses);

      KinematicCalibrationHeadLoopResidual alg = new KinematicCalibrationHeadLoopResidual(fullRobotModel, isleft, intrinsic, calibGrid, qdata, q);

      assertEquals(qoffset.size() + 4, alg.getNumOfInputsN());
      assertEquals(numPoses * calibGrid.points.size() * 2, alg.getNumOfOutputsM());

      double input[] = new double[alg.getNumOfInputsN()];
      double output[] = new double[alg.getNumOfOutputsM()];

      for (int i = 0; i < input.length; i++)
      {
         input[i] = 0;
      }
      System.arraycopy(targetToEE_param, 0, input, input.length - 4, 4);

      alg.process(input, output);

      // perfect data.  The residual should be zero
      for (int i = 0; i < output.length; i++)
      {
         assertEquals(0, output[i], 1e-8);
      }
   }

   /**
    * Pass it into an optimization function and see if it works
    */

   @Ignore
   @QuarantinedTest("Should get this working again.")
	@AverageDuration(duration = 50.0)
	@Test(timeout=300000)
   public void smallError() throws IOException
   {
      // some small offests, which won't be provided to the algorithm
      Map<String, Double> qoffset = new HashMap<>();

      for (String s : calJointNames)
      {
         qoffset.put(s, random.nextGaussian() * 0.001);
      }

      ArrayList<Map<String, Object>> qdata = new ArrayList<>();
      ArrayList<Map<String, Double>> q = new ArrayList<>();

      int numPoses = 6;
      generateData(qoffset, qdata, q, numPoses);

      KinematicCalibrationHeadLoopResidual alg = new KinematicCalibrationHeadLoopResidual(fullRobotModel, isleft, intrinsic, calibGrid, qdata, q);

      assertEquals(qoffset.size() + 4, alg.getNumOfInputsN());
      assertEquals(numPoses * calibGrid.points.size() * 2, alg.getNumOfOutputsM());

      double input[] = new double[alg.getNumOfInputsN()];
      double output[] = new double[alg.getNumOfOutputsM()];

      for (int i = 0; i < input.length; i++)
      {
         input[i] = 0;
      }
      System.arraycopy(targetToEE_param, 0, input, input.length - 4, 4);

      alg.process(input, output);

      // perfect data.  The residual should be zero
      for (int i = 0; i < output.length; i++)
      {

         // the error should not be nearly zero
         assertTrue(Math.abs(output[i]) > 1e-8);
         // the error should be less than 2 pixels
         assertTrue(Math.abs(output[i]) < 2);
      }
   }

   /**
    * Pass it into an optimization function and see if it works
    */

   @Ignore
   @QuarantinedTest("Should get this working again.")
	@AverageDuration(duration = 50.0)
	@Test(timeout=300000)
   public void optimize() throws IOException
   {
      // some small offests, which won't be provided to the algorithm
      Map<String, Double> qoffset = new HashMap<>();

      for (String s : calJointNames)
      {
         qoffset.put(s, random.nextGaussian() * 0.04);
      }

      ArrayList<Map<String, Object>> qdata = new ArrayList<>();
      ArrayList<Map<String, Double>> q = new ArrayList<>();

      int numPoses = 20;
      generateData(qoffset, qdata, q, numPoses);

      KinematicCalibrationHeadLoopResidual function = new KinematicCalibrationHeadLoopResidual(fullRobotModel, isleft, intrinsic, calibGrid, qdata, q);

      UnconstrainedLeastSquares optimizer = FactoryOptimization.leastSquaresLM(1e-3, true);

      double input[] = new double[function.getNumOfInputsN()];

      optimizer.setFunction(function, null);
      optimizer.initialize(input, 1e-12, 1e-12);

      UtilOptimize.process(optimizer, 500);

      double[] found = optimizer.getParameters();

      for (int i = 0; i < calJointNames.size(); i++)
      {
         double expected = qoffset.get(calJointNames.get(i));
//         System.out.println(calJointNames.get(i)+" expected "+expected+"  found "+found[i]);
         assertEquals(expected, found[i], 1e-5);
      }

      for (int i = 0; i < targetToEE_param.length; i++)
      {
         double expected = targetToEE_param[i];
//         System.out.println(i+" expected "+expected+"  found "+found[i+calJointNames.size()]);
         assertEquals(expected, found[i + calJointNames.size()], 1e-5);
      }

   }

   private void generateData(Map<String, Double> qoffset, List<Map<String, Object>> qdatas, List<Map<String, Double>> qs, int N) throws IOException
   {
      Map<String, Object> seed_qdata = new HashMap<>();
      Map<String, Double> seed_q = new HashMap<>();
      Map<String, Double> seed_qout = new HashMap<>();

      // load data which will act as a seed
      if (!AtlasHeadLoopKinematicCalibrator.loadData(new File("data/chessboard_joints_20131204/1272635929936818000"), seed_qdata, seed_q, seed_qout, false))
         throw new RuntimeException("Couldn't find the data directory");

      for (int i = 0; i < N; i++)
      {
         Map<String, Object> qdata = new HashMap<String, Object>();
         Map<String, Double> q = new HashMap<>();
         Map<String, Double> qactual = new HashMap<>();

         // randomize joint angles around the seed
         for (int j = 0; j < calJointNames.size(); j++)
         {
            double v = seed_q.get(calJointNames.get(j)) + random.nextGaussian() * 0.1;
            double offset = qoffset.get(calJointNames.get(j));
            q.put(calJointNames.get(j), v);
            qactual.put(calJointNames.get(j), v + offset);
         }

         CalibUtil.setRobotModelFromData(fullRobotModel, qactual);

         RigidBodyTransform targetToCamera = computeKinematicsTargetToCamera(fullRobotModel, cameraImageFrame, targetToEE);
         List<Point2D_F64> observations = generatePerfectObservations(targetToCamera);

         qdata.put(AtlasHeadLoopKinematicCalibrator.CHESSBOARD_DETECTIONS_KEY, observations);

         qdatas.add(qdata);
         qs.add(q);
      }
   }

   private RigidBodyTransform computeKinematicsTargetToCamera(SDFFullRobotModel fullRobotModel, ReferenceFrame cameraImageFrame, RigidBodyTransform targetToEE)
   {

      ReferenceFrame leftEEFrame = fullRobotModel.getEndEffectorFrame(RobotSide.LEFT, LimbName.ARM);
      ReferenceFrame boardFrame = ReferenceFrame.constructBodyFrameWithUnchangingTransformToParent("boardFrame", leftEEFrame, targetToEE);
      return boardFrame.getTransformToDesiredFrame(cameraImageFrame);
   }

   private List<Point2D_F64> generatePerfectObservations(RigidBodyTransform targetToCamera)
   {

      List<Point2D_F64> observations = new ArrayList<>();

      // Points in chessboard frame
      Point2D_F64 norm = new Point2D_F64();

      for (int i = 0; i < calibGrid.points.size(); i++)
      {
         Point2D_F64 p = calibGrid.points.get(i);
         // convert to camera frame
         Point3d p3 = new Point3d(p.x, p.y, 0);
         targetToCamera.transform(p3);

         Point2D_F64 observationPixel = new Point2D_F64();

         // convert to pixels
         norm.set(p3.x / p3.z, p3.y / p3.z);
         PerspectiveOps.convertNormToPixel(intrinsic, norm, observationPixel);

         observations.add(observationPixel);
      }

      return observations;
   }
}
