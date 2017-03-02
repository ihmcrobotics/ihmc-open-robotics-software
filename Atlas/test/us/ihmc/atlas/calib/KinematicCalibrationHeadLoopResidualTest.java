package us.ihmc.atlas.calib;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.io.File;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Random;

import org.ddogleg.optimization.FactoryOptimization;
import org.ddogleg.optimization.UnconstrainedLeastSquares;
import org.ddogleg.optimization.UtilOptimize;
import org.junit.Test;

import boofcv.abst.fiducial.calib.CalibrationDetectorChessboard;
import boofcv.abst.fiducial.calib.ConfigChessboard;
import boofcv.alg.geo.PerspectiveOps;
import boofcv.factory.calib.FactoryCalibrationTarget;
import boofcv.io.UtilIO;
import boofcv.struct.calib.IntrinsicParameters;
import georegression.struct.point.Point2D_F64;
import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.partNames.LimbName;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.wholeBodyController.DRCRobotJointMap;

/**
 * @author Peter Abeles
 */
public class KinematicCalibrationHeadLoopResidualTest
{
   Random random = new Random(23234);

   IntrinsicParameters intrinsic;
   CalibrationDetectorChessboard calibGrid = FactoryCalibrationTarget.detectorChessboard(new ConfigChessboard(
         DetectChessboardInKinematicsData.boardWidth, DetectChessboardInKinematicsData.boardHeight, 0.03));

   private static final DRCRobotModel robotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, DRCRobotModel.RobotTarget.SCS, false);
   DRCRobotJointMap jointMap = robotModel.getJointMap();
   FullHumanoidRobotModel fullRobotModel = robotModel.createFullRobotModel();

   RigidBodyTransform imageToCamera = new RigidBodyTransform(new double[]{0, 0, 1, 0, -1, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 1});

   ReferenceFrame cameraFrame = fullRobotModel.getCameraFrame("stereo_camera_left");

   RigidBodyTransform targetToEE;

   ReferenceFrame cameraImageFrame = ReferenceFrame.
         constructBodyFrameWithUnchangingTransformToParent("cameraImage", cameraFrame, imageToCamera);

   boolean isleft = true;
   List<String> calJointNames = KinematicCalibrationHeadLoopResidual.getOrderedArmJointsNames(fullRobotModel, isleft);

   double targetToEE_param[] = new double[]{0.1, -0.1, 0.05, 0.05};

   public static <T> T loadXML(String filename)
   {
      InputStream inputStream = KinematicCalibrationHeadLoopResidualTest.class.getResourceAsStream(filename);
      if (inputStream == null)
         throw new IllegalArgumentException("Can't find resource.");

      return UtilIO.loadXML(new InputStreamReader(inputStream));

   }
   
   
   public KinematicCalibrationHeadLoopResidualTest()
   {
      intrinsic = loadXML("/us/ihmc/atlas/calib/intrinsic_ros.xml");

      Vector3D tran = new Vector3D(targetToEE_param[0], targetToEE_param[1], targetToEE_param[2]);
      AxisAngle axisY = new AxisAngle(new Vector3D(0, 1, 0), targetToEE_param[3]);
      RotationMatrix matAxisY = new RotationMatrix();
      matAxisY.set(axisY);

      RotationMatrix rotFull = new RotationMatrix();
      rotFull.set(matAxisY);
      rotFull.multiply(KinematicCalibrationHeadLoopResidual.TARGET_LEFT_ROT);

      targetToEE = new RigidBodyTransform();
      targetToEE.setTranslation(tran);
      targetToEE.setRotation(rotFull);
   }

   /**
    * Should get this working again.
    * 
    * @throws IOException
    */
	@ContinuousIntegrationTest(estimatedDuration = 50.0, categoriesOverride = IntegrationCategory.EXCLUDE)
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
      assertEquals(numPoses * calibGrid.getLayout().size() * 2, alg.getNumOfOutputsM());

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

   /**
    * Should get this working again.
    * 
    * @throws IOException
    */
	@ContinuousIntegrationTest(estimatedDuration = 50.0, categoriesOverride = IntegrationCategory.EXCLUDE)
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
      assertEquals(numPoses * calibGrid.getLayout().size() * 2, alg.getNumOfOutputsM());

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

   /**
    * Should get this working again.
    * 
    * @throws IOException
    */
	@ContinuousIntegrationTest(estimatedDuration = 50.0, categoriesOverride = IntegrationCategory.EXCLUDE)
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

   private RigidBodyTransform computeKinematicsTargetToCamera(FullHumanoidRobotModel fullRobotModel, ReferenceFrame cameraImageFrame, RigidBodyTransform targetToEE)
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

      for (int i = 0; i < calibGrid.getLayout().size(); i++)
      {
         Point2D_F64 p = calibGrid.getLayout().get(i);
         // convert to camera frame
         Point3D p3 = new Point3D(p.x, p.y, 0);
         targetToCamera.transform(p3);

         Point2D_F64 observationPixel = new Point2D_F64();

         // convert to pixels
         norm.set(p3.getX() / p3.getZ(), p3.getY() / p3.getZ());
         PerspectiveOps.convertNormToPixel(intrinsic, norm, observationPixel);

         observations.add(observationPixel);
      }

      return observations;
   }
}
