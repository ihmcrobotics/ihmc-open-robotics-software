package us.ihmc.darpaRoboticsChallenge.calib;

import boofcv.alg.geo.calibration.PlanarCalibrationTarget;
import boofcv.factory.calib.FactoryPlanarCalibrationTarget;
import boofcv.misc.BoofMiscOps;
import boofcv.struct.calib.IntrinsicParameters;
import org.ddogleg.optimization.FactoryOptimization;
import org.ddogleg.optimization.UnconstrainedLeastSquares;
import org.ddogleg.optimization.UtilOptimize;
import us.ihmc.SdfLoader.JaxbSDFLoader;
import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.darpaRoboticsChallenge.DRCRobotModel;
import us.ihmc.darpaRoboticsChallenge.DRCRobotSDFLoader;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotJointMap;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.OneDoFJoint;

import javax.media.j3d.Transform3D;
import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;

/**
 * @author Peter Abeles
 */
public class StandaloneAtlasHeadLoopKinematicsCalibrator
{
   public static final String DATA_NAME = "DATA_NAME";

   private final ArrayList<Map<String,Object>> metaData;
   final ReferenceFrame cameraFrame;

   Transform3D targetToEE = new Transform3D();

   protected final Map<String, Double> qbias = new HashMap<>();
   protected final SDFFullRobotModel fullRobotModel;

   protected final OneDoFJoint[] joints;
   protected final ArrayList<Map<String, Double>> q = new ArrayList<>();


   private IntrinsicParameters intrinsic;
   private PlanarCalibrationTarget calibGrid = FactoryPlanarCalibrationTarget.gridChess(
         DetectChessboardInKinematicsData.boardWidth, DetectChessboardInKinematicsData.boardHeight, 0.03);

   public StandaloneAtlasHeadLoopKinematicsCalibrator()
   {
      //load robot
      DRCRobotJointMap jointMap = new DRCRobotJointMap(DRCRobotModel.ATLAS_NO_HANDS_ADDED_MASS, false);
      JaxbSDFLoader robotLoader = DRCRobotSDFLoader.loadDRCRobot(jointMap);
      fullRobotModel = robotLoader.createFullRobotModel(jointMap);
      joints = fullRobotModel.getOneDoFJoints();

      cameraFrame = fullRobotModel.getCameraFrame("stereo_camera_left");
      metaData = new ArrayList<>();
   }

   private void computePerImageError( double output[] ) {
      int numImages = output.length/(2*calibGrid.points.size());


      int index = 0;
      for( int i = 0; i < numImages; i++ ) {
         double averageError = 0;
         for( int j = 0; j <calibGrid.points.size(); j++ ){
            double x = output[index++];
            double y = output[index++];

            double r = Math.sqrt(x*x + y*y);

            averageError += r;
         }

         averageError/=calibGrid.points.size();

         String dataName = (String)metaData.get(i).get(DATA_NAME);

         System.out.printf("%5d  Image %20s error = %f\n",i,dataName,averageError);
      }
   }


   private double[] computeErrorStatistics(KinematicCalibrationHeadLoopResidual function, double[] found)
   {
      double output[] = new double[function.getNumOfOutputsM()];
      function.process(found,output);
      double averageError = 0;
      double errors[] = new double[ output.length/2];
      for( int i = 0; i < output.length; i+= 2 ){
         double x = output[i];
         double y = output[i+1];

         double r = Math.sqrt(x*x + y*y);

         averageError += r;
         errors[i/2] = r;
      }
      averageError /= errors.length;
      Arrays.sort(errors);
      System.out.println();
      System.out.println();
      System.out.println("Average pixel error " + averageError);
      System.out.println("25% pixel error     "+errors[errors.length/4]);
      System.out.println("50% pixel error     "+errors[errors.length/2]);
      System.out.println("75% pixel error     "+errors[(int)(errors.length*0.75)]);
      System.out.println("95% pixel error     "+errors[(int)(errors.length*0.95)]);
      System.out.println();
      System.out.println();
      return output;
   }

   public void optimizeData() {
      KinematicCalibrationHeadLoopResidual function = new KinematicCalibrationHeadLoopResidual(fullRobotModel,true,intrinsic,calibGrid,metaData,q);

      UnconstrainedLeastSquares optimizer = FactoryOptimization.leastSquaresLM(1e-3, true);

      double input[] = new double[ function.getNumOfInputsN() ];

      optimizer.setFunction(function,null);
      optimizer.initialize(input,1e-12,1e-12);

      for( int i = 0; i < 500; i++ ) {
         System.out.println("  optimization step "+i+" error = "+optimizer.getFunctionValue());
         boolean converged = UtilOptimize.step(optimizer);
         if( converged ) {
            break;
         }
      }

      double found[] = optimizer.getParameters();

      double[] output = computeErrorStatistics(function, found);

      computePerImageError(output);

      java.util.List<String> jointNames = function.getCalJointNames();

      targetToEE = KinematicCalibrationHeadLoopResidual.computeTargetToEE(found,jointNames.size());

      for(int i=0;i<jointNames.size();i++)
      {
         qbias.put(jointNames.get(i), found[i]);
         System.out.println(jointNames.get(i) + " bias: " + Math.toDegrees(found[i])+" deg");
      }
      System.out.println("board to wrist tranX:" + found[found.length-4]);
      System.out.println("board to wrist tranY:" + found[found.length-3]);
      System.out.println("board to wrist tranZ:" + found[found.length-2]);
      System.out.println("board to wrist  rotY:" + Math.toDegrees(found[found.length-1])+" deg");
   }

   public void loadData(String directory ) throws IOException
   {
      System.out.println("Loading... ");
      intrinsic = BoofMiscOps.loadXML("../DarpaRoboticsChallenge/data/calibration_images/intrinsic_ros.xml");

      File[] files = new File(directory).listFiles();

      Arrays.sort(files);

      for( File f : files ) {
         if( !f.isDirectory() )
            continue;
         System.out.println("datafolder:" + f.toString());

         Map<String,Object> mEntry = new HashMap<String, Object>();
         Map<String,Double> qEntry = new HashMap<>();

         if( !AtlasHeadLoopKinematicCalibrator.loadData(f, mEntry, qEntry, true) )
            continue;
         mEntry.put(DATA_NAME,f.getName());

         metaData.add(mEntry);
         q.add(qEntry);
      }
      System.out.println("loaded "+q.size()+" data files");
   }

   public static void main(String[] arg) throws InterruptedException, IOException
   {
      StandaloneAtlasHeadLoopKinematicsCalibrator calib = new StandaloneAtlasHeadLoopKinematicsCalibrator();
      calib.loadData("data/calibration20131208");
//      calib.loadData("data/chessboard_joints_20131204");
      calib.optimizeData();


   }
}
