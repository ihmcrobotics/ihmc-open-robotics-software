package us.ihmc.atlas.calib;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.ddogleg.optimization.functions.FunctionNtoM;

import boofcv.abst.fiducial.calib.CalibrationDetectorChessboard;
import boofcv.alg.geo.PerspectiveOps;
import boofcv.struct.calib.IntrinsicParameters;
import georegression.struct.point.Point2D_F64;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.robotics.partNames.LimbName;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.OneDoFJoint;

/**
 *
 */
public class KinematicCalibrationHeadLoopResidual implements FunctionNtoM
{
   // which side of the robot is being optimized
   boolean isLeft;

   //robot model and data
   private final FullHumanoidRobotModel fullRobotModel;
   private final ArrayList<Map<String, Object>> qdata;
   private final ArrayList<Map<String, Double>> q;
   private final List<String> calJointNames;

   //local data buffer
   Map<String, Double> qoffset = new HashMap<>(), qbuffer = new HashMap<>();

   IntrinsicParameters intrinsic;
   CalibrationDetectorChessboard calibGrid;

   // normial orientation of the target.  only rotation around y is optimized
   public static final RotationMatrix TARGET_LEFT_ROT = new RotationMatrix(
         -1.0, 0.0, 0.0,
         0.0, 0.0, 1.0,
         0.0, 1.0, 0.0);

   public static final RotationMatrix TARGET_RIGHT_ROT = new RotationMatrix(
          0, 1, 0,
          0, 0,-1,
         -1, 0, 0);

   public final RobotSide activeSide;

   public KinematicCalibrationHeadLoopResidual(FullHumanoidRobotModel fullRobotModel,
                                               boolean isLeft,
                                               IntrinsicParameters intrinsic,
                                               CalibrationDetectorChessboard calibGrid,
                                               ArrayList<Map<String, Object>> qdata,
                                               ArrayList<Map<String, Double>> q)
   {
      this.fullRobotModel = fullRobotModel;
      this.qdata = qdata;
      this.q = q;
      this.intrinsic = intrinsic;
      this.calibGrid = calibGrid;
      this.isLeft = isLeft;
      this.activeSide = isLeft ? RobotSide.LEFT : RobotSide.RIGHT;

      this.calJointNames = getOrderedArmJointsNames(fullRobotModel, isLeft);
   }

   public static List<String> getOrderedArmJointsNames(FullRobotModel fullRobotModel, boolean isLeft)
   {
      final OneDoFJoint[] joints = fullRobotModel.getOneDoFJoints();

      String filter = isLeft ? "l_arm" : "r_arm";

      ArrayList<OneDoFJoint> armJoints = new ArrayList<OneDoFJoint>();
      for (int i = 0; i < joints.length; i++)
      {
         if (joints[i].getName().contains(filter))
         {
            armJoints.add(joints[i]);
         } else if (joints[i].getName().contains("neck"))
         {
            armJoints.add(joints[i]);
         }

      }
      List<String> ret = CalibUtil.toStringArrayList(armJoints);
      Collections.sort(ret);
      return ret;
   }

   @Override
   public void process(double[] input, double[] output)
   {
      //convert input into map
      int inputCounter = 0;
      for (int i = 0; i < calJointNames.size(); i++)
         qoffset.put(calJointNames.get(i), input[inputCounter++]);

      RigidBodyTransform targetToEE = computeTargetToEE(input, inputCounter,isLeft);

      ReferenceFrame cameraFrame = fullRobotModel.getCameraFrame("stereo_camera_left");
      RigidBodyTransform imageToCamera = new RigidBodyTransform(new double[]{0, 0, 1, 0, -1, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 1});
      ReferenceFrame cameraImageFrame = ReferenceFrame.
            constructBodyFrameWithUnchangingTransformToParent("cameraImage", cameraFrame, imageToCamera);

      //compute error
      int offset = 0;
      for (int i = 0; i < qdata.size(); i++)
      {
         CalibUtil.addQ(q.get(i), qoffset, qbuffer);
         CalibUtil.setRobotModelFromData(fullRobotModel, qbuffer);

         List<Point2D_F64> observations = (List) qdata.get(i).get(AtlasHeadLoopKinematicCalibrator.CHESSBOARD_DETECTIONS_KEY);

         RigidBodyTransform targetToCamera = computeKinematicsTargetToCamera(cameraImageFrame, targetToEE);

         computeError(observations, targetToCamera, output, offset);

         offset += observations.size() * 2;
      }
   }

   public static RigidBodyTransform computeTargetToEE(double param[], int offset, boolean isLeft)
   {
      // Apply rotation around Y to the nominal rotation
      Vector3D tran = new Vector3D();

      tran.set(param[offset], param[offset + 1], param[offset + 2]);
      AxisAngle axisY = new AxisAngle(new Vector3D(0, 1, 0), param[offset + 3]);
      RotationMatrix matAxisY = new RotationMatrix();
      matAxisY.set(axisY);

      RotationMatrix rotFull = new RotationMatrix();
      RotationMatrix targetRotation = isLeft ? TARGET_LEFT_ROT : TARGET_RIGHT_ROT;
      rotFull.set(matAxisY);
      rotFull.multiply(targetRotation);

      RigidBodyTransform targetToEE = new RigidBodyTransform();
      targetToEE.setTranslation(tran);
      targetToEE.setRotation(rotFull);

      return targetToEE;
   }

   private RigidBodyTransform computeKinematicsTargetToCamera(ReferenceFrame cameraImageFrame, RigidBodyTransform targetToEE)
   {

      ReferenceFrame activeArmEEFrame = fullRobotModel.getEndEffectorFrame(activeSide, LimbName.ARM);
      ReferenceFrame boardFrame = ReferenceFrame.constructBodyFrameWithUnchangingTransformToParent("boardFrame", activeArmEEFrame, targetToEE);
      return boardFrame.getTransformToDesiredFrame(cameraImageFrame);
   }

   private void computeError(List<Point2D_F64> observations, RigidBodyTransform targetToCamera, double[] output, int offset)
   {

      // Points in chessboard frame
      Point2D_F64 norm = new Point2D_F64();
      Point2D_F64 expectedPixel = new Point2D_F64();

      for (int i = 0; i < calibGrid.getLayout().size(); i++)
      {
         Point2D_F64 p = calibGrid.getLayout().get(i);

         // convert to camera frame
         Point3D p3 = new Point3D(p.x, p.y, 0);
         targetToCamera.transform(p3);

         // convert to pixels
         norm.set(p3.getX() / p3.getZ(), p3.getY() / p3.getZ());
         PerspectiveOps.convertNormToPixel(intrinsic, norm, expectedPixel);

         Point2D_F64 observedPixel = observations.get(i);

         // Errors
         output[i * 2 + offset] = observedPixel.x - expectedPixel.x;
         output[i * 2 + offset + 1] = observedPixel.y - expectedPixel.y;
      }
   }

   public List<String> getCalJointNames()
   {
      return calJointNames;
   }

   @Override
   public int getNumOfInputsN()
   {
      //dim parameter
      return calJointNames.size() + 4;
   }

   @Override
   public int getNumOfOutputsM()
   {
      return qdata.size() * calibGrid.getLayout().size() * 2;
   }
}