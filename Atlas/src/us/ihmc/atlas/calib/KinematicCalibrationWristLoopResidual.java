package us.ihmc.atlas.calib;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

import org.ddogleg.optimization.functions.FunctionNtoM;

import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.partNames.LimbName;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;

public class KinematicCalibrationWristLoopResidual implements FunctionNtoM
{
   //robot model and data
   private final FullHumanoidRobotModel fullRobotModel;
   private final ArrayList<Map<String, Double>> qdata;
   private final ArrayList<String> calJointNames;

   //local data buffer
   Map<String, Double> qoffset = new HashMap<>(), qbuffer = new HashMap<>();
   Vector3D constantOffset = new Vector3D();


   public KinematicCalibrationWristLoopResidual(FullHumanoidRobotModel fullRobotModel, final ArrayList<String> calJointNames, ArrayList<Map<String, Double>> qdata)
   {
      this.fullRobotModel = fullRobotModel;
      this.calJointNames = calJointNames;
      this.qdata = qdata;
   }

   public Map<String, Double> prmArrayToJointMap(double[] prm)
   {
      Map<String, Double> qret = new HashMap<>();
      assert (prm.length == calJointNames.size());
      for (int i = 0; i < calJointNames.size(); i++)
      {
         qret.put(calJointNames.get(i), prm[i]);
      }
      return qret;
   }

   public double[] calcResiduals(double[] input)
   {
      double[] output = new double[getNumOfOutputsM()];
      process(input, output);
      return output;
   }


   @Override
   public void process(double[] input, double[] output)
   {
      //convert input into map
      int inputCounter = 0;
      for (int i = 0; i < calJointNames.size(); i++)
         qoffset.put(calJointNames.get(i), input[inputCounter++]);

      //remember to change getN()
//    constantOffset.x=input[inputCounter++];
      constantOffset.setY(input[inputCounter++]);
//    constantOffset.z=input[inputCounter++];

      //compute error            
      int outputCounter = 0;
      for (int i = 0; i < qdata.size(); i++)
      {
         CalibUtil.addQ(qdata.get(i), qoffset, qbuffer);
         CalibUtil.setRobotModelFromData(fullRobotModel, qbuffer);

//         FramePoint 
//            leftEE=new FramePoint(fullRobotModel.getEndEffectorFrame(RobotSide.LEFT, LimbName.ARM)  ,+0.01, 0.13,0),
//            rightEE=new FramePoint(fullRobotModel.getEndEffectorFrame(RobotSide.RIGHT, LimbName.ARM),+0.01,-0.13,0);
         FramePose leftEE = new FramePose(fullRobotModel.getEndEffectorFrame(RobotSide.LEFT, LimbName.ARM), new Point3D(+0.01, +0.13, 0), CalibUtil.quat0);
         FramePose rightEE = new FramePose(fullRobotModel.getEndEffectorFrame(RobotSide.RIGHT, LimbName.ARM), new Point3D(+0.01, -0.13, 0), CalibUtil.quat0);
         leftEE.translate(constantOffset);

         leftEE.changeFrame(ReferenceFrame.getWorldFrame());
         rightEE.changeFrame(ReferenceFrame.getWorldFrame());
         output[outputCounter++] = leftEE.getX() - rightEE.getX();
         output[outputCounter++] = leftEE.getY() - rightEE.getY();
         output[outputCounter++] = leftEE.getZ() - rightEE.getZ();

         if (AtlasKinematicCalibrator.RESIDUAL_DOF == 6)
         {
            boolean QUAT_DIFF = false;
            double scaleRadToCM = 0.01 / (Math.PI / 8); //30deg -> 1cm
            if (QUAT_DIFF)
            {
               Quaternion leftEEQuat = new Quaternion();
               Quaternion rightEEQuat = new Quaternion();
               leftEE.getOrientation(leftEEQuat);
               rightEE.getOrientation(rightEEQuat);
               Quaternion qErr = new Quaternion(leftEEQuat);
               qErr.inverse();
               qErr.multiply(rightEEQuat);
               //qErr.normalize();
               AxisAngle axErr = new AxisAngle();
               axErr.set(qErr);
               output[outputCounter++] = scaleRadToCM * axErr.getX() * axErr.getAngle();
               output[outputCounter++] = scaleRadToCM * axErr.getY() * axErr.getAngle();
               output[outputCounter++] = scaleRadToCM * axErr.getZ() * axErr.getAngle();
            }
            else
            {
               assert (leftEE.getReferenceFrame() == rightEE.getReferenceFrame());
               RotationMatrix mLeft = new RotationMatrix();
               RotationMatrix mRight = new RotationMatrix();
               leftEE.getOrientation(mLeft);
               rightEE.getOrientation(mRight);
               Vector3D vDiff = CalibUtil.rotationDiff(mLeft, mRight);
               output[outputCounter++] = scaleRadToCM * vDiff.getX();
               output[outputCounter++] = scaleRadToCM * vDiff.getY();
               output[outputCounter++] = scaleRadToCM * vDiff.getZ();
            }
         }
      }
   }


   @Override
   public int getNumOfInputsN()
   {
      //dim parameter
      return calJointNames.size() + 1; //+3 for constant offset
   }

   @Override
   public int getNumOfOutputsM()
   {
      //dim error
      return qdata.size() * AtlasKinematicCalibrator.RESIDUAL_DOF;
   }
}