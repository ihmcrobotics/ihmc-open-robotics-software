package us.ihmc.atlas.calib;

import org.ddogleg.optimization.functions.FunctionNtoM;
import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.utilities.humanoidRobot.partNames.LimbName;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.ReferenceFrame;

import javax.vecmath.*;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

public class KinematicCalibrationWristLoopResidual implements FunctionNtoM
{
   //robot model and data
   private final SDFFullRobotModel fullRobotModel;
   private final ArrayList<Map<String, Double>> qdata;
   private final ArrayList<String> calJointNames;

   //local data buffer
   Map<String, Double> qoffset = new HashMap<>(), qbuffer = new HashMap<>();
   Vector3d constantOffset = new Vector3d();


   public KinematicCalibrationWristLoopResidual(SDFFullRobotModel fullRobotModel, final ArrayList<String> calJointNames, ArrayList<Map<String, Double>> qdata)
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
      constantOffset.y = input[inputCounter++];
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
         FramePose leftEE = new FramePose(fullRobotModel.getEndEffectorFrame(RobotSide.LEFT, LimbName.ARM), new Point3d(+0.01, +0.13, 0), CalibUtil.quat0);
         FramePose rightEE = new FramePose(fullRobotModel.getEndEffectorFrame(RobotSide.RIGHT, LimbName.ARM), new Point3d(+0.01, -0.13, 0), CalibUtil.quat0);
         leftEE.addPositionInFrame(constantOffset);

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
               Quat4d qErr = leftEE.getOrientationCopy().getQuaternionCopy();
               qErr.inverse();
               qErr.mul(rightEE.getOrientationCopy().getQuaternionCopy());
               //qErr.normalize();
               AxisAngle4d axErr = new AxisAngle4d();
               axErr.set(qErr);
               output[outputCounter++] = scaleRadToCM * axErr.getX() * axErr.getAngle();
               output[outputCounter++] = scaleRadToCM * axErr.getY() * axErr.getAngle();
               output[outputCounter++] = scaleRadToCM * axErr.getZ() * axErr.getAngle();
            } else
            {
               assert (leftEE.getReferenceFrame() == rightEE.getReferenceFrame());
               Matrix3d mLeft = leftEE.getOrientationCopy().getMatrix3dCopy();
               Matrix3d mRight = rightEE.getOrientationCopy().getMatrix3dCopy();
               Vector3d vDiff = CalibUtil.RotationDiff(mLeft, mRight);
               output[outputCounter++] = scaleRadToCM * vDiff.x;
               output[outputCounter++] = scaleRadToCM * vDiff.y;
               output[outputCounter++] = scaleRadToCM * vDiff.z;
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