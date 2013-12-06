package us.ihmc.darpaRoboticsChallenge.calib;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Map;

import javax.vecmath.AxisAngle4d;
import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;

import us.ihmc.commonWalkingControlModules.partNamesAndTorques.LimbName;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.OneDoFJoint;

import com.yobotics.simulationconstructionset.DataBuffer;
import com.yobotics.simulationconstructionset.IndexChangedListener;
import com.yobotics.simulationconstructionset.util.math.frames.YoFramePose;

public class AtlasWristLoopKinematicCalibrator extends AtlasCalibrationDataViewer
{

   public AtlasWristLoopKinematicCalibrator()
   {
      super();
   }
   
   public void attachIndexChangedListener()
   {
      final DataBuffer dataBuffer = scs.getDataBuffer();
      dataBuffer.attachIndexChangedListener(new IndexChangedListener()
      {
         @Override
         public void indexChanged(int newIndex, double newTime)
         {
            System.out.println("showing yoIndex: "+ yoIndex.getIntegerValue() + "newIndex: "+newIndex);
            debugPrint(yoIndex.getIntegerValue());
         }
      });
   }

   private void debugPrint(int index)
   {
      CalibUtil.setRobotModelFromData(fullRobotModel, (Map)q.get(index));
      FramePose leftEE  = new FramePose(fullRobotModel.getEndEffectorFrame(RobotSide.LEFT,  LimbName.ARM),new Point3d(+0.01,+0.13,0), CalibUtil.quat0);
      FramePose rightEE = new FramePose(fullRobotModel.getEndEffectorFrame(RobotSide.RIGHT, LimbName.ARM),new Point3d(+0.01,-0.13,0), CalibUtil.quat0);

      leftEE.changeFrame(ReferenceFrame.getWorldFrame());
      rightEE.changeFrame(ReferenceFrame.getWorldFrame());

      Quat4d qLeft =leftEE.getOrientationCopy().getQuaternion();
      Quat4d qRight =rightEE.getOrientationCopy().getQuaternion();

      AxisAngle4d axLeft = new AxisAngle4d(), axRight = new AxisAngle4d();
      axLeft.set(qLeft);
      axRight.set(qRight);
      System.out.println("qLeft:"+qLeft);
      System.out.println("qRight:"+qRight);
      System.out.println("axLeft:"+axLeft);
      System.out.println("axRight:"+axRight);
      
      Quat4d qDiff = new Quat4d(qLeft);
      qDiff.inverse();
      qDiff.mul(qRight);
      AxisAngle4d axDiff = new AxisAngle4d();
      axDiff.set(qDiff);
      System.out.println("axDiff:"+axDiff);
      System.out.println();
   }

   
   private ArrayList<OneDoFJoint> getArmJoints()
   {
      ArrayList<OneDoFJoint> armJoints = new ArrayList<OneDoFJoint>();
      for(int i=0;i<joints.length;i++)
      {
         if(joints[i].getName().matches(".*arm.*"))
         {
            armJoints.add(joints[i]);
            if(DEBUG)
               System.out.println("arm "+ i + " "+joints[i].getName());
         }
         
      }
      return armJoints;
   }
   
  
   
   
   
   public static void main(String[] arg) 
   {
  
      AtlasWristLoopKinematicCalibrator calib = new AtlasWristLoopKinematicCalibrator();
      calib.loadData("data/coupledWristLog_20131204");
      
      // calJointNames order is the prm order
      ArrayList<String> calJointNames = CalibUtil.toStringArrayList(calib.getArmJoints());
      KinematicCalibrationWristLoopResidual residualFunc = new KinematicCalibrationWristLoopResidual(calib.fullRobotModel, calJointNames, (ArrayList)calib.q);

      double[] prm = new double[residualFunc.getN()];
      
      //initial
      
      double[] residual0 = residualFunc.calcResiduals(prm);
      calib.calibrate(residualFunc,prm, 100);
      double[] residual = residualFunc.calcResiduals(prm);
      

      //display prm in readable format
      Map<String,Double> qoffset= residualFunc.prmArrayToJointMap(prm);
      for(String jointName: qoffset.keySet())
      {
         System.out.println("jointAngleOffsetPreTransmission.put(AtlasJointId.JOINT_" + jointName.toUpperCase()+", "+qoffset.get(jointName)+");");
         //System.out.println(jointName + " "+ qoffset.get(jointName));
      }
      System.out.println("wristSpacing "+prm[prm.length-1]);
      
      //push data to visualizer
      boolean start_scs=true;
      if(start_scs)
      {
         //Yovariables for display
         YoFramePose yoResidual0 = new YoFramePose("residual0", "", ReferenceFrame.getWorldFrame(),calib.registry);
         YoFramePose yoResidual = new YoFramePose("residual", "",ReferenceFrame.getWorldFrame(),calib.registry);
         
         calib.createDisplay(calib.q.size());
         calib.attachIndexChangedListener();
         calib.createQoutYoVariables();
         
         for(int i=0;i<calib.q.size();i++)
         {
            CalibUtil.setRobotModelFromData(calib.fullRobotModel, CalibUtil.addQ((Map)calib.q.get(i),qoffset));
            yoResidual0.setXYZYawPitchRoll(Arrays.copyOfRange(residual0, i*RESIDUAL_DOF, i*RESIDUAL_DOF+6));
            yoResidual.setXYZYawPitchRoll(Arrays.copyOfRange(residual, i*RESIDUAL_DOF, i*RESIDUAL_DOF+6));      
            calib.updateQoutYoVariables(i);
            calib.displayUpdate(i);
         }
      } 
      
   }
}
