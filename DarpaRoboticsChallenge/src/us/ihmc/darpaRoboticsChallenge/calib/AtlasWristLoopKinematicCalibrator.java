package us.ihmc.darpaRoboticsChallenge.calib;

import java.awt.Color;
import java.io.BufferedReader;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;

import us.ihmc.commonWalkingControlModules.partNamesAndTorques.LimbName;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearanceRGBColor;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.utilities.math.geometry.FrameOrientation;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.OneDoFJoint;

import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicCoordinateSystem;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicPosition;
import com.yobotics.simulationconstructionset.util.math.frames.YoFramePoint;
import com.yobotics.simulationconstructionset.util.math.frames.YoFramePose;

public class AtlasWristLoopKinematicCalibrator extends AtlasCalibrationDataViewer
{

   public AtlasWristLoopKinematicCalibrator()
   {
      super();
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
   
   public KinematicCalibrationWristLoopResidual getArmLoopResidualObject()
   {
      ArrayList<String> calJointNames = CalibUtil.toStringArrayList(getArmJoints());
      return new KinematicCalibrationWristLoopResidual(fullRobotModel, calJointNames, (ArrayList)q);
   }
   
   
   
   public static void main(String[] arg) 
   {
      AtlasWristLoopKinematicCalibrator calib = new AtlasWristLoopKinematicCalibrator();
      calib.loadData("data/coupledWristLog_20131204");
      
      // calJointNames order is the prm order
      KinematicCalibrationWristLoopResidual residualFunc = calib.getArmLoopResidualObject();
      double[] prm = new double[residualFunc.getN()];
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
      boolean start_scs=false;
      if(start_scs)
      {
         //Yovariables for display
         YoFramePose yoResidual0 = new YoFramePose("residual0", "", ReferenceFrame.getWorldFrame(),calib.registry);
         YoFramePose yoResidual = new YoFramePose("residual", "",ReferenceFrame.getWorldFrame(),calib.registry);

         calib.createDisplay();
         
         for(int i=0;i<calib.q.size();i++)
         {
            CalibUtil.setRobotModelFromData(calib.fullRobotModel, CalibUtil.addQ((Map)calib.q.get(i),qoffset));
            yoResidual0.setXYZYawPitchRoll(Arrays.copyOfRange(residual0, i*RESIDUAL_DOF, i*RESIDUAL_DOF+6));
            yoResidual.setXYZYawPitchRoll(Arrays.copyOfRange(residual, i*RESIDUAL_DOF, i*RESIDUAL_DOF+6));         
            calib.displayUpdate();
         }
      } 
      
   }
}
