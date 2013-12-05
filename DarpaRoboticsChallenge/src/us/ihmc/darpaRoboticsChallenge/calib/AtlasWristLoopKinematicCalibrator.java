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

public class AtlasWristLoopKinematicCalibrator extends AtlasKinematicCalibrator
{
   //YoVariables for Display
   private final YoFramePoint ypLeftEE, ypRightEE;
   private final YoFramePose yposeLeftEE, yposeRightEE;
   
   public AtlasWristLoopKinematicCalibrator()
   {
      super();
      ypLeftEE = new YoFramePoint("leftEE", ReferenceFrame.getWorldFrame(), registry);
      ypRightEE = new YoFramePoint("rightEE", ReferenceFrame.getWorldFrame(),registry);
      yposeLeftEE = new YoFramePose("leftPoseEE", "", ReferenceFrame.getWorldFrame(), registry);
      yposeRightEE = new YoFramePose("rightPoseEE", "", ReferenceFrame.getWorldFrame(), registry);
    }

   @Override
   protected void addDynamicGraphicObjects()
   {
      double transparency = 0.5;
      double scale=0.02;
      DynamicGraphicPosition dgpLeftEE = new DynamicGraphicPosition("dgpLeftEE", ypLeftEE, scale, new YoAppearanceRGBColor(Color.BLUE, transparency));
      DynamicGraphicPosition dgpRightEE = new DynamicGraphicPosition("dgpRightEE", ypRightEE, scale, new YoAppearanceRGBColor(Color.RED, transparency));
      
      scs.addDynamicGraphicObject(dgpLeftEE);
      scs.addDynamicGraphicObject(dgpRightEE);

      DynamicGraphicCoordinateSystem dgPoseLeftEE = new DynamicGraphicCoordinateSystem("dgposeLeftEE", yposeLeftEE, 5*scale);
      DynamicGraphicCoordinateSystem dgPoseRightEE = new DynamicGraphicCoordinateSystem("dgposeRightEE", yposeRightEE, 5*scale);
      scs.addDynamicGraphicObject(dgPoseLeftEE);
      scs.addDynamicGraphicObject(dgPoseRightEE);

   }
   
   @Override
   protected void updateDynamicGraphicsObjects()
   {
      FramePoint
      leftEE=new FramePoint(fullRobotModel.getEndEffectorFrame(RobotSide.LEFT, LimbName.ARM)  ,0, 0.13,0),
      rightEE=new FramePoint(fullRobotModel.getEndEffectorFrame(RobotSide.RIGHT, LimbName.ARM),0,-0.13,0);
      

      ypLeftEE.set(leftEE.changeFrameCopy(CalibUtil.world));
      ypRightEE.set(rightEE.changeFrameCopy(CalibUtil.world));
      
      yposeLeftEE.set(new FramePose(leftEE, new FrameOrientation(leftEE.getReferenceFrame())).changeFrameCopy(CalibUtil.world));
      yposeRightEE.set(new FramePose(rightEE,new FrameOrientation(rightEE.getReferenceFrame())).changeFrameCopy(CalibUtil.world));
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
      return new KinematicCalibrationWristLoopResidual(fullRobotModel, calJointNames, q);
   }
   
   public void loadData(String calib_file)
   {
      
      BufferedReader reader = null;
      try
      {
         reader = new BufferedReader(new FileReader(calib_file));
      }
      catch (FileNotFoundException e1)
      {
         System.out.println("Cannot load calibration file " + calib_file);
         e1.printStackTrace();
      }

      String line;
      final int numJoints = 28;
      System.out.println("total joints should be " + numJoints);
      try
      {
         while ((line = reader.readLine()) != null)
         {
            if (line.matches("^entry.*"))
            {
               Map<String, Double> q_ = new HashMap<>();
               Map<String, Double> qout_ = new HashMap<>();

               for (int i = 0; i < numJoints; i++)
               {
                  line = reader.readLine();
                  if (line != null)
                  {
                     String[] items = line.split("\\s");
                     q_.put(items[0], new Double(items[1]));
                     qout_.put(items[0], new Double(items[2]));
                  }
                  else
                  {
                     System.out.println("One ill-formed data entry");
                     break;
                  }

               }

               if (q_.size() == numJoints)
                  q.add(q_);
               if (qout_.size() == numJoints)
                  qout.add(qout_);
            }
         }
      }
      catch (IOException e1)
      {
         System.err.println("File reading error");
         e1.printStackTrace();
      }
      System.out.println("total entry loaded q/qout " + q.size() + "/" + qout.size());
   }
   
   
   public static void main(String[] arg) throws InterruptedException
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
            CalibUtil.setRobotModelFromData(calib.fullRobotModel, CalibUtil.addQ(calib.q.get(i),qoffset));
            yoResidual0.setXYZYawPitchRoll(Arrays.copyOfRange(residual0, i*RESIDUAL_DOF, i*RESIDUAL_DOF+6));
            yoResidual.setXYZYawPitchRoll(Arrays.copyOfRange(residual, i*RESIDUAL_DOF, i*RESIDUAL_DOF+6));         
            calib.displayUpdate();
         }
      } //viz
      
   }
}
