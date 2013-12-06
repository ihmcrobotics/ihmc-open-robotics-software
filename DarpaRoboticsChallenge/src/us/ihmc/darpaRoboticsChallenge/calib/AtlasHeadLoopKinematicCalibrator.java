package us.ihmc.darpaRoboticsChallenge.calib;

import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicCoordinateSystem;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicPosition;
import com.yobotics.simulationconstructionset.util.math.frames.YoFramePoint;
import com.yobotics.simulationconstructionset.util.math.frames.YoFramePose;
import georegression.geometry.RotationMatrixGenerator;
import georegression.struct.se.Se3_F64;
import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.LimbName;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearanceRGBColor;
import us.ihmc.graphics3DAdapter.jme.util.JMEGeometryUtils;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.utilities.math.MatrixTools;
import us.ihmc.utilities.math.geometry.FrameOrientation;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.OneDoFJoint;

import javax.media.j3d.Transform3D;
import javax.vecmath.Matrix3d;
import javax.vecmath.Vector3d;
import java.awt.*;
import java.io.*;
import java.util.*;

import static java.lang.Double.parseDouble;

public class AtlasHeadLoopKinematicCalibrator extends AtlasKinematicCalibrator
{
   public static String TARGET_TO_CAMERA_KEY = "targetToCamera";

   //YoVariables for Display
   private final YoFramePoint ypLeftEE, ypRightEE;
   private final YoFramePose yposeLeftEE, yposeRightEE, yposeBoard;

   public AtlasHeadLoopKinematicCalibrator()
   {
      super();
      ypLeftEE = new YoFramePoint("leftEE", ReferenceFrame.getWorldFrame(), registry);
      ypRightEE = new YoFramePoint("rightEE", ReferenceFrame.getWorldFrame(),registry);
      yposeLeftEE = new YoFramePose("leftPoseEE", "", ReferenceFrame.getWorldFrame(), registry);
      yposeRightEE = new YoFramePose("rightPoseEE", "", ReferenceFrame.getWorldFrame(), registry);
      yposeBoard = new YoFramePose("board","",ReferenceFrame.getWorldFrame(),registry);
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
      DynamicGraphicCoordinateSystem dgPoseBoard = new DynamicGraphicCoordinateSystem("dgposeBoard", yposeBoard, 5*scale);
      scs.addDynamicGraphicObject(dgPoseLeftEE);
      scs.addDynamicGraphicObject(dgPoseRightEE);
      scs.addDynamicGraphicObject(dgPoseBoard);

   }
   
   @Override
   protected void updateDynamicGraphicsObjects(int index)
   {
      FramePoint
      leftEE=new FramePoint(fullRobotModel.getEndEffectorFrame(RobotSide.LEFT, LimbName.ARM)  ,0, 0.13,0),
      rightEE=new FramePoint(fullRobotModel.getEndEffectorFrame(RobotSide.RIGHT, LimbName.ARM),0,-0.13,0);
      

      ypLeftEE.set(leftEE.changeFrameCopy(CalibUtil.world));
      ypRightEE.set(rightEE.changeFrameCopy(CalibUtil.world));
      
      yposeLeftEE.set(new FramePose(leftEE, new FrameOrientation(leftEE.getReferenceFrame())).changeFrameCopy(CalibUtil.world));
      yposeRightEE.set(new FramePose(rightEE,new FrameOrientation(rightEE.getReferenceFrame())).changeFrameCopy(CalibUtil.world));
      updateBoard(index);
   }

   private void updateBoard(int index)
   {
      ReferenceFrame cameraFrame = fullRobotModel.getCameraFrame("stereo_camera_left");
      Transform3D targetToCamera = (Transform3D)q.get(index).get(TARGET_TO_CAMERA_KEY); //in camera frame
      System.out.println(targetToCamera);
      yposeBoard.set(new FramePose(cameraFrame, targetToCamera).changeFrameCopy(CalibUtil.world));
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
   
//   public KinematicCalibrationWristLoopResidual getArmLoopResidualObject()
//   {
//      ArrayList<String> calJointNames = CalibUtil.toStringArrayList(getArmJoints());
//      return new KinematicCalibrationWristLoopResidual(fullRobotModel, calJointNames, q);
//   }
//
   public void loadData(String directory ) throws IOException
   {
      File[] files = new File(directory).listFiles();

      Arrays.sort(files);

      for( File f : files ) {
         if( !f.isDirectory() )
            continue;
         System.out.println("datafolder:" + f.toString());
         File fileTarget = new File(f,"target.txt");

         if( !fileTarget.exists() || fileTarget.length() == 0 )
            continue;


         // parse targetToCamera transform
         BufferedReader reader = new BufferedReader(new FileReader(fileTarget));

         Se3_F64 rotateTarget = new Se3_F64();
         Se3_F64 translateToCamera = new Se3_F64();

         // skip comments
         reader.readLine();
         String row0[] = reader.readLine().split(" ");
         String row1[] = reader.readLine().split(" ");
         String row2[] = reader.readLine().split(" ");

         for( int col = 0; col < 3; col++ ) {
            rotateTarget.getR().set(0,col, parseDouble(row0[col]));
            rotateTarget.getR().set(1,col, parseDouble(row1[col]));
            rotateTarget.getR().set(2,col, parseDouble(row2[col]));
         }

         reader.readLine();
         String s[] = reader.readLine().split(" ");
         translateToCamera.getT().set( parseDouble(s[0]),parseDouble(s[1]),parseDouble(s[2]));

         Se3_F64 foo = rotateTarget.concat(translateToCamera,null);

         System.out.println("T in camera      : " + foo.getT());

         // load joint angles
         Properties properties = new Properties();
         properties.load(new FileReader(new File(f,"q.m")));

         Map<String,Object> entries = new HashMap<>();

         for( Map.Entry e : properties.entrySet() ) {

            entries.put((String)e.getKey(),Double.parseDouble((String)e.getValue()));
         }

         Se3_F64 cameraToEye = new Se3_F64();
         RotationMatrixGenerator.eulerXYZ(-Math.PI / 2, 0, -Math.PI / 2, cameraToEye.getR());

//         cameraToEye.getR().print();
//
//         CommonOps.transpose(cameraToEye.getR());
//
//         System.out.println("targetToCamera");
//         targetToCamera.print();
//         System.out.println("cameraToEye");
//         cameraToEye.print();

         Se3_F64 targetToEye = foo.concat(cameraToEye, null);

         System.out.println("T in camera robot: "+targetToEye.getT());

         Transform3D transform = new Transform3D();

         Matrix3d matrix3d = new Matrix3d();
         Vector3d T = new Vector3d();
         T.x = targetToEye.getT().getX();
         T.y = targetToEye.getT().getY();
         T.z = targetToEye.getT().getZ();

         MatrixTools.denseMatrixToMatrix3d(targetToEye.getR(),matrix3d,0,0);
         transform.setRotation(matrix3d);
         transform.setTranslation(T);

         entries.put(TARGET_TO_CAMERA_KEY,transform);
         q.add(entries);
      }


   }
   
   
   public static void main(String[] arg) throws InterruptedException, IOException
   {
      AtlasHeadLoopKinematicCalibrator calib = new AtlasHeadLoopKinematicCalibrator();
      calib.loadData("data/chessboard_joints_20131204");
      
      // calJointNames order is the prm order
//      KinematicCalibrationWristLoopResidual residualFunc = calib.getArmLoopResidualObject();
//      double[] prm = new double[residualFunc.getN()];
//      double[] residual0 = residualFunc.calcResiduals(prm);
//      calib.calibrate(residualFunc,prm, 100);
//      double[] residual = residualFunc.calcResiduals(prm);
//
//
//      //display prm in readable format
//      Map<String,Double> qoffset= residualFunc.prmArrayToJointMap(prm);
//      for(String jointName: qoffset.keySet())
//      {
//         System.out.println("jointAngleOffsetPreTransmission.put(AtlasJointId.JOINT_" + jointName.toUpperCase()+", "+qoffset.get(jointName)+");");
//         //System.out.println(jointName + " "+ qoffset.get(jointName));
//      }
//      System.out.println("wristSpacing "+prm[prm.length-1]);
//
      //push data to visualizer
      boolean start_scs=true;
      if(start_scs)
      {
         //Yovariables for display
//         YoFramePose yoResidual0 = new YoFramePose("residual0", "", ReferenceFrame.getWorldFrame(),calib.registry);
//         YoFramePose yoResidual = new YoFramePose("residual", "",ReferenceFrame.getWorldFrame(),calib.registry);

         calib.createDisplay();
         
         for(int i=0;i<calib.q.size();i++)
         {
            CalibUtil.setRobotModelFromData(calib.fullRobotModel, (Map)calib.q.get(i) );
//            CalibUtil.setRobotModelFromData(calib.fullRobotModel, CalibUtil.addQ(calib.q.get(i),qoffset));
//            yoResidual0.setXYZYawPitchRoll(Arrays.copyOfRange(residual0, i*RESIDUAL_DOF, i*RESIDUAL_DOF+6));
//            yoResidual.setXYZYawPitchRoll(Arrays.copyOfRange(residual, i*RESIDUAL_DOF, i*RESIDUAL_DOF+6));
            calib.displayUpdate(i);
         }
      } //viz
      
   }
}
