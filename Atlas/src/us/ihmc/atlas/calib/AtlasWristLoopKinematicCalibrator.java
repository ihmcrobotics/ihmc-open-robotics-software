package us.ihmc.atlas.calib;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Map;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.math.frames.YoFramePose;
import us.ihmc.robotics.partNames.LimbName;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.simulationconstructionset.DataBuffer;
import us.ihmc.simulationconstructionset.IndexChangedListener;

public class AtlasWristLoopKinematicCalibrator extends AtlasCalibrationDataViewer
{
   public AtlasWristLoopKinematicCalibrator(DRCRobotModel robotModel)
   {
      super(robotModel);
   }

   public void attachIndexChangedListener()
   {
      final DataBuffer dataBuffer = scs.getDataBuffer();
      dataBuffer.attachIndexChangedListener(new IndexChangedListener()
      {
         @Override
         public void indexChanged(int newIndex, double newTime)
         {
            System.out.println("showing yoIndex: " + yoIndex.getIntegerValue() + "newIndex: " + newIndex);
            debugPrint(yoIndex.getIntegerValue());
         }
      });
   }

   private void debugPrint(int index)
   {
      CalibUtil.setRobotModelFromData(fullRobotModel, q.get(index));
      FramePose leftEE = new FramePose(fullRobotModel.getEndEffectorFrame(RobotSide.LEFT, LimbName.ARM), new Point3D(+0.00179, +0.13516, +0.01176), CalibUtil.quat0);
      FramePose rightEE = new FramePose(fullRobotModel.getEndEffectorFrame(RobotSide.RIGHT, LimbName.ARM), new Point3D(+0.00179,-0.13516, -0.01176), CalibUtil.quat0);

      leftEE.changeFrame(ReferenceFrame.getWorldFrame());
      rightEE.changeFrame(ReferenceFrame.getWorldFrame());
      {
         RotationMatrix leftEEOrientation = new RotationMatrix();
         RotationMatrix rightEEOrientation = new RotationMatrix();
         leftEE.getOrientation(leftEEOrientation);
         rightEE.getOrientation(rightEEOrientation);
         System.out.println("r_axLeft: " + CalibUtil.matrix3dToAxisAngle3d(leftEEOrientation));
         System.out.println("r_axRight: " + CalibUtil.matrix3dToAxisAngle3d(rightEEOrientation));
         System.out.println("r_axDiff: " + CalibUtil.rotationDiff(leftEEOrientation, rightEEOrientation));
      }
   }


   private ArrayList<OneDoFJoint> getArmJoints()
   {
      ArrayList<OneDoFJoint> armJoints = new ArrayList<OneDoFJoint>();
      for (int i = 0; i < joints.length; i++)
      {
         if (joints[i].getName().matches(".*arm.*"))
         {
            armJoints.add(joints[i]);
            if (DEBUG)
               System.out.println("arm " + i + " " + joints[i].getName());
         }

      }
      return armJoints;
   }


   public static void main(String[] arg)
   {
	  final AtlasRobotVersion ATLAS_ROBOT_VERSION = AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS;
	  
	  DRCRobotModel robotModel = new AtlasRobotModel(ATLAS_ROBOT_VERSION, DRCRobotModel.RobotTarget.REAL_ROBOT, true);

      AtlasWristLoopKinematicCalibrator calibrator = new AtlasWristLoopKinematicCalibrator(robotModel);
      //calibrator.loadData("data/coupledWristLog_20131204");
      calibrator.loadData("data/coupledWristLog_20131206_1");

      // calJointNames order is the prm order
      ArrayList<String> calibrationJointNames = CalibUtil.toStringArrayList(calibrator.getArmJoints());
      KinematicCalibrationWristLoopResidual residualFunc = new KinematicCalibrationWristLoopResidual(calibrator.fullRobotModel, calibrationJointNames, calibrator.q);

      double[] prm = new double[residualFunc.getNumOfInputsN()];

      //initial
      double[] residual0 = residualFunc.calcResiduals(prm);
      calibrator.calibrate(residualFunc, prm, 100);
      double[] residual = residualFunc.calcResiduals(prm);


      //display prm in readable format
      Map<String, Double> qoffset = residualFunc.prmArrayToJointMap(prm);
      for (String jointName : qoffset.keySet())
      {
         System.out.println("jointAngleOffsetPreTransmission.put(AtlasJointId.JOINT_" + jointName.toUpperCase() + ", " + qoffset.get(jointName) + ");");
         //System.out.println(jointName + " "+ qoffset.get(jointName));
      }
      System.out.println("wristSpacing " + prm[prm.length - 1]);

      //push data to visualizer
      boolean start_scs = true;
      if (start_scs)
      {
         //Yovariables for display
         YoFramePose yoResidual0 = new YoFramePose("residual0", "", ReferenceFrame.getWorldFrame(), calibrator.registry);
         YoFramePose yoResidual = new YoFramePose("residual", "", ReferenceFrame.getWorldFrame(), calibrator.registry);

         calibrator.createDisplay(calibrator.q.size());
         calibrator.attachIndexChangedListener();
         calibrator.createQoutYoVariables();

         for (int i = 0; i < calibrator.q.size(); i++)
         {
            CalibUtil.setRobotModelFromData(calibrator.fullRobotModel, CalibUtil.addQ(calibrator.q.get(i), qoffset));
            yoResidual0.setXYZYawPitchRoll(Arrays.copyOfRange(residual0, i * RESIDUAL_DOF, i * RESIDUAL_DOF + 6));
            yoResidual.setXYZYawPitchRoll(Arrays.copyOfRange(residual, i * RESIDUAL_DOF, i * RESIDUAL_DOF + 6));
            calibrator.updateQoutYoVariables(i);
            calibrator.displayUpdate(i);
         }
      }

   }
}
