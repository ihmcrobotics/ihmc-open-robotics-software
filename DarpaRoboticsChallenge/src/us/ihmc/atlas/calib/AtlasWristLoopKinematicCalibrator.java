package us.ihmc.atlas.calib;

import com.yobotics.simulationconstructionset.DataBuffer;
import com.yobotics.simulationconstructionset.IndexChangedListener;
import com.yobotics.simulationconstructionset.util.math.frames.YoFramePose;

import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.LimbName;
import us.ihmc.darpaRoboticsChallenge.DRCLocalConfigParameters;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.OneDoFJoint;

import javax.vecmath.Point3d;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Map;

public class AtlasWristLoopKinematicCalibrator extends AtlasCalibrationDataViewer
{
   public AtlasWristLoopKinematicCalibrator(AtlasRobotVersion atlasVersion, boolean runningOnRealRobot)
   {
      super(atlasVersion, runningOnRealRobot);
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
      CalibUtil.setRobotModelFromData(fullRobotModel, (Map) q.get(index));
      FramePose leftEE = new FramePose(fullRobotModel.getEndEffectorFrame(RobotSide.LEFT, LimbName.ARM), new Point3d(+0.00179, +0.13516, +0.01176), CalibUtil.quat0);
      FramePose rightEE = new FramePose(fullRobotModel.getEndEffectorFrame(RobotSide.RIGHT, LimbName.ARM), new Point3d(+0.00179,-0.13516, -0.01176), CalibUtil.quat0);

      leftEE.changeFrame(ReferenceFrame.getWorldFrame());
      rightEE.changeFrame(ReferenceFrame.getWorldFrame());
      {

         System.out.println("r_axLeft: " + CalibUtil.Matrix3dToAxisAngle3d(leftEE.getOrientationCopy().getMatrix3d()));
         System.out.println("r_axRight: " + CalibUtil.Matrix3dToAxisAngle3d(rightEE.getOrientationCopy().getMatrix3d()));
         System.out.println("r_axDiff: " + CalibUtil.RotationDiff(
               leftEE.getOrientationCopy().getMatrix3d(),
               rightEE.getOrientationCopy().getMatrix3d()));
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
	  final AtlasRobotVersion ATLAS_ROBOT_VERSION = AtlasRobotVersion.DRC_NO_HANDS;
	  final boolean RUNNING_ON_REAL_ROBOT = DRCLocalConfigParameters.RUNNING_ON_REAL_ROBOT;

      AtlasWristLoopKinematicCalibrator calib = new AtlasWristLoopKinematicCalibrator(ATLAS_ROBOT_VERSION, RUNNING_ON_REAL_ROBOT);
      //calib.loadData("data/coupledWristLog_20131204");
      calib.loadData("data/coupledWristLog_20131206_1");

      // calJointNames order is the prm order
      ArrayList<String> calJointNames = CalibUtil.toStringArrayList(calib.getArmJoints());
      KinematicCalibrationWristLoopResidual residualFunc = new KinematicCalibrationWristLoopResidual(calib.fullRobotModel, calJointNames, (ArrayList) calib.q);

      double[] prm = new double[residualFunc.getNumOfInputsN()];

      //initial
      double[] residual0 = residualFunc.calcResiduals(prm);
      calib.calibrate(residualFunc, prm, 100);
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
         YoFramePose yoResidual0 = new YoFramePose("residual0", "", ReferenceFrame.getWorldFrame(), calib.registry);
         YoFramePose yoResidual = new YoFramePose("residual", "", ReferenceFrame.getWorldFrame(), calib.registry);

         calib.createDisplay(calib.q.size());
         calib.attachIndexChangedListener();
         calib.createQoutYoVariables();

         for (int i = 0; i < calib.q.size(); i++)
         {
            CalibUtil.setRobotModelFromData(calib.fullRobotModel, CalibUtil.addQ((Map) calib.q.get(i), qoffset));
            yoResidual0.setXYZYawPitchRoll(Arrays.copyOfRange(residual0, i * RESIDUAL_DOF, i * RESIDUAL_DOF + 6));
            yoResidual.setXYZYawPitchRoll(Arrays.copyOfRange(residual, i * RESIDUAL_DOF, i * RESIDUAL_DOF + 6));
            calib.updateQoutYoVariables(i);
            calib.displayUpdate(i);
         }
      }

   }
}
