package us.ihmc.valkyrieRosControl.impedance;

import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.sensorProcessing.sensorProcessors.OneDoFJointStateReadOnly;
import us.ihmc.sensorProcessing.sensorProcessors.SensorOutputMapReadOnly;
import us.ihmc.sensorProcessing.stateEstimation.IMUSensorReadOnly;
import us.ihmc.valkyrieRosControl.ValkyrieRosControlSensorReader;

import java.util.ArrayList;
import java.util.List;

public class ValkyrieImpedanceStateEstimator
{
   private final FloatingJointBasics rootJoint;
   private final RigidBodyBasics rootBody;
   private final OneDoFJointBasics[] controlledOneDoFJoints;
   private SensorOutputMapReadOnly processedSensorOutputMap;

   private ValkyrieRosControlSensorReader sensorReader;
   private List<? extends IMUSensorReadOnly> imuOutputs;

   private final RotationMatrix rootToWorldFused = new RotationMatrix();
   private final List<RotationMatrix> rootToWorlds = new ArrayList<>();
   private final List<RigidBodyTransform> rootToIMUs = new ArrayList<>();
   private final List<RotationMatrix> imuToWorldRotations = new ArrayList<>();
   private final List<RotationMatrix> rootToIMURotations = new ArrayList<>();

   public ValkyrieImpedanceStateEstimator(FloatingJointBasics rootJoint, OneDoFJointBasics[] controlledOneDoFJoints)
   {
      this.rootJoint = rootJoint;
      this.controlledOneDoFJoints = controlledOneDoFJoints;
      this.rootBody = MultiBodySystemTools.getRootBody(controlledOneDoFJoints[0].getSuccessor());
   }

   public void init(ValkyrieRosControlSensorReader sensorReader)
   {
      this.sensorReader = sensorReader;
      processedSensorOutputMap = sensorReader.getProcessedSensorOutputMap();
      imuOutputs = processedSensorOutputMap.getIMUOutputs();
      sensorReader.initialize();

      if (imuOutputs.size() != 2)
      {
         throw new RuntimeException("Expecting 2 IMU's, but there are " + imuOutputs.size());
      }

      for (int i = 0; i < imuOutputs.size(); i++)
      {
         rootToIMUs.add(new RigidBodyTransform());
         imuToWorldRotations.add(new RotationMatrix());
         rootToIMURotations.add(new RotationMatrix());
         rootToWorlds.add(new RotationMatrix());
      }
   }

   public void update()
   {
      /* First pass computes joint angles */
      sensorReader.readSensors();

      for (int i = 0; i < controlledOneDoFJoints.length; i++)
      {
         OneDoFJointStateReadOnly processedJointOutput = processedSensorOutputMap.getOneDoFJointOutput(controlledOneDoFJoints[i]);
         controlledOneDoFJoints[i].setQ(processedJointOutput.getPosition());
         controlledOneDoFJoints[i].setQd(processedJointOutput.getVelocity());
      }

      rootBody.updateFramesRecursively();

      /* Second pass computes root joint */
      computeRootToIMU(0);
      computeRootToIMU(1);

      rootToWorldFused.interpolate(rootToWorlds.get(0), rootToWorlds.get(1), 0.5);

      double pitch = rootToWorldFused.getPitch();
      double roll = rootToWorldFused.getRoll();
      rootToWorldFused.setYawPitchRoll(0.0, pitch, roll);

      rootJoint.getJointPose().getOrientation().set(rootToWorldFused);
      rootBody.updateFramesRecursively();

      /* Third pass computes vertical offset */
   }

   private void computeRootToIMU(int index)
   {
      IMUSensorReadOnly imuOutput = imuOutputs.get(index);
      RigidBodyTransform rootToIMU = rootToIMUs.get(index);
      RotationMatrix imuToWorldRotation = imuToWorldRotations.get(index);
      RotationMatrix rootToIMURotation = rootToIMURotations.get(index);
      RotationMatrix rootToWorld = rootToWorlds.get(index);

      rootJoint.getFrameAfterJoint().getTransformToDesiredFrame(rootToIMU, imuOutput.getMeasurementFrame());
      rootToIMURotation.set(rootToIMU.getRotation());
      imuToWorldRotation.set(imuOutput.getOrientationMeasurement());

      rootToWorld.set(rootToIMURotation);
      rootToWorld.preMultiply(imuToWorldRotation);
   }
}
