package us.ihmc.valkyrieRosControl.impedance;

import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.sensorProcessing.sensorProcessors.OneDoFJointStateReadOnly;
import us.ihmc.sensorProcessing.sensorProcessors.SensorOutputMapReadOnly;
import us.ihmc.sensorProcessing.simulatedSensors.SensorDataContext;
import us.ihmc.sensorProcessing.stateEstimation.IMUSensorReadOnly;
import us.ihmc.valkyrieRosControl.ValkyrieRosControlSensorReader;

import java.util.List;

public class ValkyrieImpedanceStateEstimator
{
   private final FloatingJointBasics rootJoint;
   private final RigidBodyBasics rootBody;
   private final OneDoFJointBasics[] controlledOneDoFJoints;
   private SensorOutputMapReadOnly processedSensorOutputMap;

   private ValkyrieRosControlSensorReader sensorReader;
   private List<? extends IMUSensorReadOnly> imuOutputs;

   private final RigidBodyTransform rootToIMU0 = new RigidBodyTransform();

   private final RotationMatrix imu0ToWorldRotation = new RotationMatrix();
   private final RotationMatrix rootToIMU0Rotation = new RotationMatrix();
   private final RotationMatrix rootToWorld = new RotationMatrix();

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
      IMUSensorReadOnly imuOutput0 = imuOutputs.get(0);
      rootJoint.getFrameAfterJoint().getTransformToDesiredFrame(rootToIMU0, imuOutput0.getMeasurementFrame());
      rootToIMU0Rotation.set(rootToIMU0.getRotation());

      imu0ToWorldRotation.set(imuOutput0.getOrientationMeasurement());

      rootToWorld.set(rootToIMU0Rotation);
      rootToWorld.preMultiply(imu0ToWorldRotation);
      rootJoint.getJointPose().getOrientation().set(rootToWorld);
      rootBody.updateFramesRecursively();
   }
}
