package us.ihmc.valkyrieRosControl.upperBody;

import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.sensorProcessing.sensorProcessors.OneDoFJointStateReadOnly;
import us.ihmc.sensorProcessing.sensorProcessors.SensorOutputMapReadOnly;
import us.ihmc.sensorProcessing.stateEstimation.IMUSensorReadOnly;
import us.ihmc.valkyrieRosControl.ValkyrieRosControlSensorReader;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.List;

public class ValkyrieUpperBodyStateEstimator
{
   private static final int EXPECTED_NUMBER_OF_IMUS = 1;

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final FloatingJointBasics rootJoint;
   private final RigidBodyBasics rootBody;
   private final OneDoFJointBasics[] controlledOneDoFJoints;
   private SensorOutputMapReadOnly processedSensorOutputMap;

   private ValkyrieRosControlSensorReader sensorReader;
   private List<? extends IMUSensorReadOnly> imuOutputs;

   public ValkyrieUpperBodyStateEstimator(FloatingJointBasics rootJoint,
                                          OneDoFJointBasics[] controlledOneDoFJoints,
                                          YoRegistry registry)
   {
      this.rootJoint = rootJoint;
      this.rootBody = rootJoint.getPredecessor();
      this.controlledOneDoFJoints = controlledOneDoFJoints;

      registry.addChild(registry);
   }

   public void init(ValkyrieRosControlSensorReader sensorReader)
   {
      this.sensorReader = sensorReader;
      processedSensorOutputMap = sensorReader.getProcessedSensorOutputMap();
      imuOutputs = processedSensorOutputMap.getIMUOutputs();
      sensorReader.initialize();

      if (imuOutputs.size() != EXPECTED_NUMBER_OF_IMUS)
      {
         throw new RuntimeException("Expecting " + EXPECTED_NUMBER_OF_IMUS + " IMU's, but there are " + imuOutputs.size());
      }
   }

   public void update()
   {
      /* For now ignore IMU and do state estimation through joint encoders */
      sensorReader.readSensors();

      for (int i = 0; i < controlledOneDoFJoints.length; i++)
      {
         OneDoFJointStateReadOnly processedJointOutput = processedSensorOutputMap.getOneDoFJointOutput(controlledOneDoFJoints[i]);
         controlledOneDoFJoints[i].setQ(processedJointOutput.getPosition());
         controlledOneDoFJoints[i].setQd(processedJointOutput.getVelocity());
      }

      rootBody.updateFramesRecursively();
   }
}
