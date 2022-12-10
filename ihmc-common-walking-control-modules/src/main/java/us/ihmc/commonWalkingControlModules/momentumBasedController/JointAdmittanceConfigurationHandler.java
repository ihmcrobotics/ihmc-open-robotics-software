package us.ihmc.commonWalkingControlModules.momentumBasedController;

import gnu.trove.impl.Constants;
import gnu.trove.map.hash.TObjectIntHashMap;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import us.ihmc.commonWalkingControlModules.configurations.JointPrivilegedConfigurationParameters;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.PrivilegedConfigurationCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.PrivilegedConfigurationCommand.PrivilegedConfigurationOption;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.PrivilegedJointSpaceCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.LowLevelOneDoFJointDesiredDataHolder;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.OneDoFJointPrivilegedConfigurationParameters;
import us.ihmc.commons.MathTools;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutput;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputBasics;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListReadOnly;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

/**
 * This class computes the input for the optimization based on the desired privileged configuration
 * commands.
 */
public class JointAdmittanceConfigurationHandler
{
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private boolean hasDefaultJointStiffnessChanged = true;
   private final YoDouble defaultJointStiffness = new YoDouble("jointAdmittanceDefaultStiffness", registry);
   private boolean hasDefaultJointDampingChanged = true;
   private final YoDouble defaultJointDamping = new YoDouble("jointAdmittanceDefaultDamping", registry);

   private final YoDouble[] yoJointAdmittanceStiffness;
   private final YoDouble[] yoJointAdmittanceDamping;
   private final YoDouble[] yoJointAdmittancePositions;
   private final YoDouble[] yoJointAdmittanceVelocities;

   private final DMatrixRMaj admittanceStiffness;
   private final DMatrixRMaj admittanceDampingGains;

   private final LowLevelOneDoFJointDesiredDataHolder jointDesiredDataHolder;

   private final OneDoFJointBasics[] oneDoFJoints;
   private final TObjectIntHashMap<OneDoFJointBasics> jointIndices;

   private final int numberOfDoFs;

   public JointAdmittanceConfigurationHandler(OneDoFJointBasics[] oneDoFJoints,
                                              YoRegistry parentRegistry)
   {
      this.oneDoFJoints = oneDoFJoints;
      numberOfDoFs = MultiBodySystemTools.computeDegreesOfFreedom(oneDoFJoints); // note that this should be equal to oneDoFJoints.length

      jointDesiredDataHolder = new LowLevelOneDoFJointDesiredDataHolder(oneDoFJoints);

      admittanceStiffness = new DMatrixRMaj(numberOfDoFs, 1);
      admittanceDampingGains = new DMatrixRMaj(numberOfDoFs, 1);

      jointIndices = new TObjectIntHashMap<>(numberOfDoFs, Constants.DEFAULT_LOAD_FACTOR, -1);

      defaultJointStiffness.set(100.0);
      defaultJointDamping.set(5.0);

      defaultJointDamping.addListener(v -> hasDefaultJointDampingChanged = true);
      defaultJointStiffness.addListener(v -> hasDefaultJointStiffnessChanged = true);

      yoJointAdmittanceStiffness = new YoDouble[numberOfDoFs];
      yoJointAdmittanceDamping = new YoDouble[numberOfDoFs];
      yoJointAdmittancePositions = new YoDouble[numberOfDoFs];
      yoJointAdmittanceVelocities = new YoDouble[numberOfDoFs];

      for (int i = 0; i < numberOfDoFs; i++)
      {
         OneDoFJointBasics joint = oneDoFJoints[i];

         jointIndices.put(joint, i);

         String jointName = joint.getName();
         yoJointAdmittanceStiffness[i] = new YoDouble("kp_adm_" + jointName, registry);
         yoJointAdmittanceDamping[i] = new YoDouble("kd_adm_" + jointName, registry);
         yoJointAdmittancePositions[i] = new YoDouble("q_adm_" + jointName, registry);
         yoJointAdmittanceVelocities[i] = new YoDouble("qd_adm_" + jointName, registry);
      }


      parentRegistry.addChild(registry);
   }


   /**
    * Computes the desired joint accelerations to be submitted to the inverse dynamics control core to
    * achieve the desired privileged configuration. Uses a simple PD controller with saturation limits
    * based on the position error.
    */
   public void computeJointAdmittanceSetpoints(LowLevelOneDoFJointDesiredDataHolder jointTorqueDataHolders)
   {
      processDefaultImpedanceSetpoints();

      jointDesiredDataHolder.set(jointTorqueDataHolders);

      for (int jointIndex = 0; jointIndex < numberOfDoFs; jointIndex++)
      {
         OneDoFJointBasics joint = oneDoFJoints[jointIndex];
         double desiredTorque = jointTorqueDataHolders.getDesiredJointTorque(joint);
         double qd = joint.getQd();
         double dampingTorque = -admittanceDampingGains.get(jointIndex, 0) * qd;

         double desiredStiffnessTorque = desiredTorque - dampingTorque;
         double requiredJointPositionError = desiredStiffnessTorque / admittanceStiffness.get(jointIndex, 0);
         double desiredJointSetpoint = joint.getQ() + requiredJointPositionError;

         double clampedSetpoint = MathTools.clamp(desiredJointSetpoint, joint.getJointLimitLower(), joint.getJointLimitUpper());
         double clampedStiffness = desiredStiffnessTorque / (clampedSetpoint - joint.getQ());

         yoJointAdmittancePositions[jointIndex].set(clampedSetpoint);
         yoJointAdmittanceVelocities[jointIndex].set(0.0);
         yoJointAdmittanceStiffness[jointIndex].set(clampedStiffness);
         yoJointAdmittanceDamping[jointIndex].set(admittanceDampingGains.get(jointIndex, 0));

         JointDesiredOutputBasics outputData = jointDesiredDataHolder.getJointDesiredOutput(joint);
         outputData.setDesiredPosition(clampedSetpoint);
         outputData.setDesiredVelocity(0.0);
         outputData.setStiffness(clampedStiffness);
         outputData.setDamping(yoJointAdmittanceDamping[jointIndex].getDoubleValue());
      }
   }

   public LowLevelOneDoFJointDesiredDataHolder getOutput()
   {
      return jointDesiredDataHolder;
   }

   private void processDefaultImpedanceSetpoints()
   {
      if (hasDefaultJointStiffnessChanged)
      {
         for (int jointIndex = 0; jointIndex < numberOfDoFs; jointIndex++)
            admittanceStiffness.set(jointIndex, 0, defaultJointStiffness.getDoubleValue());
         hasDefaultJointStiffnessChanged = false;
      }

      if (hasDefaultJointDampingChanged)
      {
         for (int jointIndex = 0; jointIndex < numberOfDoFs; jointIndex++)
            admittanceDampingGains.set(jointIndex, 0, defaultJointDamping.getDoubleValue());
         hasDefaultJointDampingChanged = false;
      }
   }
}
