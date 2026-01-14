package us.ihmc.commonWalkingControlModules.inverseKinematics;

import org.ejml.data.DMatrixRMaj;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.QuaternionCalculus;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.SixDoFJointReadOnly;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.yoVariables.providers.DoubleProvider;

public class RobotJointVelocityAccelerationIntegrator
{
   private final DMatrixRMaj jointConfigurations = new DMatrixRMaj(100, 0);
   private final DMatrixRMaj jointVelocities = new DMatrixRMaj(100, 0);
   private final DMatrixRMaj jointAccelerations = new DMatrixRMaj(100, 0);

   private final DoubleProvider controlDT;
   private double maximumOneDoFJointVelocity = Double.POSITIVE_INFINITY;
   private double maximumSixDoFJointLinearVelocity = Double.POSITIVE_INFINITY;
   private double maximumSixDoFJointAngularVelocity = Double.POSITIVE_INFINITY;

   private double maximumOneDoFJointAcceleration = Double.POSITIVE_INFINITY;
   private double maximumSixDoFJointLinearAcceleration = Double.POSITIVE_INFINITY;
   private double maximumSixDoFJointAngularAcceleration = Double.POSITIVE_INFINITY;

   private final QuaternionCalculus quaternionCalculus = new QuaternionCalculus();

   private final Quaternion previousOrientation = new Quaternion();
   private final Vector3D previousTranslation = new Vector3D();

   private final Vector3D previousLinearVelocity = new Vector3D();
   private final Vector3D previousAngularVelocity = new Vector3D();

   public RobotJointVelocityAccelerationIntegrator(DoubleProvider controlDT)
   {
      this.controlDT = controlDT;
   }

   public void setMaximumOneDoFJointVelocity(double maximumOneDoFJointVelocity)
   {
      this.maximumOneDoFJointVelocity = maximumOneDoFJointVelocity;
   }

   public void setMaximumSixDoFJointLinearVelocity(double maximumSixDoFJointLinearVelocity)
   {
      this.maximumSixDoFJointLinearVelocity = maximumSixDoFJointLinearVelocity;
   }

   public void setMaximumSixDoFJointAngularVelocity(double maximumSixDoFJointAngularVelocity)
   {
      this.maximumSixDoFJointAngularVelocity = maximumSixDoFJointAngularVelocity;
   }

   private final Vector3D angularVelocity = new Vector3D();
   private final Vector3D linearVelocity = new Vector3D();
   private final Vector3D rotationVectorIntegrated = new Vector3D();
   private final Quaternion rotationIntegrated = new Quaternion();
   private final Quaternion rotation = new Quaternion();
   private final Vector3D translationIntegrated = new Vector3D();
   private final Vector3D translation = new Vector3D();

   public void integrateJointVelocities(JointBasics[] joints, DMatrixRMaj jointVelocitiesToIntegrate)
   {
      int jointConfigurationStartIndex = 0;
      int jointStartIndex = 0;

      int numberOfDoFs = MultiBodySystemTools.computeDegreesOfFreedom(joints);
      int sixDofJoints = 0;
      for (JointBasics joint : joints)
      {
         if (joint instanceof FloatingJointReadOnly)
            sixDofJoints++;
      }
      jointConfigurations.reshape(numberOfDoFs + sixDofJoints, 1);
      jointVelocities.reshape(numberOfDoFs, 1);

      for (int i = 0; i < joints.length; i++)
      {
         if (joints[i] instanceof OneDoFJointReadOnly joint)
         {
            double qDot = MathTools.clamp(jointVelocitiesToIntegrate.get(jointStartIndex, 0), maximumOneDoFJointVelocity);

            double q = joint.getQ() + qDot * controlDT.getValue();
            q = MathTools.clamp(q, joint.getJointLimitLower(), joint.getJointLimitUpper());
            qDot = (q - joint.getQ()) / controlDT.getValue();

            jointConfigurations.set(jointConfigurationStartIndex, 0, q);
            jointVelocities.set(jointStartIndex, 0, qDot);

            jointConfigurationStartIndex++;
            jointStartIndex++;
         }
         else if (joints[i] instanceof SixDoFJointReadOnly joint)
         {
            previousOrientation.set(joint.getJointPose().getOrientation());
            previousTranslation.set(joint.getJointPose().getPosition());

            angularVelocity.set(jointStartIndex, jointVelocitiesToIntegrate);
            double angularVelocityMagnitude = angularVelocity.norm();
            if (angularVelocityMagnitude > maximumSixDoFJointAngularVelocity)
               angularVelocity.scale(maximumSixDoFJointAngularVelocity / angularVelocityMagnitude);
            rotationVectorIntegrated.setAndScale(controlDT.getValue(), angularVelocity);
            QuaternionCalculus.exp(rotationVectorIntegrated, rotationIntegrated);

            rotation.set(previousOrientation);
            rotation.multiply(rotationIntegrated);

            linearVelocity.set(jointStartIndex + 3, jointVelocitiesToIntegrate);
            double linearVelocityMagnitude = linearVelocity.norm();
            if (linearVelocityMagnitude > maximumSixDoFJointLinearVelocity)
               linearVelocity.scale(maximumSixDoFJointLinearVelocity / linearVelocityMagnitude);
            translationIntegrated.setAndScale(controlDT.getValue(), linearVelocity);
            rotation.transform(translationIntegrated);

            translation.add(previousTranslation, translationIntegrated);

            rotation.get(jointConfigurationStartIndex, jointConfigurations);
            jointConfigurationStartIndex += 4;
            translation.get(jointConfigurationStartIndex, jointConfigurations);
            jointConfigurationStartIndex += 3;

            angularVelocity.get(jointStartIndex, jointVelocities);
            linearVelocity.get(jointStartIndex + 3, jointVelocities);
            jointStartIndex += 6;
         }
      }
   }

   private final Vector3D angularAcceleration = new Vector3D();
   private final Vector3D linearAcceleration = new Vector3D();
   private final Vector3D angularAccelerationIntegrated = new Vector3D();
   private final Vector3D linearAccelerationIntegrated = new Vector3D();

   public void integrateJointAccelerations(JointBasics[] joints, DMatrixRMaj jointAccelerationsToIntegrate)
   {
      int jointStartIndex = 0;

      int numberOfDoFs = MultiBodySystemTools.computeDegreesOfFreedom(joints);
      jointVelocities.reshape(numberOfDoFs, 1);
      jointAccelerations.reshape(numberOfDoFs, 1);

      for (int i = 0; i < joints.length; i++)
      {
         if (joints[i] instanceof OneDoFJointReadOnly joint)
         {
            double qDDot = MathTools.clamp(jointAccelerationsToIntegrate.get(jointStartIndex, 0), maximumOneDoFJointAcceleration);

            double qDot = joint.getQd() + qDDot * controlDT.getValue();
            qDot = MathTools.clamp(qDot, maximumOneDoFJointVelocity);
            qDDot = (qDot - joint.getQd()) / controlDT.getValue();

            jointVelocities.set(jointStartIndex, 0, qDot);
            jointAccelerations.set(jointStartIndex, 0, qDDot);

            jointStartIndex++;
         }
         else if (joints[i] instanceof SixDoFJointReadOnly joint)
         {
            previousOrientation.set(joint.getJointPose().getOrientation());
            previousTranslation.set(joint.getJointPose().getPosition());
            previousLinearVelocity.set(joint.getJointTwist().getLinearPart());
            previousAngularVelocity.set(joint.getJointTwist().getAngularPart());

            angularAcceleration.set(jointStartIndex, jointAccelerationsToIntegrate);
            double angularAccelerationMagnitude = angularAcceleration.norm();
            if (angularAccelerationMagnitude > maximumSixDoFJointAngularAcceleration)
               angularAcceleration.scale(maximumSixDoFJointAngularAcceleration / angularAccelerationMagnitude);
            angularAccelerationIntegrated.setAndScale(controlDT.getValue(), angularAcceleration);

            linearAcceleration.set(jointStartIndex + 3, jointAccelerationsToIntegrate);
            double linearAccelerationMagnitude = linearAcceleration.norm();
            if (linearAccelerationMagnitude > maximumSixDoFJointLinearAcceleration)
               linearAcceleration.scale(maximumSixDoFJointLinearAcceleration / linearAccelerationMagnitude);
            linearAccelerationIntegrated.setAndScale(controlDT.getValue(), linearAcceleration);

            angularVelocity.add(previousAngularVelocity, angularAccelerationIntegrated);
            linearVelocity.add(previousLinearVelocity, linearAccelerationIntegrated);

            angularVelocity.get(jointStartIndex, jointVelocities);
            linearVelocity.get(jointStartIndex + 3, jointVelocities);
            angularAcceleration.get(jointStartIndex, jointAccelerations);
            linearAcceleration.get(jointStartIndex + 3, jointAccelerations);
            jointStartIndex += 6;
         }
      }
   }

   public DMatrixRMaj getJointConfigurations()
   {
      return jointConfigurations;
   }

   public DMatrixRMaj getJointVelocities()
   {
      return jointVelocities;
   }

   public DMatrixRMaj getJointAccelerations()
   {
      return jointAccelerations;
   }
}
