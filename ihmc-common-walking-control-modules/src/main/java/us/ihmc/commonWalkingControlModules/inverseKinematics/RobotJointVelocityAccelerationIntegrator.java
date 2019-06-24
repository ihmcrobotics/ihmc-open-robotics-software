package us.ihmc.commonWalkingControlModules.inverseKinematics;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.commons.MathTools;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.mecano.multiBodySystem.SixDoFJoint;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotics.linearAlgebra.MatrixTools;
import us.ihmc.robotics.math.QuaternionCalculus;

public class RobotJointVelocityAccelerationIntegrator
{
   private final DenseMatrix64F jointConfigurations = new DenseMatrix64F(100, 0);
   private final DenseMatrix64F jointVelocities = new DenseMatrix64F(100, 0);
   private final DenseMatrix64F jointAccelerations = new DenseMatrix64F(100, 0);

   private final double controlDT;
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

   public RobotJointVelocityAccelerationIntegrator(double controlDT)
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

   public void integrateJointVelocities(JointBasics[] joints, DenseMatrix64F jointVelocitiesToIntegrate)
   {
      int jointConfigurationStartIndex = 0;
      int jointStartIndex = 0;

      int numberOfDoFs = MultiBodySystemTools.computeDegreesOfFreedom(joints);
      int sixDofJoints = MultiBodySystemTools.filterJoints(joints, SixDoFJoint.class).length;
      jointConfigurations.reshape(numberOfDoFs + sixDofJoints, 1);
      jointVelocities.reshape(numberOfDoFs, 1);

      for (int i = 0; i < joints.length; i++)
      {
         if (joints[i] instanceof OneDoFJointBasics)
         {
            OneDoFJointBasics joint = (OneDoFJointBasics) joints[i];

            double qDot = MathTools.clamp(jointVelocitiesToIntegrate.get(jointStartIndex, 0), maximumOneDoFJointVelocity);

            double q = joint.getQ() + qDot * controlDT;
            q = MathTools.clamp(q, joint.getJointLimitLower(), joint.getJointLimitUpper());
            qDot = (q - joint.getQ()) / controlDT;

            jointConfigurations.set(jointConfigurationStartIndex, 0, q);
            jointVelocities.set(jointStartIndex, 0, qDot);

            jointConfigurationStartIndex++;
            jointStartIndex++;
         }
         else if (joints[i] instanceof SixDoFJoint)
         {
            SixDoFJoint joint = (SixDoFJoint) joints[i];
            previousOrientation.set(joint.getJointPose().getOrientation());
            previousTranslation.set(joint.getJointPose().getPosition());

            MatrixTools.extractTuple3dFromEJMLVector(angularVelocity, jointVelocitiesToIntegrate, jointStartIndex);
            double angularVelocityMagnitude = angularVelocity.length();
            if (angularVelocityMagnitude > maximumSixDoFJointAngularVelocity)
               angularVelocity.scale(maximumSixDoFJointAngularVelocity / angularVelocityMagnitude);
            rotationVectorIntegrated.setAndScale(controlDT, angularVelocity);
            quaternionCalculus.exp(rotationVectorIntegrated, rotationIntegrated);

            rotation.set(previousOrientation);
            rotation.multiply(rotationIntegrated);

            MatrixTools.extractTuple3dFromEJMLVector(linearVelocity, jointVelocitiesToIntegrate, jointStartIndex + 3);
            double linearVelocityMagnitude = linearVelocity.length();
            if (linearVelocityMagnitude > maximumSixDoFJointLinearVelocity)
               linearVelocity.scale(maximumSixDoFJointLinearVelocity / linearVelocityMagnitude);
            translationIntegrated.setAndScale(controlDT, linearVelocity);
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

   public void integrateJointAccelerations(JointBasics[] joints, DenseMatrix64F jointAccelerationsToIntegrate)
   {
      int jointStartIndex = 0;

      int numberOfDoFs = MultiBodySystemTools.computeDegreesOfFreedom(joints);
      jointVelocities.reshape(numberOfDoFs, 1);
      jointAccelerations.reshape(numberOfDoFs, 1);

      for (int i = 0; i < joints.length; i++)
      {
         if (joints[i] instanceof OneDoFJointBasics)
         {
            OneDoFJointBasics joint = (OneDoFJointBasics) joints[i];

            double qDDot = MathTools.clamp(jointAccelerationsToIntegrate.get(jointStartIndex, 0), maximumOneDoFJointAcceleration);

            double qDot = joint.getQd() + qDDot * controlDT;
            qDot = MathTools.clamp(qDot, maximumOneDoFJointVelocity);
            qDDot = (qDot - joint.getQd()) / controlDT;

            jointVelocities.set(jointStartIndex, 0, qDot);
            jointAccelerations.set(jointStartIndex, 0, qDDot);

            jointStartIndex++;
         }
         else if (joints[i] instanceof SixDoFJoint)
         {
            SixDoFJoint joint = (SixDoFJoint) joints[i];
            previousOrientation.set(joint.getJointPose().getOrientation());
            previousTranslation.set(joint.getJointPose().getPosition());
            previousLinearVelocity.set(joint.getJointTwist().getLinearPart());
            previousAngularVelocity.set(joint.getJointTwist().getAngularPart());

            MatrixTools.extractTuple3dFromEJMLVector(angularAcceleration, jointAccelerationsToIntegrate, jointStartIndex);
            double angularAccelerationMagnitude = angularAcceleration.length();
            if (angularAccelerationMagnitude > maximumSixDoFJointAngularAcceleration)
               angularAcceleration.scale(maximumSixDoFJointAngularAcceleration / angularAccelerationMagnitude);
            angularAccelerationIntegrated.setAndScale(controlDT, angularAcceleration);

            MatrixTools.extractTuple3dFromEJMLVector(linearAcceleration, jointAccelerationsToIntegrate, jointStartIndex + 3);
            double linearAccelerationMagnitude = linearAcceleration.length();
            if (linearAccelerationMagnitude > maximumSixDoFJointLinearAcceleration)
               linearAcceleration.scale(maximumSixDoFJointLinearAcceleration / linearAccelerationMagnitude);
            linearAccelerationIntegrated.setAndScale(controlDT, linearAcceleration);

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

   public DenseMatrix64F getJointConfigurations()
   {
      return jointConfigurations;
   }

   public DenseMatrix64F getJointVelocities()
   {
      return jointVelocities;
   }

   public DenseMatrix64F getJointAccelerations()
   {
      return jointAccelerations;
   }
}
