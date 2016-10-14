package us.ihmc.commonWalkingControlModules.inverseKinematics;

import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.linearAlgebra.MatrixTools;
import us.ihmc.robotics.math.QuaternionCalculus;
import us.ihmc.robotics.screwTheory.InverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.robotics.screwTheory.SixDoFJoint;

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

   private final Quat4d previousOrientation = new Quat4d();
   private final Vector3d previousTranslation = new Vector3d();

   private final Vector3d previousLinearVelocity = new Vector3d();
   private final Vector3d previousAngularVelocity = new Vector3d();

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

   private final Vector3d angularVelocity = new Vector3d();
   private final Vector3d linearVelocity = new Vector3d();
   private final Vector3d rotationVectorIntegrated = new Vector3d();
   private final Quat4d rotationIntegrated = new Quat4d();
   private final Quat4d rotation = new Quat4d();
   private final Vector3d translationIntegrated = new Vector3d();
   private final Vector3d translation = new Vector3d();

   public void integrateJointVelocities(InverseDynamicsJoint[] joints, DenseMatrix64F jointVelocitiesToIntegrate)
   {
      int jointConfigurationStartIndex = 0;
      int jointStartIndex = 0;

      int numberOfDoFs = ScrewTools.computeDegreesOfFreedom(joints);
      jointConfigurations.reshape(numberOfDoFs + 1, 1);
      jointVelocities.reshape(numberOfDoFs, 1);

      for (int i = 0; i < joints.length; i++)
      {
         if (joints[i] instanceof OneDoFJoint)
         {
            OneDoFJoint joint = (OneDoFJoint) joints[i];

            double qDot = MathTools.clipToMinMax(jointVelocitiesToIntegrate.get(jointStartIndex, 0), maximumOneDoFJointVelocity);

            double q = joint.getQ() + qDot * controlDT;
            q = MathTools.clipToMinMax(q, joint.getJointLimitLower(), joint.getJointLimitUpper());
            qDot = (q - joint.getQ()) / controlDT;

            jointConfigurations.set(jointConfigurationStartIndex, 0, q);
            jointVelocities.set(jointStartIndex, 0, qDot);

            jointConfigurationStartIndex++;
            jointStartIndex++;
         }
         else if (joints[i] instanceof SixDoFJoint)
         {
            SixDoFJoint joint = (SixDoFJoint) joints[i];
            joint.getRotation(previousOrientation);
            joint.getTranslation(previousTranslation);

            MatrixTools.extractTuple3dFromEJMLVector(angularVelocity, jointVelocitiesToIntegrate, jointStartIndex);
            double angularVelocityMagnitude = angularVelocity.length();
            if (angularVelocityMagnitude > maximumSixDoFJointAngularVelocity)
               angularVelocity.scale(maximumSixDoFJointAngularVelocity / angularVelocityMagnitude);
            rotationVectorIntegrated.scale(controlDT, angularVelocity);
            quaternionCalculus.exp(rotationVectorIntegrated, rotationIntegrated);

            rotation.mul(previousOrientation, rotationIntegrated);

            MatrixTools.extractTuple3dFromEJMLVector(linearVelocity, jointVelocitiesToIntegrate, jointStartIndex + 3);
            double linearVelocityMagnitude = linearVelocity.length();
            if (linearVelocityMagnitude > maximumSixDoFJointLinearVelocity)
               linearVelocity.scale(maximumSixDoFJointLinearVelocity / linearVelocityMagnitude);
            translationIntegrated.scale(controlDT, linearVelocity);
            quaternionCalculus.transform(rotation, translationIntegrated);

            translation.add(previousTranslation, translationIntegrated);

            MatrixTools.insertQuat4dIntoEJMLVector(jointConfigurations, rotation, jointConfigurationStartIndex);
            jointConfigurationStartIndex += 4;
            MatrixTools.insertTuple3dIntoEJMLVector(translation, jointConfigurations, jointConfigurationStartIndex);
            jointConfigurationStartIndex += 3;

            MatrixTools.insertTuple3dIntoEJMLVector(angularVelocity, jointVelocities, jointStartIndex);
            MatrixTools.insertTuple3dIntoEJMLVector(linearVelocity, jointVelocities, jointStartIndex + 3);
            jointStartIndex += 6;
         }
      }
   }

   private final Vector3d angularAcceleration = new Vector3d();
   private final Vector3d linearAcceleration = new Vector3d();
   private final Vector3d angularAccelerationIntegrated = new Vector3d();
   private final Vector3d linearAccelerationIntegrated = new Vector3d();

   public void integrateJointAccelerations(InverseDynamicsJoint[] joints, DenseMatrix64F jointAccelerationsToIntegrate)
   {
      int jointStartIndex = 0;

      int numberOfDoFs = ScrewTools.computeDegreesOfFreedom(joints);
      jointVelocities.reshape(numberOfDoFs, 1);
      jointAccelerations.reshape(numberOfDoFs, 1);

      for (int i = 0; i < joints.length; i++)
      {
         if (joints[i] instanceof OneDoFJoint)
         {
            OneDoFJoint joint = (OneDoFJoint) joints[i];

            double qDDot = MathTools.clipToMinMax(jointAccelerationsToIntegrate.get(jointStartIndex, 0), maximumOneDoFJointAcceleration);

            double qDot = joint.getQd() + qDDot * controlDT;
            qDot = MathTools.clipToMinMax(qDot, maximumOneDoFJointVelocity);
            qDDot = (qDot - joint.getQd()) / controlDT;

            jointVelocities.set(jointStartIndex, 0, qDot);
            jointAccelerations.set(jointStartIndex, 0, qDDot);

            jointStartIndex++;
         }
         else if (joints[i] instanceof SixDoFJoint)
         {
            SixDoFJoint joint = (SixDoFJoint) joints[i];
            joint.getRotation(previousOrientation);
            joint.getTranslation(previousTranslation);
            joint.getLinearVelocity(previousLinearVelocity);
            joint.getAngularVelocity(previousAngularVelocity);

            MatrixTools.extractTuple3dFromEJMLVector(angularAcceleration, jointAccelerationsToIntegrate, jointStartIndex);
            double angularAccelerationMagnitude = angularAcceleration.length();
            if (angularAccelerationMagnitude > maximumSixDoFJointAngularAcceleration)
               angularAcceleration.scale(maximumSixDoFJointAngularAcceleration / angularAccelerationMagnitude);
            angularAccelerationIntegrated.scale(controlDT, angularAcceleration);


            MatrixTools.extractTuple3dFromEJMLVector(linearAcceleration, jointAccelerationsToIntegrate, jointStartIndex + 3);
            double linearAccelerationMagnitude = linearAcceleration.length();
            if (linearAccelerationMagnitude > maximumSixDoFJointLinearAcceleration)
               linearAcceleration.scale(maximumSixDoFJointLinearAcceleration / linearAccelerationMagnitude);
            linearAccelerationIntegrated.scale(controlDT, linearAcceleration);

            angularVelocity.add(previousAngularVelocity, angularAccelerationIntegrated);
            linearVelocity.add(previousLinearVelocity, linearAccelerationIntegrated);

            MatrixTools.insertTuple3dIntoEJMLVector(angularVelocity, jointVelocities, jointStartIndex);
            MatrixTools.insertTuple3dIntoEJMLVector(linearVelocity, jointVelocities, jointStartIndex + 3);

            MatrixTools.insertTuple3dIntoEJMLVector(angularAcceleration, jointAccelerations, jointStartIndex);
            MatrixTools.insertTuple3dIntoEJMLVector(linearAcceleration, jointAccelerations, jointStartIndex + 3);
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
