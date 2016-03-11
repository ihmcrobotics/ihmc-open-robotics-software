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

   private final QuaternionCalculus quaternionCalculus = new QuaternionCalculus();

   private final Vector3d angularVelocity = new Vector3d();
   private final Vector3d linearVelocity = new Vector3d();
   private final Vector3d rotationVectorIntegrated = new Vector3d();
   private final Quat4d rotationIntegrated = new Quat4d();
   private final Quat4d rotation = new Quat4d();
   private final Vector3d translationIntegrated = new Vector3d();
   private final Vector3d translation = new Vector3d();

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

            MatrixTools.extractTuple3dFromEJMLVector(angularVelocity, jointVelocitiesToIntegrate, jointStartIndex);
            double angularVelocityMagnitude = angularVelocity.length();
            if (angularVelocityMagnitude > maximumSixDoFJointAngularVelocity)
               angularVelocity.scale(maximumSixDoFJointAngularVelocity / angularVelocityMagnitude);
            rotationVectorIntegrated.scale(controlDT, angularVelocity);
            quaternionCalculus.exp(rotationVectorIntegrated, rotationIntegrated);

            joint.getRotation(rotation);
            rotation.mul(rotationIntegrated);

            MatrixTools.extractTuple3dFromEJMLVector(linearVelocity, jointVelocitiesToIntegrate, jointStartIndex + 3);
            quaternionCalculus.transform(rotation, linearVelocity);
            double linearVelocityMagnitude = linearVelocity.length();
            if (linearVelocityMagnitude > maximumSixDoFJointLinearVelocity)
               linearVelocity.scale(maximumSixDoFJointLinearVelocity / linearVelocityMagnitude);
            translationIntegrated.scale(controlDT, linearVelocity);

            joint.getTranslation(translation);
            translation.add(translationIntegrated);

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
