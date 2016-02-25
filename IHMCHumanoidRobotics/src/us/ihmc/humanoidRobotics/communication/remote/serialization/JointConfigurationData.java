package us.ihmc.humanoidRobotics.communication.remote.serialization;

import java.util.Random;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.communication.packets.Packet;
import us.ihmc.robotics.screwTheory.InverseDynamicsJoint;
import us.ihmc.tools.ArrayTools;

public class JointConfigurationData extends Packet<JointConfigurationData>
{
   public DenseMatrix64F[] jointConfigurations;

   public JointConfigurationData(InverseDynamicsJoint[] joints)
   {
      this.jointConfigurations = new DenseMatrix64F[joints.length];

      getConfigurationDataFromJoints(joints);
   }

   public JointConfigurationData(Random random)
   {
      int size = Math.abs(random.nextInt(100));
      jointConfigurations = new DenseMatrix64F[size];
      for (int i = 0; i < jointConfigurations.length; i++)
      {
         double[] data = new double[random.nextInt(1000)];
         for (int j = 0; j < data.length; j++)
         {
            data[j] = random.nextDouble();
         }
         jointConfigurations[i] = new DenseMatrix64F();
         jointConfigurations[i].setData(data);
      }
   }

   public JointConfigurationData()
   {

   }

   private void getConfigurationDataFromJoints(InverseDynamicsJoint[] joints)
   {
      for (int i = 0; i < joints.length; i++)
      {
         InverseDynamicsJoint joint = joints[i];
         DenseMatrix64F jointConfiguration = jointConfigurations[i];
         if (jointConfiguration == null)
         {
            jointConfiguration = new DenseMatrix64F(joint.getConfigurationMatrixSize(), 1);
            jointConfigurations[i] = jointConfiguration;
         }

         joint.getConfigurationMatrix(jointConfiguration, 0);
      }
   }

   public void putConfigurationsIntoJoints(InverseDynamicsJoint[] joints)
   {
      if (jointConfigurations.length != joints.length)
         throw new RuntimeException("Number of joints does not match up. Received: " + jointConfigurations.length + ", expected: " + joints.length);

      for (int i = 0; i < jointConfigurations.length; i++)
      {
         DenseMatrix64F configuration = jointConfigurations[i];
         joints[i].setConfiguration(configuration, 0);
      }
   }

   public DenseMatrix64F[] getJointConfigurations()
   {
      return jointConfigurations;
   }

   @Override
   public boolean epsilonEquals(JointConfigurationData other, double epsilon)
   {
      if (jointConfigurations.length != other.jointConfigurations.length)
      {
         return false;
      }

      boolean ret = true;
      for (int i = 0; i < jointConfigurations.length; i++)
      {
         ret &= ArrayTools.deltaEquals(jointConfigurations[i].getData(), other.jointConfigurations[i].getData(), epsilon);
      }
      return ret;
   }
}
