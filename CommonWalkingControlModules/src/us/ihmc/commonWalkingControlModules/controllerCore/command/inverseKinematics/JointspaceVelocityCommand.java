package us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics;

import static us.ihmc.commonWalkingControlModules.controllerCore.command.SolverWeightLevels.HARD_CONSTRAINT;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommandType;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.lists.DenseMatrixArrayList;
import us.ihmc.robotics.screwTheory.InverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.OneDoFJoint;

public class JointspaceVelocityCommand implements InverseKinematicsCommand<JointspaceVelocityCommand>
{
   private double weight;

   private final int initialCapacity = 15;
   private final List<String> jointNames = new ArrayList<>(initialCapacity);
   private final List<InverseDynamicsJoint> joints = new ArrayList<>(initialCapacity);
   private final DenseMatrixArrayList desiredVelocities = new DenseMatrixArrayList(initialCapacity);

   public JointspaceVelocityCommand()
   {
      clear();
   }

   public void clear()
   {
      joints.clear();
      jointNames.clear();
      desiredVelocities.clear();
      removeWeight();
   }

   public void addJoint(OneDoFJoint joint, double desiredVelocity)
   {
      joints.add(joint);
      jointNames.add(joint.getName());
      DenseMatrix64F jointDesiredAcceleration = desiredVelocities.add();
      jointDesiredAcceleration.reshape(1, 1);
      jointDesiredAcceleration.set(0, 0, desiredVelocity);
   }

   public void addJoint(InverseDynamicsJoint joint, DenseMatrix64F desiredVelocity)
   {
      checkConsistency(joint, desiredVelocity);
      joints.add(joint);
      jointNames.add(joint.getName());
      desiredVelocities.add().set(desiredVelocity);
   }

   public void setOneDoFJointDesiredAcceleration(int jointIndex, double desiredVelocity)
   {
      MathTools.checkEquals(joints.get(jointIndex).getDegreesOfFreedom(), 1);
      desiredVelocities.get(jointIndex).reshape(1, 1);
      desiredVelocities.get(jointIndex).set(0, 0, desiredVelocity);
   }

   public void setDesiredAcceleration(int jointIndex, DenseMatrix64F desiredVelocity)
   {
      checkConsistency(joints.get(jointIndex), desiredVelocity);
      desiredVelocities.get(jointIndex).set(desiredVelocity);
   }

   public void retrieveJointsFromName(Map<String, ? extends InverseDynamicsJoint> nameToJointMap)
   {
      for (int i = 0; i < getNumberOfJoints(); i++)
      {
         joints.set(i, nameToJointMap.get(jointNames.get(i)));
      }
   }

   public void removeWeight()
   {
      setWeight(HARD_CONSTRAINT);
   }

   public void setWeight(double weight)
   {
      this.weight = weight;
   }

   @Override
   public void set(JointspaceVelocityCommand other)
   {
      clear();
      for (int i = 0; i < other.getNumberOfJoints(); i++)
      {
         joints.add(other.joints.get(i));
         jointNames.add(other.jointNames.get(i));
      }
      desiredVelocities.set(other.desiredVelocities);
      weight = other.weight;
   }

   private void checkConsistency(InverseDynamicsJoint joint, DenseMatrix64F desiredVelocity)
   {
      MathTools.checkEquals(joint.getDegreesOfFreedom(), desiredVelocity.getNumRows());
   }

   public boolean isHardConstraint()
   {
      return weight == HARD_CONSTRAINT;
   }

   public double getWeight()
   {
      return weight;
   }

   public int getNumberOfJoints()
   {
      return joints.size();
   }

   public List<InverseDynamicsJoint> getJoints()
   {
      return joints;
   }

   public InverseDynamicsJoint getJoint(int jointIndex)
   {
      return joints.get(jointIndex);
   }

   public String getJointName(int jointIndex)
   {
      return jointNames.get(jointIndex);
   }

   public DenseMatrix64F getDesiredVelocity(int jointIndex)
   {
      return desiredVelocities.get(jointIndex);
   }

   public DenseMatrixArrayList getDesiredVelocities()
   {
      return desiredVelocities;
   }

   @Override
   public ControllerCoreCommandType getCommandType()
   {
      return ControllerCoreCommandType.JOINTSPACE;
   }

   @Override
   public String toString()
   {
      String ret = getClass().getSimpleName() + ": ";
      for (int i = 0; i < joints.size(); i++)
      {
         ret += joints.get(i).getName();
         if (i < joints.size() - 1)
            ret += ", ";
         else
            ret += ".";
      }
      return ret;
   }
}
