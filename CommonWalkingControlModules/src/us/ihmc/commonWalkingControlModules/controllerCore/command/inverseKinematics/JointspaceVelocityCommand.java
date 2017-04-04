package us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics;

import static us.ihmc.commonWalkingControlModules.controllerCore.command.SolverWeightLevels.*;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

import org.apache.commons.lang3.mutable.MutableDouble;
import org.ejml.data.DenseMatrix64F;

import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommandType;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.lists.DenseMatrixArrayList;
import us.ihmc.robotics.lists.RecyclingArrayList;
import us.ihmc.robotics.screwTheory.InverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.OneDoFJoint;

public class JointspaceVelocityCommand implements InverseKinematicsCommand<JointspaceVelocityCommand>
{
   private final int initialCapacity = 15;
   private final List<String> jointNames = new ArrayList<>(initialCapacity);
   private final List<InverseDynamicsJoint> joints = new ArrayList<>(initialCapacity);
   private final DenseMatrixArrayList desiredVelocities = new DenseMatrixArrayList(initialCapacity);

   private final RecyclingArrayList<MutableDouble> weights = new RecyclingArrayList<>(initialCapacity, MutableDouble.class);

   public JointspaceVelocityCommand()
   {
      clear();
   }

   public JointspaceVelocityCommand(InverseDynamicsJoint joint, DenseMatrix64F desiredVelocity)
   {
      this(joint, desiredVelocity, HARD_CONSTRAINT);
   }

   public JointspaceVelocityCommand(InverseDynamicsJoint joint, DenseMatrix64F desiredVelocity, double weight)
   {
      this();
      addJoint(joint, desiredVelocity);
      setWeight(weight);
   }

   public void clear()
   {
      joints.clear();
      jointNames.clear();
      desiredVelocities.clear();
      weights.clear();
   }

   public void addJoint(OneDoFJoint joint, double desiredVelocity)
   {
      joints.add(joint);
      jointNames.add(joint.getName());
      weights.add().setValue(HARD_CONSTRAINT);
      DenseMatrix64F jointDesiredVelocity = desiredVelocities.add();
      jointDesiredVelocity.reshape(1, 1);
      jointDesiredVelocity.set(0, 0, desiredVelocity);
   }

   public void addJoint(InverseDynamicsJoint joint, DenseMatrix64F desiredVelocity)
   {
      checkConsistency(joint, desiredVelocity);
      joints.add(joint);
      jointNames.add(joint.getName());
      desiredVelocities.add().set(desiredVelocity);
   }

   public void setOneDoFJointDesiredVelocity(int jointIndex, double desiredVelocity)
   {
      MathTools.checkEquals(joints.get(jointIndex).getDegreesOfFreedom(), 1);
      desiredVelocities.get(jointIndex).reshape(1, 1);
      desiredVelocities.get(jointIndex).set(0, 0, desiredVelocity);
   }

   public void setDesiredVelocity(int jointIndex, DenseMatrix64F desiredVelocity)
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

   public void makeHardConstraint()
   {
      for (int jointIdx = 0; jointIdx < joints.size(); jointIdx++)
         setWeight(jointIdx, HARD_CONSTRAINT);
   }

   public void setWeight(int jointIndex, double weight)
   {
      weights.get(jointIndex).setValue(weight);
   }

   public void setWeight(double weight)
   {
      for (int jointIdx = 0; jointIdx < joints.size(); jointIdx++)
         weights.get(jointIdx).setValue(weight);
   }

   @Override
   public void set(JointspaceVelocityCommand other)
   {
      clear();
      for (int i = 0; i < other.getNumberOfJoints(); i++)
      {
         joints.add(other.joints.get(i));
         jointNames.add(other.jointNames.get(i));
         weights.add().setValue(other.getWeight(i));
      }
      desiredVelocities.set(other.desiredVelocities);
   }

   private void checkConsistency(InverseDynamicsJoint joint, DenseMatrix64F desiredVelocity)
   {
      MathTools.checkEquals(joint.getDegreesOfFreedom(), desiredVelocity.getNumRows());
   }

   public boolean isHardConstraint()
   {
      if (getNumberOfJoints() == 0)
         return true;

      boolean isHardConstraint = getWeight(0) == HARD_CONSTRAINT;
      if (getNumberOfJoints() == 1)
         return isHardConstraint;

      // If there are multiple joints, make sure they are consistent.
      for (int jointIdx = 1; jointIdx < joints.size(); jointIdx++)
      {
         boolean isJointHardConstraint = getWeight(jointIdx) == HARD_CONSTRAINT;
         if (isJointHardConstraint != isHardConstraint)
            throw new RuntimeException("Inconsistent weights in " + getClass().getSimpleName() + ": some joint velocity "
                  + "desireds have weights, others are hard constraints. This is not supported in a single message.");
      }

      return isHardConstraint;
   }

   public double getWeight(int jointIndex)
   {
      return weights.get(jointIndex).doubleValue();
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
