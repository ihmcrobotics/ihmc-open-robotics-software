package us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.solver;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Queue;

import org.apache.commons.lang3.mutable.MutableInt;
import org.ejml.data.DenseMatrix64F;

import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.lists.DenseMatrixArrayList;
import us.ihmc.robotics.screwTheory.InverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.OneDoFJoint;

public class JointspaceAccelerationCommand extends InverseDynamicsCommand<JointspaceAccelerationCommand>
{
   private boolean hasWeight;
   private double weight;

   private final int initialCapacity = 15;
   private final List<InverseDynamicsJoint> joints = new ArrayList<>(initialCapacity);
   private final Queue<MutableInt> unusedIntegers = new ArrayDeque<>(initialCapacity);
   private final LinkedHashMap<InverseDynamicsJoint, MutableInt> jointToJointIndexMap = new LinkedHashMap<>(initialCapacity);
   private final DenseMatrixArrayList desiredAccelerations = new DenseMatrixArrayList(initialCapacity);

   public JointspaceAccelerationCommand()
   {
      super(InverseDynamicsCommandType.JOINTSPACE_MOTION);
      initializeUnusedIntegers();
      clear();
   }

   public JointspaceAccelerationCommand(InverseDynamicsJoint joint, DenseMatrix64F desiredAcceleration)
   {
      this(joint, desiredAcceleration, Double.POSITIVE_INFINITY);
   }

   public JointspaceAccelerationCommand(InverseDynamicsJoint joint, DenseMatrix64F desiredAcceleration, double weight)
   {
      this();
      addJoint(joint, desiredAcceleration);
      setWeight(weight);
   }

   private void initializeUnusedIntegers()
   {
      for (int i = 0; i < initialCapacity; i++)
         unusedIntegers.add(new MutableInt(-1));
   }

   public void clear()
   {
      while (!joints.isEmpty())
         unusedIntegers.add(jointToJointIndexMap.remove(joints.remove(joints.size() - 1)));

      desiredAccelerations.clear();
      removeWeight();
   }

   public void addJoint(OneDoFJoint joint, double desiredAcceleration)
   {
      joints.add(joint);
      DenseMatrix64F jointDesiredAcceleration = desiredAccelerations.add();
      jointDesiredAcceleration.reshape(1, 1);
      jointDesiredAcceleration.set(0, 0, desiredAcceleration);

      updateIndexMap(joint, joints.size() - 1);
   }

   public void addJoint(InverseDynamicsJoint joint, DenseMatrix64F desiredAcceleration)
   {
      checkConsistency(joint, desiredAcceleration);
      joints.add(joint);
      desiredAccelerations.add().set(desiredAcceleration);

      updateIndexMap(joint, joints.size() - 1);
   }

   private void updateIndexMap(InverseDynamicsJoint joint, int jointIndex)
   {
      if (unusedIntegers.isEmpty())
      {
         jointToJointIndexMap.put(joint, new MutableInt(joints.size() - 1));
      }
      else
      {
         MutableInt jointMutableIndex = unusedIntegers.remove();
         jointMutableIndex.setValue(jointIndex);
         jointToJointIndexMap.put(joint, jointMutableIndex);
      }
   }

   public void setOneDoFJointDesiredAcceleration(int jointIndex, double desiredAcceleration)
   {
      MathTools.checkIfEqual(joints.get(jointIndex).getDegreesOfFreedom(), 1);
      desiredAccelerations.get(jointIndex).reshape(1, 1);
      desiredAccelerations.get(jointIndex).set(0, 0, desiredAcceleration);
   }

   public void setOneDoFJointDesiredAcceleration(OneDoFJoint joint, double desiredAcceleration)
   {
      int jointIndex = jointToJointIndexMap.get(joint).intValue();
      desiredAccelerations.get(jointIndex).reshape(1, 1);
      desiredAccelerations.get(jointIndex).set(0, 0, desiredAcceleration);
   }

   public void setDesiredAcceleration(int jointIndex, DenseMatrix64F desiredAcceleration)
   {
      checkConsistency(joints.get(jointIndex), desiredAcceleration);
      desiredAccelerations.get(jointIndex).set(desiredAcceleration);
   }

   public void setDesiredAcceleration(InverseDynamicsJoint joint, DenseMatrix64F desiredAcceleration)
   {
      int jointIndex = jointToJointIndexMap.get(joint).intValue();
      setDesiredAcceleration(jointIndex, desiredAcceleration);
   }

   public void removeWeight()
   {
      setWeight(Double.POSITIVE_INFINITY);
   }

   public void setWeight(double weight)
   {
      hasWeight = weight != Double.POSITIVE_INFINITY;
      this.weight = weight;
   }

   @Override
   public void set(JointspaceAccelerationCommand other)
   {
      joints.clear();
      for (int i = 0; i < other.getNumberOfJoints(); i++)
         joints.add(other.joints.get(i));
      desiredAccelerations.set(other.desiredAccelerations);
      hasWeight = other.hasWeight;
      weight = other.weight;
   }

   private void checkConsistency(InverseDynamicsJoint joint, DenseMatrix64F desiredAcceleration)
   {
      MathTools.checkIfEqual(joint.getDegreesOfFreedom(), desiredAcceleration.getNumRows());
   }

   public boolean getHasWeight()
   {
      return hasWeight;
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

   public DenseMatrix64F getDesiredAcceleration(int jointIndex)
   {
      return desiredAccelerations.get(jointIndex);
   }

   public DenseMatrixArrayList getDesiredAccelerations()
   {
      return desiredAccelerations;
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
