package us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics;

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

public class JointspaceAccelerationCommand implements InverseDynamicsCommand<JointspaceAccelerationCommand>
{
   private boolean hasWeight;
   private double weight;

   private final int initialCapacity = 15;
   private final List<String> jointNames = new ArrayList<>(initialCapacity);
   private final List<InverseDynamicsJoint> joints = new ArrayList<>(initialCapacity);
   private final DenseMatrixArrayList desiredAccelerations = new DenseMatrixArrayList(initialCapacity);

   public JointspaceAccelerationCommand()
   {
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

   public void clear()
   {
      joints.clear();
      jointNames.clear();
      desiredAccelerations.clear();
      removeWeight();
   }

   public void addJoint(OneDoFJoint joint, double desiredAcceleration)
   {
      joints.add(joint);
      jointNames.add(joint.getName());
      DenseMatrix64F jointDesiredAcceleration = desiredAccelerations.add();
      jointDesiredAcceleration.reshape(1, 1);
      jointDesiredAcceleration.set(0, 0, desiredAcceleration);
   }

   public void addJoint(InverseDynamicsJoint joint, DenseMatrix64F desiredAcceleration)
   {
      checkConsistency(joint, desiredAcceleration);
      joints.add(joint);
      jointNames.add(joint.getName());
      desiredAccelerations.add().set(desiredAcceleration);
   }

   public void setOneDoFJointDesiredAcceleration(int jointIndex, double desiredAcceleration)
   {
      MathTools.checkEquals(joints.get(jointIndex).getDegreesOfFreedom(), 1);
      desiredAccelerations.get(jointIndex).reshape(1, 1);
      desiredAccelerations.get(jointIndex).set(0, 0, desiredAcceleration);
   }

   public void setDesiredAcceleration(int jointIndex, DenseMatrix64F desiredAcceleration)
   {
      checkConsistency(joints.get(jointIndex), desiredAcceleration);
      desiredAccelerations.get(jointIndex).set(desiredAcceleration);
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
      hasWeight = weight != HARD_CONSTRAINT;
      this.weight = weight;
   }

   @Override
   public void set(JointspaceAccelerationCommand other)
   {
      clear();
      for (int i = 0; i < other.getNumberOfJoints(); i++)
      {
         joints.add(other.joints.get(i));
         jointNames.add(other.jointNames.get(i));
      }
      desiredAccelerations.set(other.desiredAccelerations);
      hasWeight = other.hasWeight;
      weight = other.weight;
   }

   private void checkConsistency(InverseDynamicsJoint joint, DenseMatrix64F desiredAcceleration)
   {
      MathTools.checkEquals(joint.getDegreesOfFreedom(), desiredAcceleration.getNumRows());
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

   public String getJointName(int jointIndex)
   {
      return jointNames.get(jointIndex);
   }

   public DenseMatrix64F getDesiredAcceleration(int jointIndex)
   {
      return desiredAccelerations.get(jointIndex);
   }

   public DenseMatrixArrayList getDesiredAccelerations()
   {
      return desiredAccelerations;
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
