package us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics;

import java.util.ArrayList;
import java.util.List;

import gnu.trove.list.array.TDoubleArrayList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommandType;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.commons.MathTools;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;

public class JointLimitReductionCommand implements InverseKinematicsCommand<JointLimitReductionCommand>, InverseDynamicsCommand<JointLimitReductionCommand>
{
   private static final int initialCapacity = 40;
   private final List<OneDoFJointBasics> joints = new ArrayList<>(initialCapacity);
   private final TDoubleArrayList jointReductionFactors = new TDoubleArrayList(initialCapacity);

   public JointLimitReductionCommand()
   {
   }

   public void clear()
   {
      joints.clear();
      jointReductionFactors.reset();
   }

   public void addReductionFactor(OneDoFJointBasics joint, double reductionFactor)
   {
      MathTools.checkIntervalContains(reductionFactor, 0.0, 1.0);
      joints.add(joint);
      jointReductionFactors.add(reductionFactor);
   }

   @Override
   public void set(JointLimitReductionCommand other)
   {
      clear();

      for (int i = 0; i < other.getNumberOfJoints(); i++)
      {
         joints.add(other.joints.get(i));
         jointReductionFactors.add(other.jointReductionFactors.get(i));
      }
   }

   public double getJointLimitReductionFactor(int jointIndex)
   {
      return jointReductionFactors.get(jointIndex);
   }

   public int getNumberOfJoints()
   {
      return joints.size();
   }

   public OneDoFJointBasics getJoint(int jointIndex)
   {
      return joints.get(jointIndex);
   }

   @Override
   public ControllerCoreCommandType getCommandType()
   {
      return ControllerCoreCommandType.LIMIT_REDUCTION;
   }

   @Override
   public boolean equals(Object object)
   {
      if (object == this)
      {
         return true;
      }
      else if (object instanceof JointLimitReductionCommand)
      {
         JointLimitReductionCommand other = (JointLimitReductionCommand) object;

         if (getNumberOfJoints() != other.getNumberOfJoints())
            return false;
         for (int jointIndex = 0; jointIndex < getNumberOfJoints(); jointIndex++)
         {
            if (joints.get(jointIndex) != other.joints.get(jointIndex))
               return false;
         }
         if (!jointReductionFactors.equals(other.jointReductionFactors))
            return false;

         return true;
      }
      else
      {
         return false;
      }
   }

   @Override
   public String toString()
   {
      String ret = getClass().getSimpleName() + ":";
      for (int jointIndex = 0; jointIndex < getNumberOfJoints(); jointIndex++)
      {
         ret += "\nJoint: " + joints.get(jointIndex) + ", reduction: " + jointReductionFactors.get(jointIndex);
      }
      return ret;
   }
}
