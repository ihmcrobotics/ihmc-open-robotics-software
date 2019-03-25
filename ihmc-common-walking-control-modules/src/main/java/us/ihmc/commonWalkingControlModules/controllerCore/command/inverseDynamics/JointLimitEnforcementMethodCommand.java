package us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommandType;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.InverseKinematicsCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.JointLimitEnforcement;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.JointLimitParameters;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;

public class JointLimitEnforcementMethodCommand
      implements InverseKinematicsCommand<JointLimitEnforcementMethodCommand>, InverseDynamicsCommand<JointLimitEnforcementMethodCommand>
{
   private static final int initialCapacity = 40;
   private final List<OneDoFJointBasics> joints = new ArrayList<>(initialCapacity);
   private final List<JointLimitEnforcement> methods = new ArrayList<>(initialCapacity);
   private final List<JointLimitParameters> parameters = new ArrayList<>(initialCapacity);

   public void clear()
   {
      joints.clear();
      methods.clear();
      parameters.clear();
   }

   public void addLimitEnforcementMethod(OneDoFJointBasics joint, JointLimitEnforcement method, JointLimitParameters limitParameters)
   {
      joints.add(joint);
      methods.add(method);
      parameters.add(limitParameters);
   }

   public JointLimitEnforcement getJointLimitReductionFactor(int jointIndex)
   {
      return methods.get(jointIndex);
   }

   public JointLimitParameters getJointLimitParameters(int jointIndex)
   {
      return parameters.get(jointIndex);
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
   public void set(JointLimitEnforcementMethodCommand other)
   {
      clear();

      for (int i = 0; i < other.getNumberOfJoints(); i++)
      {
         joints.add(other.joints.get(i));
         methods.add(other.methods.get(i));
         parameters.add(other.parameters.get(i));
      }
   }

   @Override
   public ControllerCoreCommandType getCommandType()
   {
      return ControllerCoreCommandType.JOINT_LIMIT_ENFORCEMENT;
   }

   @Override
   public boolean equals(Object object)
   {
      if (object == this)
      {
         return true;
      }
      else if (object instanceof JointLimitEnforcementMethodCommand)
      {
         JointLimitEnforcementMethodCommand other = (JointLimitEnforcementMethodCommand) object;

         if (getNumberOfJoints() != other.getNumberOfJoints())
            return false;

         for (int jointIndex = 0; jointIndex < getNumberOfJoints(); jointIndex++)
         {
            if (getJoint(jointIndex) != other.getJoint(jointIndex))
               return false;
            if (getJointLimitReductionFactor(jointIndex) != other.getJointLimitReductionFactor(jointIndex))
               return false;
            if (!getJointLimitParameters(jointIndex).equals(other.getJointLimitParameters(jointIndex)))
               return false;
         }

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
         ret += "\nJoint: " + getJoint(jointIndex).getName() + ", method: " + getJointLimitReductionFactor(jointIndex) + ", "
               + getJointLimitParameters(jointIndex).toString();
      }
      return ret;
   }
}
