package us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommandType;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.JointLimitEnforcement;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.JointLimitParameters;
import us.ihmc.robotics.screwTheory.OneDoFJoint;

public class JointLimitEnforcementMethodCommand implements InverseKinematicsCommand<JointLimitEnforcementMethodCommand>, InverseDynamicsCommand<JointLimitEnforcementMethodCommand>
{
   private final int initialCapacity = 40;
   private final List<String> jointNames = new ArrayList<>(initialCapacity);
   private final List<OneDoFJoint> joints = new ArrayList<>(initialCapacity);
   private final List<JointLimitEnforcement> methods = new ArrayList<>(initialCapacity);
   private final List<JointLimitParameters> parameters = new ArrayList<>(initialCapacity);

   public void clear()
   {
      jointNames.clear();
      joints.clear();
      methods.clear();
      parameters.clear();
   }

   public void addLimitEnforcementMethod(OneDoFJoint joint, JointLimitEnforcement method, JointLimitParameters limitParameters)
   {
      jointNames.add(joint.getName());
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

   public OneDoFJoint getJoint(int jointIndex)
   {
      return joints.get(jointIndex);
   }

   @Override
   public void set(JointLimitEnforcementMethodCommand other)
   {
      clear();

      for (int i = 0; i < other.getNumberOfJoints(); i++)
      {
         jointNames.add(other.jointNames.get(i));
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

}
