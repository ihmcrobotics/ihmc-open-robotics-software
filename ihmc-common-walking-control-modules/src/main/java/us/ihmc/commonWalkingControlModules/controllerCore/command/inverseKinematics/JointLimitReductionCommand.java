package us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics;

import java.util.ArrayList;
import java.util.List;

import gnu.trove.list.array.TDoubleArrayList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommandType;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.screwTheory.OneDoFJoint;

public class JointLimitReductionCommand implements InverseKinematicsCommand<JointLimitReductionCommand>, InverseDynamicsCommand<JointLimitReductionCommand>
{
   private final int initialCapacity = 40;
   private final List<String> jointNames = new ArrayList<>(initialCapacity);
   private final List<OneDoFJoint> joints = new ArrayList<>(initialCapacity);
   private final TDoubleArrayList jointReductionFactors = new TDoubleArrayList(initialCapacity);

   public JointLimitReductionCommand()
   {
   }

   public void clear()
   {
      jointNames.clear();
      joints.clear();
      jointReductionFactors.reset();
   }

   public void addReductionFactor(OneDoFJoint joint, double reductionFactor)
   {
      MathTools.checkIntervalContains(reductionFactor, 0.0, 1.0);
      jointNames.add(joint.getName());
      joints.add(joint);
      jointReductionFactors.add(reductionFactor);
   }

   @Override
   public void set(JointLimitReductionCommand other)
   {
      clear();

      for (int i = 0; i < other.getNumberOfJoints(); i++)
      {
         jointNames.add(other.jointNames.get(i));
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

   public OneDoFJoint getJoint(int jointIndex)
   {
      return joints.get(jointIndex);
   }

   @Override
   public ControllerCoreCommandType getCommandType()
   {
      return ControllerCoreCommandType.LIMIT_REDUCTION;
   }
}
