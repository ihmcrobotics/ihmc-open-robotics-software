package us.ihmc.robotics.physics;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.mecano.algorithms.MultiBodyResponseCalculator;
import us.ihmc.mecano.multiBodySystem.interfaces.JointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointReadOnly;
import us.ihmc.mecano.tools.JointStateType;

public interface JointStateProvider
{
   JointStateType getState();

   DenseMatrix64F getJointState(JointReadOnly joint);

   default double getJointState(OneDoFJointReadOnly joint)
   {
      DenseMatrix64F state = getJointState((JointReadOnly) joint);
      if (state == null)
         return Double.NaN;
      else
         return state.get(0);
   }

   public static JointStateProvider toJointTwistProvider(MultiBodyResponseCalculator calculator)
   {
      return new JointStateProvider()
      {
         @Override
         public JointStateType getState()
         {
            return JointStateType.VELOCITY;
         }

         @Override
         public DenseMatrix64F getJointState(JointReadOnly joint)
         {
            return calculator.getJointTwistChange(joint);
         }
      };
   }
}
