package us.ihmc.robotModels.description;

import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.robotDescription.LoopClosurePinConstraintDescription;
import us.ihmc.robotics.robotDescription.OneDoFJointDescription;
import us.ihmc.robotics.robotDescription.PinJointDescription;

public class CrossFourBarJointDescription extends OneDoFJointDescription
{
   private PinJointDescription[] fourBarJoints;
   private LoopClosurePinConstraintDescription fourBarClosure;
   private int actuatedJointIndex;

   public CrossFourBarJointDescription(String name)
   {
      super(name, new Vector3D(Double.NaN, Double.NaN, Double.NaN), new Vector3D(Double.NaN, Double.NaN, Double.NaN));
   }

   public void setActuatedJointIndex(int actuatedJointIndex)
   {
      this.actuatedJointIndex = actuatedJointIndex;
   }

   public void setFourBarJoints(PinJointDescription[] fourBarJoints)
   {
      this.fourBarJoints = fourBarJoints;
   }

   public void setFourBarClosure(LoopClosurePinConstraintDescription fourBarClosure)
   {
      this.fourBarClosure = fourBarClosure;
   }

   public int getActuatedJointIndex()
   {
      return actuatedJointIndex;
   }

   public PinJointDescription[] getFourBarJoints()
   {
      return fourBarJoints;
   }

   public LoopClosurePinConstraintDescription getFourBarClosure()
   {
      return fourBarClosure;
   }

   @Override
   public CrossFourBarJointDescription copy()
   {
      throw new UnsupportedOperationException();
   }
}
