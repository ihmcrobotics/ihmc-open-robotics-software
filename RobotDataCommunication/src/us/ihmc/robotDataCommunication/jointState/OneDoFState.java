package us.ihmc.robotDataCommunication.jointState;

import java.nio.LongBuffer;

import us.ihmc.simulationconstructionset.OneDegreeOfFreedomJoint;

public class OneDoFState extends JointState<OneDegreeOfFreedomJoint>
{
   public static final int numberOfStateVariables = 2;
   
   private double q;
   private double qd;
   
   public OneDoFState(String name)
   {
      super(name);
   }

   @Override
   public void update(LongBuffer buffer)
   {
      q = Double.longBitsToDouble(buffer.get());
      qd = Double.longBitsToDouble(buffer.get());
   }

   @Override
   public void get(OneDegreeOfFreedomJoint joint)
   {
      if(!Double.isNaN(q))
      {
         joint.setQ(q);
      }
      if(!Double.isNaN(qd))
      {
         joint.setQd(qd);
      }
   }

   @Override
   public void get(double[] array)
   {
      array[0] = q;
      array[1] = qd;
   }

   @Override
   public int getNumberOfStateVariables()
   {
      return numberOfStateVariables;
   }

}
