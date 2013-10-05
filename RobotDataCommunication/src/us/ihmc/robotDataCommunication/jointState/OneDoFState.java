package us.ihmc.robotDataCommunication.jointState;

import java.nio.LongBuffer;

import com.yobotics.simulationconstructionset.OneDegreeOfFreedomJoint;

public class OneDoFState extends JointState<OneDegreeOfFreedomJoint>
{
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
      joint.setQ(q);
      joint.setQd(qd);
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
      return 2;
   }

}
