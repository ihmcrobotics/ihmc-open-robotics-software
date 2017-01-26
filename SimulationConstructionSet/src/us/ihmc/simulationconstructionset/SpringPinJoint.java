package us.ihmc.simulationconstructionset;

import javax.vecmath.Vector3d;

import us.ihmc.robotics.Axis;
import us.ihmc.robotics.dataStructures.listener.VariableChangedListener;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.YoVariable;

/**
 * creates a pin joint with a pd controller built in.
 * This calls compute on the pd controller every time getTau is called.
 * @author steel
 *
 */
@SuppressWarnings("serial")
public class SpringPinJoint extends PinJoint
{
   
   private final DoubleYoVariable kp;
   private final DoubleYoVariable kd;
   
   public SpringPinJoint(String jname, Vector3d offset, Robot rob, Vector3d u_hat)
   {
      super(jname, offset, rob, u_hat);
      kp = new DoubleYoVariable(jname + "_kp", registry);
      kd = new DoubleYoVariable(jname + "_kd", registry);
      setKp(8000.0);
      setKd(200.0);
//      setChangeListeners();
   }

   public SpringPinJoint(String jname, Vector3d offset, Robot rob, Axis jaxis)
   {
      super(jname, offset, rob, jaxis);
      kp = new DoubleYoVariable(jname + "_kp", registry);
      kd = new DoubleYoVariable(jname + "_kd", registry);
      setKp(10000.0);
      setKd(200.0);
//      setChangeListeners();
   }
   
   private void setChangeListeners()
   {
      kp.addVariableChangedListener(new VariableChangedListener()
      {
         @Override
         public void variableChanged(YoVariable<?> v)
         {
            setKp(kp.getDoubleValue());
         }
      });
      kd.addVariableChangedListener(new VariableChangedListener()
      {
         @Override
         public void variableChanged(YoVariable<?> v)
         {
            setKd(kd.getDoubleValue());
         }
      });
   }
   
   @Override
   public void setKp(double kp)
   {
      super.setKp(kp);
      this.kp.set(kp, false);
   }
   
   @Override
   public void setKd(double kd)
   {
      super.setKd(kd);
      this.kd.set(kd, false);
   }
   
   @Override
   public double getTau()
   {
      compute();
      return tau.getDoubleValue();
   }

   public void compute()
   {
      tau.set(doPDControl());
   }

   @Override
   public DoubleYoVariable getTauYoVariable()
   {
      return tau;
   }

}
