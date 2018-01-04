package us.ihmc.simulationconstructionset;

import us.ihmc.euclid.Axis;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.yoVariables.listener.VariableChangedListener;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoVariable;

/**
 * creates a pin joint with a pd controller built in.
 * This calls compute on the pd controller every time getTau is called.
 * @author steel
 *
 */
@SuppressWarnings("serial")
public class SpringPinJoint extends PinJoint
{
   
   private final YoDouble kp;
   private final YoDouble kd;
   
   public SpringPinJoint(String jname, Vector3DReadOnly offset, Robot rob, Vector3DReadOnly u_hat)
   {
      super(jname, offset, rob, u_hat);
      kp = new YoDouble(jname + "_kp", registry);
      kd = new YoDouble(jname + "_kd", registry);
      setKp(8000.0);
      setKd(200.0);
//      setChangeListeners();
   }

   public SpringPinJoint(String jname, Vector3DReadOnly offset, Robot rob, Axis jaxis)
   {
      super(jname, offset, rob, jaxis);
      kp = new YoDouble(jname + "_kp", registry);
      kd = new YoDouble(jname + "_kd", registry);
      setKp(10000.0);
      setKd(200.0);
//      setChangeListeners();
   }
   
   private void setChangeListeners()
   {
      kp.addVariableChangedListener(new VariableChangedListener()
      {
         @Override
         public void notifyOfVariableChange(YoVariable<?> v)
         {
            setKp(kp.getDoubleValue());
         }
      });
      kd.addVariableChangedListener(new VariableChangedListener()
      {
         @Override
         public void notifyOfVariableChange(YoVariable<?> v)
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
   public YoDouble getTauYoVariable()
   {
      return tau;
   }

}
