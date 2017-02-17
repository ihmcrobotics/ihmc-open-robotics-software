package us.ihmc.valkyrieRosControl.dataHolders;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.sensors.ForceSensorDefinition;
import us.ihmc.rosControl.wholeRobot.ForceTorqueSensorHandle;

public class YoForceTorqueSensorHandle
{
   private final ForceTorqueSensorHandle handle;
   private final ForceSensorDefinition forceSensorDefinition;

   private final DoubleYoVariable tx, ty, tz;
   private final DoubleYoVariable fx, fy, fz;

   public YoForceTorqueSensorHandle(ForceTorqueSensorHandle handle, ForceSensorDefinition forceSensorDefinition, YoVariableRegistry parentRegistry)
   {
      this.handle = handle;
      this.forceSensorDefinition = forceSensorDefinition;

      String name = forceSensorDefinition.getSensorName();
      YoVariableRegistry registry = new YoVariableRegistry(name);

      this.tx = new DoubleYoVariable(name + "_tx", registry);
      this.ty = new DoubleYoVariable(name + "_ty", registry);
      this.tz = new DoubleYoVariable(name + "_tz", registry);

      this.fx = new DoubleYoVariable(name + "_fx", registry);
      this.fy = new DoubleYoVariable(name + "_fy", registry);
      this.fz = new DoubleYoVariable(name + "_fz", registry);
   }

   public void update()
   {
      this.tx.set(handle.getTx());
      this.ty.set(handle.getTy());
      this.tz.set(handle.getTz());

      this.fx.set(handle.getFx());
      this.fy.set(handle.getFy());
      this.fz.set(handle.getFz());
   }

   public ForceSensorDefinition getForceSensorDefinition()
   {
      return forceSensorDefinition;
   }

   
   public void packWrench(DenseMatrix64F torqueForce)
   {
      torqueForce.set(0, tx.getDoubleValue());
      torqueForce.set(1, ty.getDoubleValue());
      torqueForce.set(2, tz.getDoubleValue());

      torqueForce.set(3, fx.getDoubleValue());
      torqueForce.set(4, fy.getDoubleValue());
      torqueForce.set(5, fz.getDoubleValue());
   }
}
