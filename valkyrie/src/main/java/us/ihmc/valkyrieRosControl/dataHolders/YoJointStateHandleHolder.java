package us.ihmc.valkyrieRosControl.dataHolders;

import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.rosControl.wholeRobot.JointStateHandle;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

/**
 * Created by parallels on 7/6/16.
 */
public class YoJointStateHandleHolder
{
   private final String name;
   private final JointStateHandle handle;
   private final OneDoFJointBasics joint;

   private final YoDouble tauMeasured;
   private final YoDouble q;
   private final YoDouble qd;

   public YoJointStateHandleHolder(JointStateHandle handle, OneDoFJointBasics joint, YoRegistry parentRegistry)
   {
      this.name = handle.getName();
      YoRegistry registry = new YoRegistry(name);

      this.handle = handle;
      this.joint = joint;

      this.tauMeasured = new YoDouble(name + "TauMeasured", registry);
      this.q = new YoDouble(name + "_q", registry);
      this.qd = new YoDouble(name + "_qd", registry);

      parentRegistry.addChild(registry);
   }

   public void update()
   {
      this.q.set(handle.getPosition());
      this.qd.set(handle.getVelocity());
      this.tauMeasured.set(handle.getEffort());
   }

   public OneDoFJointBasics getOneDoFJoint()
   {
      return joint;
   }

   public double getTauMeasured()
   {
      return tauMeasured.getDoubleValue();
   }

   public double getQ()
   {
      return q.getDoubleValue();
   }

   public double getQd()
   {
      return qd.getDoubleValue();
   }

   public String getName()
   {
      return this.name;
   }
}
