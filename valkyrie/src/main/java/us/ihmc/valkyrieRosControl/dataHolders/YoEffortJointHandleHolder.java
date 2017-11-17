package us.ihmc.valkyrieRosControl.dataHolders;

import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.YoJointDesiredOutput;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.rosControl.EffortJointHandle;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutput;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class YoEffortJointHandleHolder
{
   private final String name;
   private final EffortJointHandle handle;
   private final OneDoFJoint joint;
   private final JointDesiredOutput desiredJointData;
   private final YoJointDesiredOutput yoDesiredJointData;

   private final YoDouble tauDesired;
   private final YoDouble tauMeasured;
   private final YoDouble q;
   private final YoDouble qd;

   public YoEffortJointHandleHolder(EffortJointHandle handle, OneDoFJoint joint, JointDesiredOutput desiredJointData, YoVariableRegistry parentRegistry)
   {
      this.name = handle.getName();
      YoVariableRegistry registry = new YoVariableRegistry(name);

      this.handle = handle;
      this.joint = joint;
      this.desiredJointData = desiredJointData != null ? desiredJointData : new JointDesiredOutput();

      yoDesiredJointData = new YoJointDesiredOutput(name, registry, "JointHandle");

      this.tauDesired = new YoDouble(name + "TauDesired", registry);
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
      yoDesiredJointData.set(desiredJointData);
   }

   public void setDesiredEffort(double effort)
   {
      tauDesired.set(effort);
      handle.setDesiredEffort(effort);
   }

   public OneDoFJoint getOneDoFJoint()
   {
      return joint;
   }

   public YoJointDesiredOutput getDesiredJointData()
   {
      return yoDesiredJointData;
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
      return name;
   }
}
