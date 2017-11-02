package us.ihmc.valkyrieRosControl.dataHolders;

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

   private final YoDouble tauMeasured;
   private final YoDouble q;
   private final YoDouble qd;

   private final YoDouble controllerQddDesired;
   private final YoDouble controllerTauDesired;
   private final YoDouble tauDesired;

   public YoEffortJointHandleHolder(EffortJointHandle handle, OneDoFJoint joint, JointDesiredOutput desiredJointData, YoVariableRegistry parentRegistry)
   {
      this.name = handle.getName();
      YoVariableRegistry registry = new YoVariableRegistry(name);

      this.handle = handle;
      this.joint = joint;
      this.desiredJointData = desiredJointData != null ? desiredJointData : new JointDesiredOutput();

      this.tauMeasured = new YoDouble(name + "TauMeasured", registry);
      this.q = new YoDouble(name + "_q", registry);
      this.qd = new YoDouble(name + "_qd", registry);
      this.controllerQddDesired = new YoDouble(name + "ControllerQddDesired", registry);
      this.controllerTauDesired = new YoDouble(name + "ControllerTauDesired", registry);
      this.tauDesired = new YoDouble(name + "TauDesired", registry);

      parentRegistry.addChild(registry);
   }

   public void update()
   {
      this.q.set(handle.getPosition());
      this.qd.set(handle.getVelocity());
      this.tauMeasured.set(handle.getEffort());
      if (desiredJointData.hasDesiredTorque())
         this.controllerTauDesired.set(desiredJointData.getDesiredTorque());
      else
         this.controllerTauDesired.set(0.0);
      
      if (desiredJointData.hasDesiredAcceleration())
         this.controllerQddDesired.set(desiredJointData.getDesiredAcceleration());
      else
         this.controllerQddDesired.set(0.0);
   }

   public void setDesiredEffort(double effort)
   {
      this.tauDesired.set(effort);
      handle.setDesiredEffort(effort);
   }

   public OneDoFJoint getOneDoFJoint()
   {
      return joint;
   }

   public JointDesiredOutput getDesiredJointData()
   {
      return desiredJointData;
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

   public double getControllerTauDesired()
   {
      return controllerTauDesired.getDoubleValue();
   }

   public void addOffetControllerTauDesired(double effortOffset)
   {
      controllerTauDesired.add(effortOffset);
   }

   public double getControllerQddDesired()
   {
      return controllerQddDesired.getDoubleValue();
   }

   public String getName()
   {
      return name;
   }
}
