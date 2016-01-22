package us.ihmc.valkyrieRosControl.dataHolders;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.rosControl.JointHandle;
import us.ihmc.sensorProcessing.model.DesiredJointDataHolder;
import us.ihmc.sensorProcessing.model.DesiredJointDataHolder.DesiredJointData;

public class YoJointHandleHolder
{
   private final String name;
   private final JointHandle handle;
   private final OneDoFJoint joint;
   private final DesiredJointDataHolder.DesiredJointData desiredJointData;

   private final DoubleYoVariable tauMeasured;
   private final DoubleYoVariable q;
   private final DoubleYoVariable qd;

   private final DoubleYoVariable controllerQddDesired;
   private final DoubleYoVariable controllerTauDesired;
   private final DoubleYoVariable tauDesired;

   public YoJointHandleHolder(JointHandle handle, OneDoFJoint joint, DesiredJointData desiredJointData, YoVariableRegistry parentRegistry)
   {
      this.name = handle.getName();
      YoVariableRegistry registry = new YoVariableRegistry(name);

      this.handle = handle;
      this.joint = joint;
      this.desiredJointData = desiredJointData;

      this.tauMeasured = new DoubleYoVariable(name + "TauMeasured", registry);
      this.q = new DoubleYoVariable(name + "_q", registry);
      this.qd = new DoubleYoVariable(name + "_qd", registry);
      this.controllerQddDesired = new DoubleYoVariable(name + "ControllerQddDesired", registry);
      this.controllerTauDesired = new DoubleYoVariable(name + "ControllerTauDesired", registry);
      this.tauDesired = new DoubleYoVariable(name + "TauDesired", registry);

      parentRegistry.addChild(registry);
   }

   public void update()
   {
      this.q.set(handle.getPosition());
      this.qd.set(handle.getVelocity());
      this.tauMeasured.set(handle.getEffort());
      this.controllerTauDesired.set(desiredJointData.getTauDesired());
      this.controllerQddDesired.set(desiredJointData.getQddDesired());
   }

   public void setDesiredEffort(double effort)
   {
      this.tauDesired.set(effort);
      handle.setDesiredEffort(effort);
   }

   public void addDesiredEffort(double effortOffset)
   {
      setDesiredEffort(tauDesired.getDoubleValue() + effortOffset);
   }

   public OneDoFJoint getOneDoFJoint()
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

   public double getControllerTauDesired()
   {
      return controllerTauDesired.getDoubleValue();
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
