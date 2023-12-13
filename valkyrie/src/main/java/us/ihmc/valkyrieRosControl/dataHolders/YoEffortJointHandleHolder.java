package us.ihmc.valkyrieRosControl.dataHolders;

import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.YoJointDesiredOutput;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.rosControl.EffortJointHandle;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutput;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputBasics;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class YoEffortJointHandleHolder
{
   private final String name;
   private final EffortJointHandle handle;
   private final OneDoFJointBasics joint;
   private final JointDesiredOutputBasics desiredJointData;
   private final YoJointDesiredOutput yoDesiredJointData;
   private final DoubleParameter tauMax;

   private final YoDouble tauDesired;
   private final YoDouble tauMeasured;
   private final YoDouble q;
   private final YoDouble qd;

   public YoEffortJointHandleHolder(EffortJointHandle handle, OneDoFJointBasics joint, JointDesiredOutputBasics desiredJointData, YoRegistry parentRegistry)
   {
      this.name = handle.getName();
      YoRegistry registry = new YoRegistry(name);

      this.handle = handle;
      this.joint = joint;
      this.desiredJointData = desiredJointData != null ? desiredJointData : new JointDesiredOutput();
      this.tauMax = new DoubleParameter(handle.getName() + "TauMax", registry, Double.POSITIVE_INFINITY);

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
   }

   public void updateControllerOutput()
   {
      yoDesiredJointData.set(desiredJointData);
   }

   public void setDesiredEffort(double effort)
   {
      tauDesired.set(effort);
      handle.setDesiredEffort(effort);
   }

   public OneDoFJointBasics getOneDoFJoint()
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

   public double getTauMax()
   {
      return tauMax.getValue();
   }
}
