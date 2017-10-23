package us.ihmc.valkyrieRosControl.dataHolders;

import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.rosControl.wholeRobot.PositionJointHandle;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputReadOnly;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class YoPositionJointHandleHolder
{
   private final String name;
   private final PositionJointHandle handle;
   private final OneDoFJoint joint;
   private final JointDesiredOutputReadOnly desiredJointData;

   private final YoDouble q;
   private final YoDouble qd;

   private final YoDouble controllerPositionDesired;
   private final YoDouble positionDesired;

   public YoPositionJointHandleHolder(PositionJointHandle handle, OneDoFJoint joint, JointDesiredOutputReadOnly desiredJointData,
                                      YoVariableRegistry parentRegistry)
   {
      this.name = handle.getName();
      YoVariableRegistry registry = new YoVariableRegistry(name);

      this.handle = handle;
      this.joint = joint;
      this.desiredJointData = desiredJointData;

      this.q = new YoDouble(name + "_q", registry);
      this.qd = new YoDouble(name + "_qd", registry);
      this.controllerPositionDesired = new YoDouble(name + "ControllerPositionDesired", registry);
      this.positionDesired = new YoDouble(name + "PositionDesired", registry);

      parentRegistry.addChild(registry);
   }

   public void update()
   {
      this.q.set(handle.getPosition());
      this.qd.set(handle.getVelocity());
      if (desiredJointData.hasDesiredPosition())
         this.controllerPositionDesired.set(desiredJointData.getDesiredPosition());
      // Otherwise, don't change the desired position
   }

   public void setDesiredPosition(double position)
   {
      this.positionDesired.set(position);
      handle.setDesiredPosition(position);
   }

   public OneDoFJoint getOneDoFJoint()
   {
      return joint;
   }

   public double getQ()
   {
      return q.getDoubleValue();
   }

   public double getQd()
   {
      return qd.getDoubleValue();
   }

   public double getControllerPositionDesired()
   {
      return controllerPositionDesired.getDoubleValue();
   }

   public String getName()
   {
      return name;
   }
}
