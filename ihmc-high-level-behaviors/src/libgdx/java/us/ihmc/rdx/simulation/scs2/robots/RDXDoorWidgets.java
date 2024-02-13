package us.ihmc.rdx.simulation.scs2.robots;

import imgui.type.ImDouble;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.scs2.simulation.robot.Robot;
import us.ihmc.scs2.simulation.robot.multiBodySystem.SimRevoluteJoint;

public class RDXDoorWidgets
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImDouble hingeTorque = new ImDouble();
   private final ImDouble leverTorque = new ImDouble();
   private SimRevoluteJoint doorHingeJoint;
   private SimRevoluteJoint doorLeverJoint;

   public void initialize(Robot doorRobot)
   {
      doorHingeJoint = (SimRevoluteJoint) doorRobot.getJoint("doorHingeJoint");
      doorLeverJoint = (SimRevoluteJoint) doorRobot.getJoint("doorLeverJoint");

      doorRobot.addController(() ->
      {
         doorHingeJoint.setTau(hingeTorque.get());
         doorLeverJoint.setTau(doorLeverJoint.getTau() + leverTorque.get());
      });
   }

   public void renderImGuiWidgets()
   {
      ImGuiTools.sliderDouble(labels.get("Hinge torque"), hingeTorque, -100.0, 100.0);
      ImGuiTools.sliderDouble(labels.get("Lever torque"), leverTorque, -2.0, 2.0);
   }

   public void zeroTorques()
   {
      hingeTorque.set(0.0);
      leverTorque.set(0.0);
   }
}
