package us.ihmc.commonWalkingControlModules.capturePoint.lqrControl;

import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.robotics.controllers.PIDController;
import us.ihmc.robotics.controllers.pidGains.implementations.YoPIDGains;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class BasicHeightController
{
   private static final double kp = 100.0;
   private static final double ki = 0.0;
   private static final double kd = 10.0;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final YoDouble yoDesiredHeight = new YoDouble("desiredHeight", registry);
   private final YoDouble weight = new YoDouble("weight", registry);

   private final PIDController heightController;

   private final YoDouble verticalForce = new YoDouble("verticalForce", registry);
   private final double controlDT;

   private final FramePoint3DReadOnly centerOfMass;
   private final FrameVector3DReadOnly centerOfMassVelocity;

   private final SphereControlToolbox controlToolbox;

   public BasicHeightController(SphereControlToolbox controlToolbox, YoVariableRegistry parentRegistry)
   {
      this.controlDT = controlToolbox.getControlDT();
      this.controlToolbox = controlToolbox;

      yoDesiredHeight.set(controlToolbox.getDesiredHeight());

      centerOfMass = controlToolbox.getCenterOfMass();
      centerOfMassVelocity = controlToolbox.getCenterOfMassVelocity();

      YoPIDGains pidGains = new YoPIDGains("height", registry);
      pidGains.setKp(kp);
      pidGains.setKd(kd);
      pidGains.setKi(ki);

      heightController = new PIDController(pidGains, "heightController", registry);

      parentRegistry.addChild(registry);
   }

   public void doControl()
   {
      double z = centerOfMass.getZ();

      weight.set(controlToolbox.getGravityZ() * controlToolbox.getTotalMass());
      verticalForce.set(heightController.compute(z, yoDesiredHeight.getDoubleValue(), centerOfMassVelocity.getZ(), 0.0, controlDT));
      verticalForce.add(Math.abs(weight.getDoubleValue()));
   }

   public double getVerticalForce()
   {
      return verticalForce.getDoubleValue();
   }

}
