package us.ihmc.exampleSimulations.sphereICPControl.controllers;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.mecano.algorithms.CenterOfMassJacobian;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.robotics.controllers.PIDController;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class BasicHeightController
{
   private static final double kp = 100.0;
   private static final double ki = 0.0;
   private static final double kd = 10.0;

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final FullRobotModel robotModel;

   private final ReferenceFrame centerOfMassFrame;

   private final FramePoint3D centerOfMass = new FramePoint3D();
   private final FrameVector3D centerOfMassVelocity = new FrameVector3D();

   private final YoFramePoint3D yoCenterOfMass = new YoFramePoint3D("centerOfMass", worldFrame, registry);
   private final YoDouble yoDesiredHeight = new YoDouble("desiredHeight", registry);

   private final YoDouble heightKp = new YoDouble("heightKp", registry);
   private final YoDouble heightKi = new YoDouble("heightKi", registry);
   private final YoDouble heightKd = new YoDouble("heightKd", registry);
   private final YoDouble maxIntegralError = new YoDouble("heightMaxIntegralError", registry);

   private final PIDController heightController;

   private final YoDouble verticalForce = new YoDouble("verticalForce", registry);
   private final double controlDT;
   private final CenterOfMassJacobian centerOfMassJacobian;

   public BasicHeightController(SphereControlToolbox controlToolbox, YoRegistry parentRegistry)
   {
      this.robotModel = controlToolbox.getFullRobotModel();
      this.controlDT = controlToolbox.getControlDT();

      centerOfMassFrame = controlToolbox.getCenterOfMassFrame();
      centerOfMassJacobian = controlToolbox.getCenterOfMassJacobian();

      yoDesiredHeight.set(controlToolbox.getDesiredHeight());

      heightKp.set(kp);
      heightKd.set(kd);
      heightKi.set(ki);
      maxIntegralError.set(Double.POSITIVE_INFINITY);

      heightController = new PIDController(heightKp, heightKi, heightKd, maxIntegralError, "heightController", registry);

      parentRegistry.addChild(registry);
   }

   public void doControl()
   {
      centerOfMass.setToZero(centerOfMassFrame);
      centerOfMass.changeFrame(worldFrame);

      centerOfMassVelocity.setIncludingFrame(centerOfMassJacobian.getCenterOfMassVelocity());
      centerOfMassVelocity.changeFrame(worldFrame);

      yoCenterOfMass.set(centerOfMass);

      verticalForce.set(heightController.compute(centerOfMass.getZ(), yoDesiredHeight.getDoubleValue(), centerOfMassVelocity.getZ(), 0.0, controlDT));
   }

   public double getVerticalForce()
   {
      return verticalForce.getDoubleValue();
   }

}
