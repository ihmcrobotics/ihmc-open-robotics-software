package us.ihmc.exampleSimulations.sphereICPControl.controllers;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.robotics.controllers.PIDController;
import us.ihmc.robotics.math.frames.YoFramePoint2d;
import us.ihmc.robotics.screwTheory.CenterOfMassJacobian;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class BasicPlanarController
{
   private static final double desiredX = 0.0;
   private static final double desiredY = 0.0;

   private static final double kp = 10.0;
   private static final double ki = 0.0;
   private static final double kd = 1.0;

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final FullRobotModel robotModel;

   private final ReferenceFrame centerOfMassFrame;

   private final FramePoint3D centerOfMass = new FramePoint3D();
   private final FrameVector3D centerOfMassVelocity = new FrameVector3D();

   private final YoDouble yoDesiredX = new YoDouble("desiredX", registry);
   private final YoDouble yoDesiredY = new YoDouble("desiredY", registry);

   private final YoDouble xKp = new YoDouble("planarXKp", registry);
   private final YoDouble xKi = new YoDouble("planarXKi", registry);
   private final YoDouble xKd = new YoDouble("planarXKd", registry);
   private final YoDouble xMaxIntegralError = new YoDouble("xMaxIntegralError", registry);
   private final YoDouble yKp = new YoDouble("planarYKp", registry);
   private final YoDouble yKi = new YoDouble("planarYKi", registry);
   private final YoDouble yKd = new YoDouble("planarYKd", registry);
   private final YoDouble yMaxIntegralError = new YoDouble("yMaxIntegralError", registry);

   private final PIDController xController;
   private final PIDController yController;

   private final YoFramePoint2d planarForces = new YoFramePoint2d("planarForces", null, registry);
   private final double controlDT;
   private final CenterOfMassJacobian centerOfMassJacobian;

   public BasicPlanarController(SphereControlToolbox controlToolbox, YoVariableRegistry parentRegistry)
   {
      this.robotModel = controlToolbox.getFullRobotModel();
      this.controlDT = controlToolbox.getControlDT();

      centerOfMassFrame = controlToolbox.getCenterOfMassFrame();
      centerOfMassJacobian = controlToolbox.getCenterOfMassJacobian();

      yoDesiredX.set(desiredX);
      yoDesiredY.set(desiredY);

      xKp.set(kp);
      xKd.set(kd);
      xKi.set(ki);
      xMaxIntegralError.set(Double.POSITIVE_INFINITY);
      yKp.set(kp);
      yKd.set(kd);
      yKi.set(ki);
      yMaxIntegralError.set(Double.POSITIVE_INFINITY);

      xController = new PIDController(xKp, xKi, xKd, xMaxIntegralError, "xController", registry);
      yController = new PIDController(yKp, yKi, yKd, yMaxIntegralError, "yController", registry);

      parentRegistry.addChild(registry);
   }

   public void doControl()
   {
      centerOfMass.setToZero(centerOfMassFrame);
      centerOfMass.changeFrame(worldFrame);

      centerOfMassJacobian.getCenterOfMassVelocity(centerOfMassVelocity);
      centerOfMassVelocity.changeFrame(worldFrame);

      double xForce = xController.compute(centerOfMass.getX(), yoDesiredX.getDoubleValue(), centerOfMassVelocity.getX(), 0.0, controlDT);
      double yForce = yController.compute(centerOfMass.getY(), yoDesiredY.getDoubleValue(), centerOfMassVelocity.getY(), 0.0, controlDT);
      planarForces.set(xForce, yForce);
   }

   public void getPlanarForces(Point2D forces)
   {
      forces.set(planarForces);
   }

}
