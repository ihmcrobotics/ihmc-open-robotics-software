package us.ihmc.simulationToolkit.controllers;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.math.filters.FilteredVelocityYoFrameVector;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class SimulatedRobotCenterOfMassVisualizer implements RobotController
{
   private final Robot robot;

   private final YoRegistry registry = new YoRegistry("ExactCoMCalcualtor");
   private final YoFramePoint3D exactCenterOfMassPosition = new YoFramePoint3D("exactCenterOfMassPosition", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector3D exactCenterOfMassVelocity = new YoFrameVector3D("exactCenterOfMassVelocity", ReferenceFrame.getWorldFrame(), registry);
   private final FilteredVelocityYoFrameVector exactCenterOfMassAcceleration;
   
   
   private final Point3D tempCenterOfMassPoint = new Point3D();
   private final Vector3D tempCenterOfMassVelocity = new Vector3D();
   private final Vector3D tempAngularMomentum = new Vector3D();


   public SimulatedRobotCenterOfMassVisualizer(Robot robot, double dt)
   {
      this.robot = robot;
      YoDouble alphaSimCoMAcceleration = new YoDouble("alphaSimCoMAcceleration", registry);
      exactCenterOfMassAcceleration = FilteredVelocityYoFrameVector.createFilteredVelocityYoFrameVector("exactCenterOfMassAcceleration", "", alphaSimCoMAcceleration, dt, registry, exactCenterOfMassVelocity);
      alphaSimCoMAcceleration.set(0.99);
   }


   @Override
   public void doControl()
   {
      double mass = robot.computeCOMMomentum(tempCenterOfMassPoint, tempCenterOfMassVelocity, tempAngularMomentum);

      exactCenterOfMassPosition.set(tempCenterOfMassPoint);
      tempCenterOfMassVelocity.scale(1.0 / mass);
      exactCenterOfMassVelocity.set(tempCenterOfMassVelocity);
      
      exactCenterOfMassAcceleration.update();
   }


   @Override
   public void initialize()
   {
   }


   @Override
   public YoRegistry getYoRegistry()
   {
      return registry;
   }


   @Override
   public String getName()
   {
      return getClass().getSimpleName();
   }


   @Override
   public String getDescription()
   {
      return getName();
   }
}
