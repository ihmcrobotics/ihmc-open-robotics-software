package us.ihmc.simulationToolkit.controllers;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.math.filters.FilteredVelocityYoFrameVector;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotController.RobotController;
import us.ihmc.simulationconstructionset.Robot;

public class SimulatedRobotCenterOfMassVisualizer implements RobotController
{
   private final Robot robot;

   private final YoVariableRegistry registry = new YoVariableRegistry("ExactCoMCalcualtor");
   private final YoFramePoint exactCenterOfMassPosition = new YoFramePoint("exactCenterOfMassPosition", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector exactCenterOfMassVelocity = new YoFrameVector("exactCenterOfMassVelocity", ReferenceFrame.getWorldFrame(), registry);
   private final FilteredVelocityYoFrameVector exactCenterOfMassAcceleration;
   
   
   private final Point3d tempCenterOfMassPoint = new Point3d();
   private final Vector3d tempCenterOfMassVelocity = new Vector3d();
   private final Vector3d tempAngularMomentum = new Vector3d();


   public SimulatedRobotCenterOfMassVisualizer(Robot robot, double dt)
   {
      this.robot = robot;
      DoubleYoVariable alphaSimCoMAcceleration = new DoubleYoVariable("alphaSimCoMAcceleration", registry);
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
   public YoVariableRegistry getYoVariableRegistry()
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
