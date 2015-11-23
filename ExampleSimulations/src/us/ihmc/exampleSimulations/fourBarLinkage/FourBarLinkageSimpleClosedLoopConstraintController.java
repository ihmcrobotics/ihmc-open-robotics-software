package us.ihmc.exampleSimulations.fourBarLinkage;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.robotController.RobotController;

public class FourBarLinkageSimpleClosedLoopConstraintController implements RobotController
{
   private final YoFramePoint yoPositionA, yoPositionB;
   private final YoFrameVector yoVelocityA, yoVelocityB;
   private final YoFrameVector yoForceA, yoForceB;
   
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final DoubleYoVariable constraintGain = new DoubleYoVariable("constraintGain", registry);
   private final DoubleYoVariable constraintDamping = new DoubleYoVariable("constraintDamping", registry);
   private final DoubleYoVariable positionErrorMagnitude = new DoubleYoVariable("positionErrorMagnitude", registry);

   // Temp variables:
   private Point3d posA = new Point3d();
   private Point3d posB = new Point3d();
   private Vector3d velA = new Vector3d();
   private Vector3d velB = new Vector3d();
   private Vector3d springForceA = new Vector3d();
   private Vector3d dampingForceA = new Vector3d();
   private Vector3d newForceA = new Vector3d();
   private Vector3d newForceB = new Vector3d();
   
   public FourBarLinkageSimpleClosedLoopConstraintController(FourBarLinkageRobot robot)
   {
      ExternalForcePoint efpA = robot.getEfpJoint1to2();
      ExternalForcePoint efpB = robot.getEfpJoint1to4();
      
      yoPositionA = efpA.getYoPosition();
      yoPositionB = efpB.getYoPosition();
      
      yoVelocityA = efpA.getYoVelocity();
      yoVelocityB = efpB.getYoVelocity();
      
      yoForceA = efpA.getYoForce();
      yoForceB = efpB.getYoForce();
      
      initialize();
   }
   
   @Override
   public void initialize()
   {
      constraintGain.set(1000.0);
      constraintDamping.set(10.0);
   }

   @Override
   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   @Override
   public String getName()
   {
      return registry.getName();
   }

   @Override
   public String getDescription()
   {
      return getName();
   }

   @Override
   public void doControl()
   {
      yoPositionA.get(posA);
      yoPositionB.get(posB);
      
      yoVelocityA.get(velA);
      yoVelocityB.get(velB);
      
      springForceA.sub(posB, posA);
      positionErrorMagnitude.set(springForceA.length());
      springForceA.scale(constraintGain.getDoubleValue());

      dampingForceA.sub(velB, velA);
      dampingForceA.scale(constraintDamping.getDoubleValue());

      newForceA.add(springForceA, dampingForceA);
      newForceB.scale(-1.0, newForceA);
      
      yoForceA.set(newForceA);
      yoForceB.set(newForceB);
   }

}
