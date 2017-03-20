package us.ihmc.exampleSimulations.flyballGovernor;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotController.RobotController;
import us.ihmc.simulationconstructionset.ExternalForcePoint;

public class FlyballGovernorSimpleClosedLoopConstraintController implements RobotController
{
   private final YoFramePoint position1A, position1B, position2A, position2B;
   private final YoFrameVector velocity1A, velocity1B, velocity2A, velocity2B;
   private final YoFrameVector force1A, force1B, force2A, force2B;

   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final DoubleYoVariable constraintGain = new DoubleYoVariable("constraintGain", registry);
   private final DoubleYoVariable constraintDamp = new DoubleYoVariable("constraintDamp", registry);
   private final DoubleYoVariable positionErrorMagnitude1 = new DoubleYoVariable("positionErrorMagnitude1", registry);
   private final DoubleYoVariable positionErrorMagnitude2 = new DoubleYoVariable("positionErrorMagnitude2", registry);
   
   // Temp variables:
   private Point3D posA = new Point3D();
   private Point3D posB = new Point3D();
   private Vector3D velA = new Vector3D();
   private Vector3D velB = new Vector3D();
   private Vector3D springForceA = new Vector3D();
   private Vector3D dampingForceA = new Vector3D();
   private Vector3D newForceA = new Vector3D();
   private Vector3D newForceB = new Vector3D();

   public FlyballGovernorSimpleClosedLoopConstraintController(FlyballGovernorRobot robot)
   {
      ExternalForcePoint constraint1A = robot.getConstraint1A();
      ExternalForcePoint constraint1B = robot.getConstraint1B();
      ExternalForcePoint constraint2A = robot.getConstraint2A();
      ExternalForcePoint constraint2B = robot.getConstraint2B();

      position1A = constraint1A.getYoPosition();
      position1B = constraint1B.getYoPosition();
      position2A = constraint2A.getYoPosition();
      position2B = constraint2B.getYoPosition();

      velocity1A = constraint1A.getYoVelocity();
      velocity1B = constraint1B.getYoVelocity();
      velocity2A = constraint2A.getYoVelocity();
      velocity2B = constraint2B.getYoVelocity();

      force1A = constraint1A.getYoForce();
      force1B = constraint1B.getYoForce();
      force2A = constraint2A.getYoForce();
      force2B = constraint2B.getYoForce();
      
      initialize();
   }

   public void initialize()
   {
      constraintGain.set(10000.0);
      constraintDamp.set(15.0);
   }

   public void doControl()
   {
      doConstraint(position1A, position1B, velocity1A, velocity1B, force1A, force1B, positionErrorMagnitude1);
      doConstraint(position2A, position2B, velocity2A, velocity2B, force2A, force2B, positionErrorMagnitude2);

   }

   private void doConstraint(YoFramePoint positionA, YoFramePoint positionB, YoFrameVector velocityA, YoFrameVector velocityB,
         YoFrameVector forceA, YoFrameVector forceB, DoubleYoVariable positionErrorMagnitude)
   {
      positionA.get(posA);
      positionB.get(posB);
      velocityA.get(velA);
      velocityB.get(velB);

      springForceA.sub(posB, posA);
      positionErrorMagnitude.set(springForceA.length());
      springForceA.scale(constraintGain.getDoubleValue());

      dampingForceA.sub(velB, velA);
      dampingForceA.scale(constraintDamp.getDoubleValue());

      newForceA.add(springForceA, dampingForceA);
      newForceB.setAndScale(-1.0, newForceA);
      
      forceA.set(newForceA);
      forceB.set(newForceB);
   }

   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   public String getDescription()
   {
      return getName();
   }

   public String getName()
   {
      return registry.getName();
   }

}
