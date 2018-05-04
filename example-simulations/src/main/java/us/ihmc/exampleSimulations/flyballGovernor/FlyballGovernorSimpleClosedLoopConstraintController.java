package us.ihmc.exampleSimulations.flyballGovernor;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoFramePoint3D;
import us.ihmc.yoVariables.variable.YoFrameVector3D;
import us.ihmc.robotics.robotController.RobotController;
import us.ihmc.simulationconstructionset.ExternalForcePoint;

public class FlyballGovernorSimpleClosedLoopConstraintController implements RobotController
{
   private final YoFramePoint3D position1A, position1B, position2A, position2B;
   private final YoFrameVector3D velocity1A, velocity1B, velocity2A, velocity2B;
   private final YoFrameVector3D force1A, force1B, force2A, force2B;

   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final YoDouble constraintGain = new YoDouble("constraintGain", registry);
   private final YoDouble constraintDamp = new YoDouble("constraintDamp", registry);
   private final YoDouble positionErrorMagnitude1 = new YoDouble("positionErrorMagnitude1", registry);
   private final YoDouble positionErrorMagnitude2 = new YoDouble("positionErrorMagnitude2", registry);
   
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

   private void doConstraint(YoFramePoint3D positionA, YoFramePoint3D positionB, YoFrameVector3D velocityA, YoFrameVector3D velocityB,
         YoFrameVector3D forceA, YoFrameVector3D forceB, YoDouble positionErrorMagnitude)
   {
      posA.set(positionA);
      posB.set(positionB);
      velA.set(velocityA);
      velB.set(velocityB);

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
