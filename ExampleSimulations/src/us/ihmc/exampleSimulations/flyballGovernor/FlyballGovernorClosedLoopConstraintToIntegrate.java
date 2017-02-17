package us.ihmc.exampleSimulations.flyballGovernor;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.FunctionToIntegrate;
import us.ihmc.simulationconstructionset.Robot;

public class FlyballGovernorClosedLoopConstraintToIntegrate implements FunctionToIntegrate
{
   private final YoVariableRegistry registry;

   private final YoFramePoint positionA, positionB;
   private final YoFrameVector velocityA, velocityB;
   private final YoFrameVector forceA, forceB;

   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final DoubleYoVariable positionErrorMagnitude;

   private final double k = 10000.0;
   private final double b = 15.0;

   private Point3D posA = new Point3D();
   private Point3D posB = new Point3D();
   private Vector3D velA = new Vector3D();
   private Vector3D velB = new Vector3D();
   private Vector3D springForceA = new Vector3D();
   private Vector3D dampingForceA = new Vector3D();
   private Vector3D newForceA = new Vector3D();
   private Vector3D newForceB = new Vector3D();

   public FlyballGovernorClosedLoopConstraintToIntegrate(String name, ExternalForcePoint constraintA, ExternalForcePoint constraintB, Robot robot,
         YoVariableRegistry parentRegistry)
   {
      positionA = constraintA.getYoPosition();
      positionB = constraintB.getYoPosition();

      velocityA = constraintA.getYoVelocity();
      velocityB = constraintB.getYoVelocity();

      forceA = constraintA.getYoForce();
      forceB = constraintB.getYoForce();

      robot.addFunctionToIntegrate(this);

      registry = new YoVariableRegistry(name);
      positionErrorMagnitude = new DoubleYoVariable("positionErrorMagnitude", registry);
      parentRegistry.addChild(registry);
   }

   public void doConstraint()
   {
      positionA.get(posA);
      positionB.get(posB);
      velocityA.get(velA);
      velocityB.get(velB);

      springForceA.sub(posB, posA);
      positionErrorMagnitude.set(springForceA.length());
      springForceA.scale(k);
      
      dampingForceA.sub(velB, velA);
      dampingForceA.scale(b);
      newForceA.add(springForceA, dampingForceA);
      newForceB.setAndScale(-1.0, newForceA);

      forceA.set(newForceA);
      forceB.set(newForceB);
   }

   public double[] computeDerivativeVector()
   {
      doConstraint();

      // Return 0.0 for the force rate. It doesn't change anything to compute some stuff to return.
      return null;
   }

   public int getVectorSize()
   {
      return 0;
   }

   public DoubleYoVariable[] getOutputVariables()
   {
      return null;
   }

}
