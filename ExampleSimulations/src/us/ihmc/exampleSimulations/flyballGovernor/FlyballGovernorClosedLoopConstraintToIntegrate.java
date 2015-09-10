package us.ihmc.exampleSimulations.flyballGovernor;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.FunctionToIntegrate;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.yoUtilities.math.frames.YoFramePoint;
import us.ihmc.yoUtilities.math.frames.YoFrameVector;

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

   private Point3d posA = new Point3d();
   private Point3d posB = new Point3d();
   private Vector3d velA = new Vector3d();
   private Vector3d velB = new Vector3d();
   private Vector3d springForceA = new Vector3d();
   private Vector3d dampingForceA = new Vector3d();
   private Vector3d newForceA = new Vector3d();
   private Vector3d newForceB = new Vector3d();

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
      newForceB.scale(-1.0, newForceA);

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
