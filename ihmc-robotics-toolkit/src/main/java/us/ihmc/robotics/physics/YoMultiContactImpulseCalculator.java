package us.ihmc.robotics.physics;

import java.util.Map;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

public class YoMultiContactImpulseCalculator extends MultiContactImpulseCalculator
{
   private final YoDouble alphaMin;
   private final YoDouble gamma;
   private final YoDouble tolerance;

   private final YoInteger maxNumberOfIterations;
   private final YoInteger iterationCounter;

   private final YoInteger numberOfCollisions;
   private final YoDouble maxUpdateMagnitude;
   private final YoInteger noConvergenceCounter;

   public YoMultiContactImpulseCalculator(int identifier, ReferenceFrame rootFrame, YoRegistry registry)
   {
      super(rootFrame);

      alphaMin = new YoDouble("alphaMin" + identifier, registry);
      gamma = new YoDouble("gamma" + identifier, registry);
      tolerance = new YoDouble("tolerance" + identifier, registry);
      maxNumberOfIterations = new YoInteger("maxNumberOfIterations" + identifier, registry);
      iterationCounter = new YoInteger("iterationCounter" + identifier, registry);

      numberOfCollisions = new YoInteger("numberOfCollisions" + identifier, registry);
      maxUpdateMagnitude = new YoDouble("maxUpdateMagnitude" + identifier, registry);

      noConvergenceCounter = new YoInteger("noConvergenceCounter" + identifier, registry);

      alphaMin.set(getAlphaMin());
      gamma.set(getGamma());
      tolerance.set(getTolerance());
      maxNumberOfIterations.set(getMaxNumberOfIterations());
   }

   @Override
   public void setAlphaMin(double alphaMin)
   {
      this.alphaMin.set(alphaMin);
   }

   @Override
   public void setGamma(double gamma)
   {
      this.gamma.set(gamma);
   }

   @Override
   public void setTolerance(double tolerance)
   {
      this.tolerance.set(tolerance);
   }

   @Override
   public void setMaxNumberOfIterations(int maxNumberOfIterations)
   {
      this.maxNumberOfIterations.set(maxNumberOfIterations);
   }

   public void clear()
   {
      numberOfCollisions.set(-1);
      iterationCounter.set(-1);
   }

   @Override
   public void configure(Map<RigidBodyBasics, PhysicsEngineRobotData> robots, MultiRobotCollisionGroup collisionGroup)
   {
      super.configure(robots, collisionGroup);

      numberOfCollisions.set(collisionGroup.getNumberOfCollisions());
   }

   @Override
   public double computeImpulses(double time, double dt, boolean verbose)
   {
      super.setAlphaMin(alphaMin.getValue());
      super.setGamma(gamma.getValue());
      super.setTolerance(tolerance.getValue());
      super.setMaxNumberOfIterations(maxNumberOfIterations.getValue());

      maxUpdateMagnitude.set(super.computeImpulses(time, dt, verbose));

      iterationCounter.set(getNumberOfIterations());

      if (iterationCounter.getValue() > maxNumberOfIterations.getValue())
         noConvergenceCounter.increment();

      return maxUpdateMagnitude.getValue();
   }
}
