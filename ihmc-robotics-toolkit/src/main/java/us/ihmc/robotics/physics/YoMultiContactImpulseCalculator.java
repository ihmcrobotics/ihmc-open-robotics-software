package us.ihmc.robotics.physics;

import java.util.Map;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
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

   public YoMultiContactImpulseCalculator(int identifier, ReferenceFrame rootFrame, YoVariableRegistry registry)
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
   public void configure(Map<RigidBodyBasics, PhysicsEngineRobotData> robots, MultiRobotCollisionGroup collisionGroup)
   {
      super.configure(robots, collisionGroup);

      numberOfCollisions.set(collisionGroup.getNumberOfCollisions());
   }

   @Override
   public double computeImpulses(double time, double dt, boolean verbose)
   {
      setAlphaMin(alphaMin.getValue());
      setGamma(gamma.getValue());
      setTolerance(tolerance.getValue());

      setMaxNumberOfIterations(maxNumberOfIterations.getValue());

      maxUpdateMagnitude.set(super.computeImpulses(time, dt, verbose));

      iterationCounter.set(getNumberOfIterations());

      if (iterationCounter.getValue() > maxNumberOfIterations.getValue())
         noConvergenceCounter.increment();

      return maxUpdateMagnitude.getValue();
   }
}
