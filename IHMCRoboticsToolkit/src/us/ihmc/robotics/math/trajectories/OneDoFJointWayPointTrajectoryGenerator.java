package us.ihmc.robotics.math.trajectories;

import java.util.ArrayList;

import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.IntegerYoVariable;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.trajectories.providers.DoubleProvider;

public class OneDoFJointWayPointTrajectoryGenerator implements OneDoFJointTrajectoryGenerator
{
   private final YoVariableRegistry registry;
   private final DoubleYoVariable finalPosition;
   private final ArrayList<DoubleYoVariable> intermediatePositions;
   private final ArrayList<YoPolynomial> connectingPolynomials;
   private final YoPolynomial startMotionPolynomial, finalizeMotionPolynomial;
   private final DoubleYoVariable trajectoryTime;
   private final DoubleYoVariable subTrajectoryTime;
   private final DoubleProvider trajectoryTimeProvider;
   private final DoubleYoVariable currentTime;
   private final OneDoFJoint joint;
   private final DoubleYoVariable currentTimeOffset;
   private final IntegerYoVariable currentPolynomialIndex;
   private final IntegerYoVariable currentNumberOfWaypoints;

   public OneDoFJointWayPointTrajectoryGenerator(String namePrefix, OneDoFJoint joint, DoubleProvider trajectoryTimeProvider, int maxNumberOfWayPoints,
         YoVariableRegistry parentRegistry)
   {
      registry = new YoVariableRegistry(namePrefix + getClass().getSimpleName());
      this.joint = joint;
      trajectoryTime = new DoubleYoVariable(namePrefix + "TrajectoryTime", registry);
      subTrajectoryTime = new DoubleYoVariable(namePrefix + "SubTrajectoryTime", registry);
      currentTime = new DoubleYoVariable(namePrefix + "CurrentTime", registry);
      this.trajectoryTimeProvider = trajectoryTimeProvider;
      finalPosition = new DoubleYoVariable(namePrefix + "FinalPosition", registry);
      intermediatePositions = new ArrayList<>(maxNumberOfWayPoints);

      startMotionPolynomial = new YoPolynomial(namePrefix + "StartMotionPolynomial", 4, registry);
      finalizeMotionPolynomial = new YoPolynomial(namePrefix + "FinalizeMotionPolynomial", 4, registry);

      currentTimeOffset = new DoubleYoVariable(namePrefix + "CurrentTimeOffset", registry);
      currentNumberOfWaypoints = new IntegerYoVariable(namePrefix + "CurrentNumberOfWayPoints", registry);
      currentPolynomialIndex = new IntegerYoVariable(namePrefix + "CurrentPolynomialIndex", registry);
      connectingPolynomials = new ArrayList<>(maxNumberOfWayPoints - 2);
      for (int i = 0; i < maxNumberOfWayPoints - 2; i++)
      {
         YoPolynomial connectingPolynomial = new YoPolynomial(namePrefix + "ConnectingPolynomial" + Integer.toString(i), 2, registry);
         connectingPolynomials.add(connectingPolynomial);
      }

      for (int i = 0; i < maxNumberOfWayPoints; i++)
      {
         DoubleYoVariable intermediatePosition = new DoubleYoVariable(namePrefix + "IntermediatePosition" + Integer.toString(i), registry);
         intermediatePosition.set(Double.NaN);
         intermediatePositions.add(intermediatePosition);
      }

      parentRegistry.addChild(registry);
   }

   /**
    * Desired joint angles and velocities come from reading the joints, this method can override them those position and velocity values. 
    * @param currentDesiredPosition Sets the desired joint position.
    * @param currentDesiredVelocity Sets the desired joint velocity.
    */
   @Override
   public void initialize(double initialPosition, double initialVelocity)
   {
      currentTime.set(0.0);
      currentTimeOffset.set(0.0);
      currentPolynomialIndex.set(0);

      trajectoryTime.set(trajectoryTimeProvider.getValue());
      subTrajectoryTime.set(trajectoryTime.getDoubleValue() / (currentNumberOfWaypoints.getIntegerValue() + 1));
      startMotionPolynomial.setCubicThreeInitialConditionsFinalPosition(0.0, subTrajectoryTime.getDoubleValue(), initialPosition, initialVelocity, 0.0,
            intermediatePositions.get(0).getDoubleValue());

      startMotionPolynomial.compute(subTrajectoryTime.getDoubleValue());

      for (int i = 0; i < currentNumberOfWaypoints.getIntegerValue() - 1; i++)
      {
         YoPolynomial connectingPolynomial = connectingPolynomials.get(i);
         double z0 = intermediatePositions.get(i).getDoubleValue();
         double zFinal = intermediatePositions.get(i + 1).getDoubleValue();
         connectingPolynomial.setLinear(0.0, subTrajectoryTime.getDoubleValue(), z0, zFinal);
         connectingPolynomial.compute(subTrajectoryTime.getDoubleValue());
      }

      double z0 = intermediatePositions.get(currentNumberOfWaypoints.getIntegerValue() - 1).getDoubleValue();
      double zFinal = finalPosition.getDoubleValue();
      finalizeMotionPolynomial.setCubicInitialPositionThreeFinalConditions(0.0, subTrajectoryTime.getDoubleValue(), z0, zFinal, 0.0, 0.0);
   }

   @Override
   public void initialize()
   {
      initialize(joint.getQ(), joint.getQd());
   }

   @Override
   public void compute(double time)
   {
      currentTime.set(time);
      time = MathTools.clipToMinMax(time, 0.0, trajectoryTime.getDoubleValue());

      if (currentTime.getDoubleValue() - currentTimeOffset.getDoubleValue() > subTrajectoryTime.getDoubleValue())
      {
         currentTimeOffset.add(subTrajectoryTime.getDoubleValue());
         currentPolynomialIndex.increment();
         currentPolynomialIndex.set(Math.min(currentPolynomialIndex.getIntegerValue(), currentNumberOfWaypoints.getIntegerValue()));
      }

      time -= currentTimeOffset.getDoubleValue();
      time = MathTools.clipToMinMax(time, 0.0, subTrajectoryTime.getDoubleValue());
      
      findCurrentPolynomial().compute(time);
   }

   private YoPolynomial findCurrentPolynomial()
   {
      YoPolynomial currentPolynomial;

      if (currentPolynomialIndex.getIntegerValue() == 0)
         currentPolynomial = startMotionPolynomial;
      else if (currentPolynomialIndex.getIntegerValue() == currentNumberOfWaypoints.getIntegerValue())
         currentPolynomial = finalizeMotionPolynomial;
      else
         currentPolynomial = connectingPolynomials.get(currentPolynomialIndex.getIntegerValue() - 1);
      
      return currentPolynomial;
   }

   @Override
   public boolean isDone()
   {
      return currentTime.getDoubleValue() > trajectoryTime.getDoubleValue();
   }

   @Override
   public double getValue()
   {
      return isDone() ? finalPosition.getDoubleValue() : findCurrentPolynomial().getPosition();
   }

   @Override
   public double getVelocity()
   {
      return isDone() ? 0.0 : findCurrentPolynomial().getVelocity();
   }

   @Override
   public double getAcceleration()
   {
      return isDone() ? 0.0 : findCurrentPolynomial().getAcceleration();
   }

   public void setDesiredPositions(double[] desiredPositions)
   {
      if (desiredPositions.length > intermediatePositions.size() + 1)
         throw new RuntimeException("Too many waypoints!");

      if (desiredPositions.length == 1)
         throw new RuntimeException("No waypoint! Use OneDoFJointQuinticTrajectoryGenerator instead.");

      currentNumberOfWaypoints.set(desiredPositions.length - 1);

      for (int i = 0; i < currentNumberOfWaypoints.getIntegerValue(); i++)
      {
         intermediatePositions.get(i).set(desiredPositions[i]);
      }

      finalPosition.set(desiredPositions[currentNumberOfWaypoints.getIntegerValue()]);
   }
}