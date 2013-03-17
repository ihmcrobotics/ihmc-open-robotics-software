package us.ihmc.commonWalkingControlModules.trajectories;

import static org.ejml.ops.CommonOps.solve;
import org.ejml.data.DenseMatrix64F;

import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.utilities.math.MathTools;
import us.ihmc.utilities.math.geometry.Direction;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;

import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.graphics.BagOfBalls;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;
import com.yobotics.simulationconstructionset.util.math.frames.YoFramePoint;
import com.yobotics.simulationconstructionset.util.math.frames.YoFrameVector;
import com.yobotics.simulationconstructionset.util.trajectory.DoubleProvider;
import com.yobotics.simulationconstructionset.util.trajectory.PositionProvider;
import com.yobotics.simulationconstructionset.util.trajectory.PositionTrajectoryGenerator;
import com.yobotics.simulationconstructionset.util.trajectory.VectorProvider;
import com.yobotics.simulationconstructionset.util.trajectory.YoConcatenatedSplines;
import com.yobotics.simulationconstructionset.util.trajectory.YoPolynomial;
import com.yobotics.simulationconstructionset.util.trajectory.YoPositionProvider;
import com.yobotics.simulationconstructionset.util.trajectory.YoSpline3D;

public class TwoWaypointPositionTrajectoryGenerator implements PositionTrajectoryGenerator
{
   private final static double[] DESIRED_PROPORTIONS_THROUGH_TRAJECTORY_FOR_GROUND_CLEARANCE = new double[] {1.0 / 3.0, 2.0 / 3.0};

   private final DoubleYoVariable groundClearance;

   private final String namePostFix = getClass().getSimpleName();
   private final YoVariableRegistry registry;

   private boolean VISUALIZE = true;
   private final int numberOfVisualizationMarkers = 50;

   private final BagOfBalls trajectoryBagOfBalls;
   private final BagOfBalls waypointBagOfBalls;

   private final DoubleProvider stepTimeProvider;
   private final PositionProvider[] positionSources = new PositionProvider[2];
   private final VectorProvider[] velocitySources = new VectorProvider[2];

   private final DoubleYoVariable stepTime;
   private final DoubleYoVariable timeIntoStep;

   private final YoFramePoint desiredPosition;
   private final YoFrameVector desiredVelocity;
   private final YoFrameVector desiredAcceleration;
   private final ReferenceFrame referenceFrame;
   
   private final int desiredNumberOfSplines;
   
   private final DoubleYoVariable[] arcLengthAtAccelerationEndpoints = new DoubleYoVariable[2];

   private final DoubleYoVariable[] fixedPointTimes = new DoubleYoVariable[4];
   private final YoFramePoint[] fixedPointPositions = new YoFramePoint[4];
   private final YoFrameVector[] fixedPointVelocities = new YoFrameVector[4];
   
   private double[] timesForArcLengthSplines;

   private final YoConcatenatedSplines originalConcatenatedSplines;
   private final YoConcatenatedSplines respacedConcatenatedSplines;
//   private final YoPolynomial timeSpline;

   public TwoWaypointPositionTrajectoryGenerator(String namePrefix, ReferenceFrame referenceFrame, YoVariableDoubleProvider stepTimeProvider,
           PositionProvider initialPositionProvider, VectorProvider initalVelocityProvider, YoPositionProvider finalPositionProvider,
           VectorProvider finalDesiredVelocityProvider, YoVariableRegistry parentRegistry, double groundClearance, int desiredNumberOfSplines,
           DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry)
   {
      this.registry = new YoVariableRegistry(namePrefix + namePostFix);
      parentRegistry.addChild(registry);
      trajectoryBagOfBalls = new BagOfBalls(numberOfVisualizationMarkers, 0.01, namePrefix + "TrajectoryBagOfBalls", registry,
              dynamicGraphicObjectsListRegistry);
      waypointBagOfBalls = new BagOfBalls(4, 0.02, namePrefix + "WaypointBagOfBalls", registry, dynamicGraphicObjectsListRegistry);

      this.stepTimeProvider = stepTimeProvider;

      this.referenceFrame = referenceFrame;

      this.positionSources[0] = initialPositionProvider;
      this.positionSources[1] = finalPositionProvider;

      this.velocitySources[0] = initalVelocityProvider;
      this.velocitySources[1] = finalDesiredVelocityProvider;

      this.stepTime = new DoubleYoVariable("StepTime", registry);
      this.timeIntoStep = new DoubleYoVariable("TimeIntoStep", registry);

      this.desiredPosition = new YoFramePoint(namePrefix + "DesiredPosition", referenceFrame, registry);
      this.desiredVelocity = new YoFrameVector(namePrefix + "DesiredVelocity", referenceFrame, registry);
      this.desiredAcceleration = new YoFrameVector(namePrefix + "DesiredAcceleration", referenceFrame, registry);

      this.groundClearance = new DoubleYoVariable(namePrefix + "GroundClearance", registry);
      this.groundClearance.set(groundClearance);
      
      this.desiredNumberOfSplines = desiredNumberOfSplines;
      int[] reparameterizedNumberOfCoefficientsPerPolynomial = new int[desiredNumberOfSplines];
      for (int i = 0; i < reparameterizedNumberOfCoefficientsPerPolynomial.length; i++)
      {
         reparameterizedNumberOfCoefficientsPerPolynomial[i] = 6;
      }
      
      this.respacedConcatenatedSplines = new YoConcatenatedSplines(reparameterizedNumberOfCoefficientsPerPolynomial, referenceFrame, 2, registry, namePrefix + "RespacedConcatenatedSplines");
      this.timesForArcLengthSplines = new double[desiredNumberOfSplines + 1];

      for (int i = 0; i < 4; i++)
      {
         fixedPointTimes[i] = new DoubleYoVariable(namePrefix + "FixedPointTime" + i, registry);
         fixedPointPositions[i] = new YoFramePoint(namePrefix + "FixedPointPosition" + i, referenceFrame, registry);
         fixedPointVelocities[i] = new YoFrameVector(namePrefix + "FixedPointVelocity" + i, referenceFrame, registry);
      }
      
//      for (int i = 0; i < 2; i++)
//      {
//    	  arcLengthAtAccelerationEndpoints[i] = new DoubleYoVariable(namePrefix + "ArcLengthAtAccelerationEndpoints", registry);
//      }

      this.originalConcatenatedSplines = new YoConcatenatedSplines(new int[] {4, 6, 4}, referenceFrame, 2, registry, namePrefix + "OriginalConcatenatedSplines");
//      this.timeSpline = new YoPolynomial(namePrefix + "timeSpline", 4, registry);
   }

   public void compute(double time)
   {
//	   double percentStepComplete = time / stepTime.getDoubleValue();

//	   timeSpline.compute(inputTime);
//	   double time = timeSpline.getPosition();
	   
      timeIntoStep.set(time);

      double totalTime = stepTime.getDoubleValue();
      if (time > totalTime)
         time = totalTime;

      respacedConcatenatedSplines.compute(time);
      desiredPosition.set(respacedConcatenatedSplines.getPosition());
      desiredVelocity.set(respacedConcatenatedSplines.getVelocity());
      desiredAcceleration.set(respacedConcatenatedSplines.getAcceleration());
   }

   public void get(FramePoint positionToPack)
   {
      desiredPosition.getFramePointAndChangeFrameOfPackedPoint(positionToPack);
   }

   public void packVelocity(FrameVector velocityToPack)
   {
      desiredVelocity.getFrameVectorAndChangeFrameOfPackedVector(velocityToPack);
   }

   public void packAcceleration(FrameVector accelerationToPack)
   {
      desiredAcceleration.getFrameVectorAndChangeFrameOfPackedVector(accelerationToPack);
   }

   private void setStepTime()
   {
      double stepTime = stepTimeProvider.getValue();
      MathTools.checkIfInRange(stepTime, 0.0, Double.POSITIVE_INFINITY);
      this.stepTime.set(stepTime);
      timeIntoStep.set(0.0);
   }

   public void initialize()
   {
      initialize(null);
   }

   public void initialize(FramePoint[] waypoints)
   {
	   double t0 = System.currentTimeMillis();
      setStepTime();
      setInitialAndFinalPositionsAndVelocities();
      setWaypointPositionsAndVelocities(waypoints);

      setConcatenatedSplines();
      double t1 = System.currentTimeMillis();
      
      respaceSplineRangesWithEqualArcLengths(originalConcatenatedSplines, respacedConcatenatedSplines);
      double t2 = System.currentTimeMillis();
      
      double originalSplineTime = t1 - t0;
      double respacedSplineTime = t2 - t1;
      
      System.out.println("originalSplineTime : " + originalSplineTime);
      System.out.println("respacedSplineTime : " + respacedSplineTime);
      
//      setTimeSpline();

      if (VISUALIZE)
      {
         visualizeSpline();
      }
   }

   private void setInitialAndFinalPositionsAndVelocities()
   {
      int[] sourceIndicies = new int[] {0, 1};
      int[] fixedPointIndicies = new int[] {0, 3};
      FramePoint tempPositions = new FramePoint(referenceFrame);
      FrameVector tempVelocities = new FrameVector(referenceFrame);
      for (int i = 0; i < 2; i++)
      {
         positionSources[sourceIndicies[i]].get(tempPositions);
         velocitySources[sourceIndicies[i]].get(tempVelocities);
         tempPositions.changeFrame(referenceFrame);
         tempVelocities.changeFrame(referenceFrame);
         fixedPointPositions[fixedPointIndicies[i]].set(tempPositions);
         fixedPointVelocities[fixedPointIndicies[i]].set(tempVelocities);
      }
   }

   private void setWaypointPositionsAndVelocities(FramePoint[] waypoints)
   {
      setWaypointPositions(waypoints);

      setFixedPointTimesAndWaypointVelocities();
   }

   private void setWaypointPositions(FramePoint[] waypoints)
   {
      if (waypoints == null)
      {
         FramePoint initialPosition = fixedPointPositions[0].getFramePointCopy();
         FramePoint finalPosition = fixedPointPositions[3].getFramePointCopy();
         waypoints = new FramePoint[2];
         waypoints[0] = new FramePoint(referenceFrame);
         waypoints[1] = new FramePoint(referenceFrame);
         positionSources[0].get(initialPosition);
         positionSources[1].get(finalPosition);
         initialPosition.changeFrame(referenceFrame);
         finalPosition.changeFrame(referenceFrame);

         for (int i = 0; i < 2; i++)
         {
            waypoints[i].set(finalPosition);
            waypoints[i].sub(initialPosition);
            waypoints[i].scale(DESIRED_PROPORTIONS_THROUGH_TRAJECTORY_FOR_GROUND_CLEARANCE[i]);
            waypoints[i].add(initialPosition);
            waypoints[i].setZ(waypoints[i].getZ() + groundClearance.getDoubleValue());
         }

         fixedPointPositions[1].set(waypoints[0]);
         fixedPointPositions[2].set(waypoints[1]);
      }
      else if (waypoints.length == 2)
      {
         waypoints[0].changeFrame(referenceFrame);
         waypoints[1].changeFrame(referenceFrame);
      }
      else
      {
         throw new RuntimeException("TwoWaypointPositionTrajectoryGenerator only supports trajectory generation for two waypoints.");
      }

      fixedPointPositions[1].set(waypoints[0]);
      fixedPointPositions[2].set(waypoints[1]);
   }

   private void setFixedPointTimesAndWaypointVelocities()
   {
      int timeVelocityIterations = 1;

      double[] distances = new double[4];

      for (int i = 0; i < fixedPointPositions.length - 1; i++)
      {
         distances[i] = fixedPointPositions[i].distance(fixedPointPositions[i + 1]);
      }

      for (int i = 0; i < 4; i++)
      {
         fixedPointTimes[i].set(stepTime.getDoubleValue() * ((double) i / 3.0));
      }

      for (int i = 0; i < timeVelocityIterations; i++)
      {
         setWaypointVelocities();
         getFixedPointTimes(distances);
      }
   }

   private void setConcatenatedSplines()
   {
      double[] times = new double[4];
      FramePoint[] positions = new FramePoint[4];
      FrameVector[] velocities = new FrameVector[4];

      for (int i = 0; i < 4; i++)
      {
         times[i] = fixedPointTimes[i].getDoubleValue();
         positions[i] = fixedPointPositions[i].getFramePointCopy();
         velocities[i] = fixedPointVelocities[i].getFrameVectorCopy();
      }

      originalConcatenatedSplines.setCubicQuinticCubic(times, positions, velocities);
   }

   private void visualizeSpline()
   {
	   
//      double ds = concatenatedSplines.getArcLength() / numberOfVisualizationMarkers;
       
      for (int i = 0; i < numberOfVisualizationMarkers; i++)
      {
         double t0 = originalConcatenatedSplines.getT0();
         double tf = originalConcatenatedSplines.getTf();
         double t = t0 + (double) i / (double) (numberOfVisualizationMarkers) * (tf - t0);
         compute(t);
         trajectoryBagOfBalls.setBall(desiredPosition.getFramePointCopy(), i);
    	  
//          double t = concatenatedSplines.approximateTimeFromArcLength((((double) i)*ds));
//          compute(t);
//          trajectoryBagOfBalls.setBall(desiredPosition.getFramePointCopy(), i);
      }

      for (int i = 0; i < fixedPointPositions.length; i++)
      {
         waypointBagOfBalls.setBall(fixedPointPositions[i].getFramePointCopy(), YoAppearance.AliceBlue(), i);
      }
   }

   private void setWaypointVelocities()
   {
      fixedPointVelocities[1].set(getWaypointVelocity(0, 1));
      fixedPointVelocities[2].set(getWaypointVelocity(3, 2));
   }

   private FrameVector getWaypointVelocity(int indexInitialOrFinal, int indexOfWaypoint)
   {
      FrameVector waypointVelocity = new FrameVector(referenceFrame);
      DenseMatrix64F constraintsMatrix = new DenseMatrix64F(3, 3);
      DenseMatrix64F constraintsVector = new DenseMatrix64F(3, 1);
      DenseMatrix64F coefficientsVector = new DenseMatrix64F(3, 1);

      double footstepTime = fixedPointTimes[indexInitialOrFinal].getDoubleValue();
      double wayPointTime = fixedPointTimes[indexOfWaypoint].getDoubleValue();
      constraintsMatrix.setData(new double[]
      {
         footstepTime * footstepTime, footstepTime, 1, wayPointTime * wayPointTime, wayPointTime, 1, 2 * footstepTime, 1, 0
      });

      for (Direction d : Direction.values())
      {
         constraintsVector.setData(new double[] {fixedPointPositions[indexInitialOrFinal].get(d), fixedPointPositions[indexOfWaypoint].get(d),
                 fixedPointVelocities[indexInitialOrFinal].get(d)});
         solve(constraintsMatrix, constraintsVector, coefficientsVector);
         double velocity = 2 * coefficientsVector.get(0) * wayPointTime + coefficientsVector.get(1);
         waypointVelocity.set(d, velocity);
      }

      return waypointVelocity;
   }

   private void getFixedPointTimes(double[] distances)
   {
      for (int i = 1; i < fixedPointTimes.length; i++)
      {
         fixedPointTimes[i].set(fixedPointTimes[i - 1].getDoubleValue()
                                + distances[i - 1] / (fixedPointVelocities[i - 1].length() + fixedPointVelocities[i].length()) / 2);
      }

      double totalTime = fixedPointTimes[fixedPointTimes.length - 1].getDoubleValue();
      double scaleFactor = stepTime.getDoubleValue() / totalTime;

      for (int i = 0; i < fixedPointTimes.length; i++)
      {
         fixedPointTimes[i].mul(scaleFactor);
      }
   }

   public void respaceSplineRangesWithEqualArcLengths(YoConcatenatedSplines oldSplines, YoConcatenatedSplines newSplines)
   {
      int desiredNumberOfSplines = newSplines.getNumberOfSplines();

      double[] oldTimes = new double[desiredNumberOfSplines + 1];
      double[] newTimes = new double[desiredNumberOfSplines + 1];

      double t0 = oldSplines.getT0();
      double tf = oldSplines.getTf();
      double totalTime = tf - t0;

      double totalArcLength = oldSplines.getArcLength();
      double desiredIndividualArcLength = totalArcLength / ((double) desiredNumberOfSplines);
      oldTimes[0] = t0;
      newTimes[0] = t0;

      for (int i = 1; i < oldTimes.length - 1; i++)
      {
         oldTimes[i] = oldSplines.approximateTimeFromArcLength((double) i * desiredIndividualArcLength);
         newTimes[i] = t0 + totalTime * ((double) i) / ((double) desiredNumberOfSplines);
      }

      oldTimes[oldTimes.length - 1] = tf;
      newTimes[newTimes.length - 1] = tf;

      newSplines.setQuintics(oldSplines, oldTimes, newTimes);
   }
   
   public boolean isDone()
   {
      return timeIntoStep.getDoubleValue() >= stepTime.getDoubleValue();
   }

   private void setTimeSpline()
   {
	   double[] accelerationEndPointTimes = new double[2];
	   accelerationEndPointTimes[0] = 0.1 * stepTime.getDoubleValue();
	   accelerationEndPointTimes[1] = 0.9 * stepTime.getDoubleValue();
	   
	   for(int i = 0; i < 2; i++)
	   {
		 arcLengthAtAccelerationEndpoints[i].set(originalConcatenatedSplines.getArcLength(0.0, accelerationEndPointTimes[i]));
	   }
   }
}
