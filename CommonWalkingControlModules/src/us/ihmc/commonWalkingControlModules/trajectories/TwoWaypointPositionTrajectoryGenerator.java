package us.ihmc.commonWalkingControlModules.trajectories;

import static org.ejml.ops.CommonOps.solve;

import javax.vecmath.Point3d;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.utilities.math.MathTools;
import us.ihmc.utilities.math.geometry.Direction;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.math.geometry.Sphere3d;

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
import com.yobotics.simulationconstructionset.util.trajectory.YoPositionProvider;
import com.yobotics.simulationconstructionset.util.trajectory.YoSpline3D;

public class TwoWaypointPositionTrajectoryGenerator implements PositionTrajectoryGenerator
{
   private final static double[] DESIRED_PROPORTIONS_THROUGH_TRAJECTORY_FOR_GROUND_CLEARANCE = new double[] {1.0 / 5.0, 4.0 / 5.0};
   private final static double FRACTION_OF_TIME_TO_OR_FROM_WAYPOINT_FOR_SPEED_UP_OR_SLOW_DOWN = .6;
   private final static double SPHERE_EDGE_TO_WAYPOINT_DISTANCE = 0.1;

   private final double groundClearance;

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

   private DoubleYoVariable[] fixedPointTimes = new DoubleYoVariable[4];
   private YoFramePoint[] fixedPointPositions = new YoFramePoint[4];
   private YoFrameVector[] fixedPointVelocities = new YoFrameVector[4];

   private DoubleYoVariable[] accelerationEndpointTimes = new DoubleYoVariable[2];
   private YoFrameVector[] accelerationEndpointVelocities = new YoFrameVector[2];
   
   private final double[] arcLengthsOfAccelerationPeriods = new double[2];
   
   private final DenseMatrix64F constraintsMatrix = new DenseMatrix64F(6, 6);
   private final DenseMatrix64F constraintsVector = new DenseMatrix64F(6, 1);
   private final DenseMatrix64F coefficientsVector = new DenseMatrix64F(6, 1);
   
   private final YoSpline3D accelerationPeriod;

   private final YoConcatenatedSplines concatenatedSplinesWithArcLengthApproximatedByDistance;
   private final YoConcatenatedSplines concatenatedSplinesWithArcLengthCalculatedIteratively;

   public TwoWaypointPositionTrajectoryGenerator(String namePrefix, ReferenceFrame referenceFrame, YoVariableDoubleProvider stepTimeProvider,
           PositionProvider initialPositionProvider, VectorProvider initalVelocityProvider, YoPositionProvider finalPositionProvider,
           VectorProvider finalDesiredVelocityProvider, YoVariableRegistry parentRegistry, double groundClearance,
           int arcLengthCalculatorDivisionsPerPolynomial, DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry)
   {
      registry = new YoVariableRegistry(namePrefix + namePostFix);
      parentRegistry.addChild(registry);
      trajectoryBagOfBalls = new BagOfBalls(numberOfVisualizationMarkers, 0.01, namePrefix + "TrajectoryBagOfBalls", registry,
              dynamicGraphicObjectsListRegistry);
      waypointBagOfBalls = new BagOfBalls(4, 0.02, namePrefix + "WaypointBagOfBalls", registry, dynamicGraphicObjectsListRegistry);

      this.stepTimeProvider = stepTimeProvider;

      this.referenceFrame = referenceFrame;

      positionSources[0] = initialPositionProvider;
      positionSources[1] = finalPositionProvider;

      velocitySources[0] = initalVelocityProvider;
      velocitySources[1] = finalDesiredVelocityProvider;

      stepTime = new DoubleYoVariable("stepTime", registry);
      timeIntoStep = new DoubleYoVariable("timeIntoStep", registry);

      desiredPosition = new YoFramePoint(namePrefix + "DesiredPosition", referenceFrame, registry);
      desiredVelocity = new YoFrameVector(namePrefix + "DesiredVelocity", referenceFrame, registry);
      desiredAcceleration = new YoFrameVector(namePrefix + "DesiredAcceleration", referenceFrame, registry);

      this.groundClearance = groundClearance;

      for (int i = 0; i < 4; i++)
      {
         fixedPointTimes[i] = new DoubleYoVariable(namePrefix + "FixedPointTime" + i, registry);
         fixedPointPositions[i] = new YoFramePoint(namePrefix + "FixedPointPosition" + i, referenceFrame, registry);
         fixedPointVelocities[i] = new YoFrameVector(namePrefix + "FixedPointVelocity" + i, referenceFrame, registry);
      }

      for (int i = 0; i < 2; i++)
      {
         accelerationEndpointTimes[i] = new DoubleYoVariable(namePrefix + "AccelerationEndpointTimes" + i, registry);
         accelerationEndpointVelocities[i] = new YoFrameVector(namePrefix + "AccelerationEndpointVelocities" + i, referenceFrame, registry);
      }
      
      accelerationPeriod = new YoSpline3D(4, 20, referenceFrame, registry, "accelerationPeriod");

      concatenatedSplinesWithArcLengthApproximatedByDistance = new YoConcatenatedSplines(new int[] {6, 6, 6}, referenceFrame,
              arcLengthCalculatorDivisionsPerPolynomial, registry, namePrefix + "ConcatenatedSplinesWithArcLengthApproximatedByDistance");
      concatenatedSplinesWithArcLengthCalculatedIteratively = new YoConcatenatedSplines(new int[] {6, 6, 6}, referenceFrame, 2, registry,
              namePrefix + "ConcatenatedSplinesWithArcLengthCalculatedIteratively");
   }
   
   public void compute(double time)
   {
	   compute(time, true);
   }

   public void compute(double time, boolean printtime)
   {
      if(printtime) System.out.println(time);
      
      timeIntoStep.set(time);

      double totalTime = stepTime.getDoubleValue();
      if (time > totalTime)
         time = totalTime;

      concatenatedSplinesWithArcLengthCalculatedIteratively.compute(time);
      desiredPosition.set(concatenatedSplinesWithArcLengthCalculatedIteratively.getPosition());
      desiredVelocity.set(concatenatedSplinesWithArcLengthCalculatedIteratively.getVelocity());
      desiredAcceleration.set(concatenatedSplinesWithArcLengthCalculatedIteratively.getAcceleration());
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
      setStepTime();

      setInitialAndFinalPositionsAndVelocities();
      setWaypointPositions(waypoints);

      double[] arcLengths;

      arcLengths = getArcLengthsApproximatedByDistance();
      setFixedPointTimesAndWaypointVelocities(arcLengths);
      setConcatenatedSplines(concatenatedSplinesWithArcLengthApproximatedByDistance);

      arcLengths = getArcLengthsCalculatedIteratively();
      setFixedPointTimesAndWaypointVelocities(arcLengths);
      setConcatenatedSplines(concatenatedSplinesWithArcLengthCalculatedIteratively);

      if (VISUALIZE)
      {
         visualizeSpline();
      }
   }
   
   public void initializeUsingSphere(Sphere3d sphere3d)
   {
	   FramePoint[] waypoints = getWaypointsForSphere(sphere3d);
	   initialize(waypoints);
   }
   
   private FramePoint[] getWaypointsForSphere(Sphere3d sphere3d)
   {
	   FramePoint[] wayPoints = new FramePoint[2];
	   Point3d sphereCenterPoint3d = new Point3d();
	   sphere3d.getCenter(sphereCenterPoint3d);
	   FramePoint sphereCenterFramePoint = new FramePoint(referenceFrame, sphereCenterPoint3d.getX(), sphereCenterPoint3d.getY(), sphereCenterPoint3d.getZ());
	   double radius = sphere3d.getRadius();
	   boolean areDefaultWaypointsOutsideSphere = true;
	   
	   FramePoint tempPositions = new FramePoint(referenceFrame);
	   for (int i = 0; i < 2; i++)
	   {
	      positionSources[i].get(tempPositions);
	      tempPositions.changeFrame(referenceFrame);
	      fixedPointPositions[3*i].set(tempPositions);
	   }
	   
	   setWaypointsToDefault();
	   double[] sphereWaypointDistances = new double[2];
	   for(int i = 0; i < 2; i++)
	   {
		   sphereWaypointDistances[i] = wayPoints[i].distance(sphereCenterFramePoint);
		   wayPoints[i].set(fixedPointPositions[i].getX(), fixedPointPositions[i].getY(), fixedPointPositions[i].getZ());
		   areDefaultWaypointsOutsideSphere = areDefaultWaypointsOutsideSphere && (sphereWaypointDistances[i] < radius);
	   }
	   
	   if(!areDefaultWaypointsOutsideSphere)
	   {
		   FrameVector waypointToSphereSurface = new FrameVector(referenceFrame);
		   for(int i = 0; i < 2; i++)
		   {
			   waypointToSphereSurface.sub(wayPoints[i], sphereCenterFramePoint);
			   waypointToSphereSurface.normalize();
			   double distanceToMoveWaypoint = radius - sphereWaypointDistances[i] + SPHERE_EDGE_TO_WAYPOINT_DISTANCE;
			   waypointToSphereSurface.scale(distanceToMoveWaypoint);
			   wayPoints[i].add(waypointToSphereSurface);
		   }
	   }
	   
	   return wayPoints;
   }

   private double[] getArcLengthsApproximatedByDistance()
   {
      double[] arcLengths = new double[4];

      for (int i = 0; i < fixedPointPositions.length - 1; i++)
      {
         arcLengths[i] = fixedPointPositions[i].distance(fixedPointPositions[i + 1]);
      }

      return arcLengths;
   }

   private double[] getArcLengthsCalculatedIteratively()
   {
      double[] arcLengths = new double[3];

      for (int i = 0; i < arcLengths.length; i++)
      {
         arcLengths[i] = concatenatedSplinesWithArcLengthApproximatedByDistance.getSplineByIndex(i).getArcLength();
      }

      return arcLengths;
   }

   private void setWaypointTimes(double[] arcLengths)
   {
      double[] arcLengthsOutsideOfAccelerationPeriod = new double[3];

      arcLengthsOutsideOfAccelerationPeriod[0] = arcLengths[0]- arcLengthsOfAccelerationPeriods[0];
      arcLengthsOutsideOfAccelerationPeriod[1] = arcLengths[1];
      arcLengthsOutsideOfAccelerationPeriod[2] = arcLengths[2] - arcLengthsOfAccelerationPeriods[1];
      
      System.out.println("accel 1 period : " + arcLengthsOfAccelerationPeriods[0]);
      System.out.println("rest of 1 	 : " + arcLengthsOutsideOfAccelerationPeriod[0]);
      System.out.println("2			 	 : " + arcLengthsOutsideOfAccelerationPeriod[1]);
      System.out.println("beginning of 2 : " + arcLengthsOutsideOfAccelerationPeriod[2]);
      System.out.println("accel 2 period : " + arcLengthsOfAccelerationPeriods[1] +"\n\n");
      
      double totalArcLengthOutsideOfAccelerationPeriod = 0.0;
      for (int i = 0; i < arcLengthsOutsideOfAccelerationPeriod.length; i++)
      {
         totalArcLengthOutsideOfAccelerationPeriod += arcLengthsOutsideOfAccelerationPeriod[i];
      }
      
      double scaleFactor = (accelerationEndpointTimes[1].getDoubleValue() - accelerationEndpointTimes[0].getDoubleValue()) / totalArcLengthOutsideOfAccelerationPeriod;
      
      double[] timeDifferencesOutsideOfAccelerationPeriod = new double[3];
      for (int i = 0; i < timeDifferencesOutsideOfAccelerationPeriod.length; i++)
      {
         timeDifferencesOutsideOfAccelerationPeriod[i] = arcLengthsOutsideOfAccelerationPeriod[i] * scaleFactor;
      }
      
      fixedPointTimes[1].set(timeDifferencesOutsideOfAccelerationPeriod[0] / (1 - FRACTION_OF_TIME_TO_OR_FROM_WAYPOINT_FOR_SPEED_UP_OR_SLOW_DOWN));
      fixedPointTimes[2].set(fixedPointTimes[1].getDoubleValue() + timeDifferencesOutsideOfAccelerationPeriod[1]);

      accelerationEndpointTimes[0].set(FRACTION_OF_TIME_TO_OR_FROM_WAYPOINT_FOR_SPEED_UP_OR_SLOW_DOWN * fixedPointTimes[1].getDoubleValue());
      accelerationEndpointTimes[1].set(fixedPointTimes[3].getDoubleValue()
                                     - FRACTION_OF_TIME_TO_OR_FROM_WAYPOINT_FOR_SPEED_UP_OR_SLOW_DOWN
                                       * (fixedPointTimes[3].getDoubleValue() - fixedPointTimes[2].getDoubleValue()));
   }

   private void setWaypointVelocities()
   {
      setWaypointVelocity(0, 1);
      setWaypointVelocity(3, 2);
   }

   private void setWaypointVelocity(int indexOfInitialOrFinal, int indexOfWaypoint)
   {
      double t0 = fixedPointTimes[indexOfInitialOrFinal].getDoubleValue();
      double t1 = accelerationEndpointTimes[indexOfWaypoint - 1].getDoubleValue();
      double t2 = fixedPointTimes[indexOfWaypoint].getDoubleValue();

      FramePoint z0 = fixedPointPositions[indexOfInitialOrFinal].getFramePointCopy();
      FrameVector zd0 = fixedPointVelocities[indexOfInitialOrFinal].getFrameVectorCopy();
      FramePoint z2 = fixedPointPositions[indexOfWaypoint].getFramePointCopy();

      constraintsMatrix.setData(new double[]
      {
         t0 * t0 * t0, t0 * t0, t0, 1.0, 0.0, 0.0, 
         3.0 * t0 * t0, 2.0 * t0, 1.0, 0.0, 0.0, 0.0, 
         t1 * t1 * t1, t1 * t1, t1, 1.0, -t1, -1.0, 
         0.0, 0.0, 0.0, 0.0, t2, 1.0, 
         3.0 * t1 * t1, 2.0 * t1, 1.0, 0.0, -1.0, 0.0,
         6.0 * t1, 2.0, 0.0, 0.0, 0.0, 0.0
      });

      FrameVector velocity = new FrameVector(referenceFrame);
      FramePoint z1 = new FramePoint(referenceFrame);
      FrameVector zd1 = new FrameVector(referenceFrame);

      for (Direction d : Direction.values())
      {
         constraintsVector.setData(new double[] {z0.get(d), zd0.get(d), 0.0, z2.get(d), 0.0, 0.0});
         solve(constraintsMatrix, constraintsVector, coefficientsVector);
         double velocityComponent = coefficientsVector.get(4);
         z1.set(d, coefficientsVector.get(0)*t1*t1*t1+coefficientsVector.get(1)*t1*t1+coefficientsVector.get(2)*t1+coefficientsVector.get(3));
         zd1.set(d, 3*coefficientsVector.get(0)*t1*t1+2*coefficientsVector.get(1)*t1+coefficientsVector.get(2));
         velocity.set(d, velocityComponent);
      }
      
      accelerationPeriod.setCubic(t0, t1, z0, zd0, z1, zd1);
      arcLengthsOfAccelerationPeriods[indexOfWaypoint - 1] = accelerationPeriod.getArcLength(t0, t1);

      accelerationEndpointVelocities[indexOfWaypoint - 1].set(velocity);
      fixedPointVelocities[indexOfWaypoint].set(velocity);
    
      FrameVector initialOrFinalToWaypoint = new FrameVector(referenceFrame);
      FrameVector projectedVelocity = new FrameVector(referenceFrame);
      initialOrFinalToWaypoint.sub(fixedPointPositions[indexOfWaypoint + 1].getFramePointCopy(), fixedPointPositions[indexOfWaypoint - 1].getFramePointCopy());
      initialOrFinalToWaypoint.normalize();
      projectedVelocity.set(initialOrFinalToWaypoint);
      double magnitudeOfProjection = velocity.dot(initialOrFinalToWaypoint);
      projectedVelocity.scale(magnitudeOfProjection);
      fixedPointVelocities[indexOfWaypoint].set(projectedVelocity);
      
      FrameVector projectedIntermediateVelocity = new FrameVector(referenceFrame);
      FrameVector velocityComponent = new FrameVector(velocity);
      velocityComponent.scale(1-FRACTION_OF_TIME_TO_OR_FROM_WAYPOINT_FOR_SPEED_UP_OR_SLOW_DOWN);
      FrameVector projectedComponent = new FrameVector(referenceFrame); // TODO fix
      projectedComponent.scale(FRACTION_OF_TIME_TO_OR_FROM_WAYPOINT_FOR_SPEED_UP_OR_SLOW_DOWN);
      projectedIntermediateVelocity.add(velocityComponent, projectedComponent);
      projectedIntermediateVelocity.normalize();
      projectedIntermediateVelocity.scale(projectedVelocity.length());
      accelerationEndpointVelocities[indexOfWaypoint - 1].set(projectedIntermediateVelocity);
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

   private void setWaypointPositions(FramePoint[] waypoints)
   {
      if (waypoints == null)
      {
        setWaypointsToDefault();
      }
      else if (waypoints.length == 2)
      {
         waypoints[0].changeFrame(referenceFrame);
         waypoints[1].changeFrame(referenceFrame);
         fixedPointPositions[1].set(waypoints[0]);
         fixedPointPositions[2].set(waypoints[1]);
      }
      else
      {
         throw new RuntimeException("TwoWaypointPositionTrajectoryGenerator only supports trajectory generation for two waypoints.");
      }
   }
   
   private void setWaypointsToDefault()
   {
       FramePoint initialPosition = fixedPointPositions[0].getFramePointCopy();
       FramePoint finalPosition = fixedPointPositions[3].getFramePointCopy();
       FramePoint[] waypoints = new FramePoint[2];
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
          waypoints[i].setZ(waypoints[i].getZ() + groundClearance);
       }

       fixedPointPositions[1].set(waypoints[0]);
       fixedPointPositions[2].set(waypoints[1]);      
   }

   private void setFixedPointTimesAndWaypointVelocities(double[] arcLengths)
   {
      int timeVelocityIterations = 10;

      setInitialFixedPointTimes();

      for (int i = 0; i < timeVelocityIterations; i++)
      {
         setWaypointVelocities();
         setWaypointTimes(arcLengths);
//         System.out.println(Arrays.toString(fixedPointTimes));
//         System.out.println(Arrays.toString(fixedPointVelocities));
      }
   }

   private void setInitialFixedPointTimes()
   {
      for (int i = 0; i < 4; i++)
      {
         fixedPointTimes[i].set(stepTime.getDoubleValue() * ((double) i / 3.0));
      }

      accelerationEndpointTimes[0].set(FRACTION_OF_TIME_TO_OR_FROM_WAYPOINT_FOR_SPEED_UP_OR_SLOW_DOWN * fixedPointTimes[1].getDoubleValue());
      accelerationEndpointTimes[1].set(fixedPointTimes[3].getDoubleValue()
                                     - FRACTION_OF_TIME_TO_OR_FROM_WAYPOINT_FOR_SPEED_UP_OR_SLOW_DOWN
                                       * (fixedPointTimes[3].getDoubleValue() - fixedPointTimes[2].getDoubleValue()));
   }

   private void setConcatenatedSplines(YoConcatenatedSplines concatenatedSplines)
   {
      double[] times = new double[4];
      double[] intermediateTimes = new double[2];
      FramePoint[] positions = new FramePoint[4];
      FrameVector[] velocities = new FrameVector[4];
      FrameVector[] intermediateVelocities = new FrameVector[2];
      FrameVector[] intermediateAccelerations = new FrameVector[2];

      for (int i = 0; i < 4; i++)
      {
         times[i] = fixedPointTimes[i].getDoubleValue();
         positions[i] = fixedPointPositions[i].getFramePointCopy();
         velocities[i] = fixedPointVelocities[i].getFrameVectorCopy();
      }

      for (int i = 0; i < 2; i++)
      {
         intermediateTimes[i] = accelerationEndpointTimes[i].getDoubleValue();
         intermediateVelocities[i] = accelerationEndpointVelocities[i].getFrameVectorCopy();
         intermediateAccelerations[i] = new FrameVector(referenceFrame, 0.0, 0.0, 0.0);
      }

      concatenatedSplines.setQuinticsUsingIntermediateAccelerations(times, positions, velocities, intermediateTimes, intermediateVelocities, intermediateAccelerations);
   }

   private void visualizeSpline()
   {
      for (int i = 0; i < numberOfVisualizationMarkers; i++)
      {
         double t0 = concatenatedSplinesWithArcLengthCalculatedIteratively.getT0();
         double tf = concatenatedSplinesWithArcLengthCalculatedIteratively.getTf();
         double t = t0 + (double) i / (double) (numberOfVisualizationMarkers) * (tf - t0);
         compute(t, false);
         trajectoryBagOfBalls.setBall(desiredPosition.getFramePointCopy(), i);
      }

      for (int i = 0; i < fixedPointPositions.length; i++)
      {
         waypointBagOfBalls.setBall(fixedPointPositions[i].getFramePointCopy(), YoAppearance.AliceBlue(), i);
      }
   }

   public boolean isDone()
   {
      return timeIntoStep.getDoubleValue() >= stepTime.getDoubleValue();
   }
}
