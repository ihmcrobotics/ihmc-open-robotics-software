package us.ihmc.commonWalkingControlModules.trajectories;

import org.apache.batik.dom.xbl.OriginalEvent;

import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.utilities.math.MathTools;
import us.ihmc.utilities.math.geometry.Direction;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;

import cern.colt.Arrays;

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
import com.yobotics.simulationconstructionset.util.trajectory.YoPositionProvider;

public class TwoWaypointPositionTrajectoryGenerator implements PositionTrajectoryGenerator
{
   private final static double[] DESIRED_PROPORTIONS_THROUGH_TRAJECTORY_FOR_GROUND_CLEARANCE = new double[] {1.0 / 3.0, 2.0 / 3.0};

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

   private final YoFramePoint[] fixedPointPositions = new YoFramePoint[4];
   private final YoFrameVector[] fixedPointVelocities = new YoFrameVector[4];

   private final YoConcatenatedSplines origianlConcatenatedSplines;
   private final YoConcatenatedSplines respacedConcatenatedSplines;
   
   private double[] timesFor3Splines = new double[4];
   private double[] timesForArcLengthSplines;

   private final int desiredNumberOfSplines;

   public TwoWaypointPositionTrajectoryGenerator(String namePrefix, ReferenceFrame referenceFrame, YoVariableDoubleProvider stepTimeProvider,
           PositionProvider initialPositionProvider, VectorProvider initalVelocityProvider, YoPositionProvider finalPositionProvider,
           VectorProvider finalDesiredVelocityProvider, YoVariableRegistry parentRegistry, double groundClearance, int desiredNumberOfSplines,
           int arcLengthCalculatorDivisions, DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry)
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

      this.stepTime = new DoubleYoVariable("stepTime", registry);
      this.timeIntoStep = new DoubleYoVariable("timeIntoStep", registry);

      this.desiredPosition = new YoFramePoint(namePrefix + "DesiredPosition", referenceFrame, registry);
      this.desiredVelocity = new YoFrameVector(namePrefix + "DesiredVelocity", referenceFrame, registry);
      this.desiredAcceleration = new YoFrameVector(namePrefix + "DesiredAcceleration", referenceFrame, registry);

      this.groundClearance = groundClearance;

      for (int i = 0; i < 4; i++)
      {
         fixedPointPositions[i] = new YoFramePoint(namePrefix + "FixedPointPosition" + i, referenceFrame, registry);
         fixedPointVelocities[i] = new YoFrameVector(namePrefix + "FixedPointVelocity" + i, referenceFrame, registry);
      }

      this.origianlConcatenatedSplines = new YoConcatenatedSplines(new int[] {4, 6, 4}, referenceFrame, arcLengthCalculatorDivisions, registry, namePrefix + "OriginalConcatenatedSplines");

      int[] reparameterizedNumberOfCoefficientsPerPolynomial = new int[desiredNumberOfSplines];
      for (int i = 0; i < reparameterizedNumberOfCoefficientsPerPolynomial.length; i++)
      {
         reparameterizedNumberOfCoefficientsPerPolynomial[i] = 6;
      }

      this.respacedConcatenatedSplines = new YoConcatenatedSplines(reparameterizedNumberOfCoefficientsPerPolynomial, referenceFrame, 2, registry, namePrefix + "RespacedConcatenatedSplines");

      this.desiredNumberOfSplines = desiredNumberOfSplines;
      this.timesForArcLengthSplines = new double[desiredNumberOfSplines + 1];
   }

   public void compute(double time)
   {
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

   public void initialize(FramePoint[] waypoints)
   {
      setInitialAndFinalPositionsAndVelocities();
      setWaypointPositionsAndVelocities(waypoints);
      
      setStepTime();

      setOriginalConcatenatedSplines();

      if (desiredNumberOfSplines == 3)
      {
         respaceSplineRangesProportionalToCurrentArcLengths(origianlConcatenatedSplines, respacedConcatenatedSplines);
     //    resetWaypointVelocities();
     //    respaceSplineTemp(respacedConcatenatedSplines);
      }
      else
      {
         respaceSplineRangesWithEqualArcLengths(origianlConcatenatedSplines, respacedConcatenatedSplines);
      }

      if (VISUALIZE)
      {
         visualizeSpline();
      }
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
      double t = System.nanoTime();
      initialize(null);
      System.out.println(System.nanoTime() - t);
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
      setWaypointVelocities();
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
            waypoints[i].setZ(waypoints[i].getZ() + groundClearance);    //TODO z from lidar?
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

   private void setWaypointVelocities()
   {
      double[] waypointSpeeds = new double[] {2.0, 2.0};
      FrameVector[] waypointVelocities = new FrameVector[2];
      for (int i = 0; i < 2; i++)
      {
         waypointVelocities[i] = new FrameVector(referenceFrame);
         waypointVelocities[i].set(fixedPointPositions[i + 2].getFramePointCopy());
         waypointVelocities[i].sub(fixedPointPositions[i].getFramePointCopy());
         waypointVelocities[i].normalize();
         waypointVelocities[i].scale(waypointSpeeds[i]);
         fixedPointVelocities[i + 1].set(waypointVelocities[i]);
      }
   }
   
   private void resetWaypointVelocities()
   {
	      FrameVector[] waypointVelocities = new FrameVector[2];
	      double[] timesTemp = new double[4];
	      timesTemp[0] = 0.0;
	      timesTemp[3] = stepTime.getDoubleValue();
	      double d1 = fixedPointPositions[1].distance(fixedPointPositions[0]);
	      double d2 = fixedPointPositions[2].distance(fixedPointPositions[1]);
	      double d3 = fixedPointPositions[3].distance(fixedPointPositions[2]);
	      double totalD = d1+d2+d3;
	      timesTemp[1] = (d1/totalD)*stepTime.getDoubleValue();
	      timesTemp[2] = ((d1+d2)/totalD)*stepTime.getDoubleValue();
	      System.out.println("timesTemp = " + Arrays.toString(timesTemp));
	      for (int i = 0; i < 1; i++)
	      {
	         waypointVelocities[i] = new FrameVector(referenceFrame);
	         waypointVelocities[i].set(fixedPointPositions[i + 2].getFramePointCopy());
	         waypointVelocities[i].sub(fixedPointPositions[i].getFramePointCopy());
	         waypointVelocities[i].normalize();
	         double scaleFactor = 0.0;
	         
	         for(Direction d : Direction.values())
	         {
	        	 double invTime = 1 / (timesFor3Splines[i] - timesFor3Splines[i+1]);
	        	 double a = invTime * (-invTime*fixedPointPositions[i].get(d) + invTime*fixedPointPositions[i+1].get(d) + fixedPointVelocities[i].get(d));
	        	 double b = invTime * (2 * timesFor3Splines[i] * invTime * (fixedPointPositions[i].get(d) - fixedPointPositions[i+1].get(d)) - (timesFor3Splines[i+1] + timesFor3Splines[i])*fixedPointVelocities[i].get(d));
	        	 scaleFactor += waypointVelocities[i].get(d)*(2*a*timesFor3Splines[i+1] + b);
	         }
	         System.out.println("scale factor = " + scaleFactor);
	         waypointVelocities[i].scale(scaleFactor);
	         fixedPointVelocities[i + 1].set(waypointVelocities[i]);
	      }

	      for (int i = 1; i < 2; i++)
	      {
	         waypointVelocities[i] = new FrameVector(referenceFrame);
	         waypointVelocities[i].set(fixedPointPositions[i + 2].getFramePointCopy());
	         waypointVelocities[i].sub(fixedPointPositions[i].getFramePointCopy());
	         waypointVelocities[i].normalize();
	         double scaleFactor = 0.0;
	         
	         for(Direction d : Direction.values())
	         {
	        	 double invTime = 1 / (timesFor3Splines[i+2] - timesFor3Splines[i+1]);
	        	 double a = invTime * (-invTime*fixedPointPositions[i+2].get(d) + invTime*fixedPointPositions[i+1].get(d) + fixedPointVelocities[i+2].get(d));
	        	 double b = invTime * (2 * timesFor3Splines[i+2] * invTime * (fixedPointPositions[i+2].get(d) - fixedPointPositions[i+1].get(d)) - (timesFor3Splines[i+1] + timesFor3Splines[i+2])*fixedPointVelocities[i+2].get(d));
	        	 scaleFactor += waypointVelocities[i].get(d)*(2*a*timesFor3Splines[i+1] + b);
	         }
	         System.out.println("scale factor = " + scaleFactor);
	         waypointVelocities[i].scale(scaleFactor);
	         fixedPointVelocities[i + 1].set(waypointVelocities[i]);
	      }
   }

   private void setOriginalConcatenatedSplines()
   {
      double[] times = new double[4];
      FramePoint[] positions = new FramePoint[4];
      FrameVector[] velocities = new FrameVector[4];
      double[] distances = new double[4];
      double deltaDistance;

      distances[0] = 0.0;

      for (int i = 1; i < 4; i++)
      {
         deltaDistance = fixedPointPositions[i - 1].distance(fixedPointPositions[i]);
         distances[i] = distances[i - 1] + deltaDistance;
      }

      double totalDistance = distances[3];

      for (int i = 0; i < 4; i++)
      {
         times[i] = distances[i] * stepTime.getDoubleValue() / totalDistance;
         positions[i] = fixedPointPositions[i].getFramePointCopy();
         velocities[i] = fixedPointVelocities[i].getFrameVectorCopy();
      }

      origianlConcatenatedSplines.setCubicQuinticCubic(times, positions, velocities);
      System.out.println("times = " + Arrays.toString(times));
      this.timesFor3Splines = times;
   }

   public void respaceSplineRangesProportionalToCurrentArcLengths(YoConcatenatedSplines oldSplines, YoConcatenatedSplines newSplines)
   {
      double[] oldTimes = new double[desiredNumberOfSplines + 1];
      double[] newTimes = new double[desiredNumberOfSplines + 1];

      double t0 = oldSplines.getT0();
      double tf = oldSplines.getTf();
      double totalTime = tf - t0;

      double totalArcLength = oldSplines.getArcLength();
      double cumulativeArcLength = 0.0;
      YoSpline3D oldSpline;

      oldTimes[0] = t0;
      newTimes[0] = t0;

      for (int i = 1; i < oldTimes.length - 1; i++)
      {
         oldSpline = oldSplines.getSplineByIndex(i - 1);
         oldTimes[i] = oldSpline.getTf();
         cumulativeArcLength += oldSpline.getArcLength();
         newTimes[i] = t0 + totalTime * cumulativeArcLength / totalArcLength;
      }

      oldTimes[oldTimes.length - 1] = tf;
      newTimes[newTimes.length - 1] = tf;

      newSplines.setQuintics(oldSplines, oldTimes, newTimes);
      this.timesForArcLengthSplines = newTimes;
   }
   
//   public void respaceSplineTemp(YoConcatenatedSplines newSplines)
//   {
//	   FrameVector[] accelerations = new FrameVector[4];
//	   origianlConcatenatedSplines.compute(timesFor3Splines[0]);
//	   accelerations[0] = origianlConcatenatedSplines.getAcceleration();
//	   origianlConcatenatedSplines.compute(timesFor3Splines[1]);
//	   accelerations[1] = origianlConcatenatedSplines.getAcceleration();
//	   origianlConcatenatedSplines.compute(timesFor3Splines[2]);
//	   accelerations[2] = origianlConcatenatedSplines.getAcceleration();
//	   origianlConcatenatedSplines.compute(timesFor3Splines[3]);
//	   accelerations[3] = origianlConcatenatedSplines.getAcceleration();	 
//	   newSplines.setQuintics(timesFor3Splines, fixedPointPositions, fixedPointVelocities, accelerations);
//   }

   public void respaceSplineRangesWithEqualArcLengths(YoConcatenatedSplines oldSplines, YoConcatenatedSplines newSplines)
   {
      int desiredNumberOfSplines = newSplines.getNumberOfSplines();

      double[] oldTimes = new double[desiredNumberOfSplines + 1];
      double[] newTimes = new double[desiredNumberOfSplines + 1];

      double t0 = oldSplines.getT0();
      double tf = oldSplines.getTf();
      double totalTime = tf - t0;

      double totalArcLength = oldSplines.getArcLength();
      double desiredIndividualArcLength = totalArcLength / (double) desiredNumberOfSplines;

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

   private void visualizeSpline()
   {
      for (int i = 0; i < numberOfVisualizationMarkers; i++)
      {
         double t0 = respacedConcatenatedSplines.getT0();
         double tf = respacedConcatenatedSplines.getTf();
         double t = t0 + (double) i / (double) (numberOfVisualizationMarkers) * (tf - t0);
         compute(t);
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
