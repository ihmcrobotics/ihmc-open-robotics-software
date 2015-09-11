package us.ihmc.commonWalkingControlModules.trajectories;

import java.util.Arrays;

import javax.vecmath.AxisAngle4d;
import javax.vecmath.Vector3d;

import org.apache.commons.math3.analysis.polynomials.PolynomialFunction;
import org.apache.commons.math3.analysis.solvers.LaguerreSolver;

import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.dataStructures.listener.VariableChangedListener;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.YoVariable;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.GeometryTools;
import us.ihmc.robotics.math.frames.YoFramePointInMultipleFrames;
import us.ihmc.robotics.math.frames.YoFrameVectorInMultipleFrames;
import us.ihmc.robotics.math.trajectories.PositionTrajectoryGeneratorInMultipleFrames;
import us.ihmc.robotics.math.trajectories.YoPolynomial;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.yoUtilities.graphics.BagOfBalls;
import us.ihmc.yoUtilities.graphics.YoGraphicPosition;
import us.ihmc.yoUtilities.graphics.YoGraphicVector;
import us.ihmc.yoUtilities.graphics.YoGraphicsList;
import us.ihmc.yoUtilities.graphics.YoGraphicsListRegistry;

/**
 * Created by agrabertilton on 10/21/14.
 */
public class TwoViaPointTrajectoryGenerator extends PositionTrajectoryGeneratorInMultipleFrames
{
   private final YoVariableRegistry registry;

   private final DoubleYoVariable currentTime;
   private final DoubleYoVariable trajectoryTime;
   private final DoubleYoVariable trajectoryAcclerationCost;
   private final YoPolynomial distancePolynomial;
   private final YoPolynomial xPolynomial1, xPolynomial2, xPolynomial3, yPolynomial1, yPolynomial2, yPolynomial3, zPolynomial1, zPolynomial2, zPolynomial3;

   private final YoFramePointInMultipleFrames initialPosition;
   private final YoFramePointInMultipleFrames finalPosition;
   private final YoFramePointInMultipleFrames firstViaPosition;
   private final YoFramePointInMultipleFrames secondViaPosition;
   private final YoFrameVectorInMultipleFrames initialDirection;
   private final YoFrameVectorInMultipleFrames finalDirection;

   private final YoFramePointInMultipleFrames currentPosition;
   private final YoFrameVectorInMultipleFrames currentVelocity;
   private final YoFrameVectorInMultipleFrames currentAcceleration;

   private final DoubleYoVariable leaveDistance;
   private final DoubleYoVariable approachDistance;
   private final DoubleYoVariable finalVelocity;

   // For viz
   private final boolean visualize;
   private final BagOfBalls bagOfBalls;
   private final FramePoint ballPosition = new FramePoint();
   private final int numberOfBalls = 50;

   /** Use a BooleanYoVariable to hide and show visualization with a VariableChangedListener, so it is still working in playback mode. */
   private final BooleanYoVariable showViz;

   public TwoViaPointTrajectoryGenerator(String namePrefix, ReferenceFrame referenceFrame, YoVariableRegistry parentRegistry)
   {
      this(namePrefix, false, referenceFrame, parentRegistry, false, null);
   }

   public TwoViaPointTrajectoryGenerator(String namePrefix, ReferenceFrame referenceFrame, YoVariableRegistry parentRegistry, boolean visualize,
         YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this(namePrefix, false, referenceFrame, parentRegistry, visualize, yoGraphicsListRegistry);
   }

   public TwoViaPointTrajectoryGenerator(String namePrefix, boolean allowMultipleFrames, ReferenceFrame referenceFrame, YoVariableRegistry parentRegistry)
   {
      this(namePrefix, allowMultipleFrames, referenceFrame, parentRegistry, false, null);
   }

   public TwoViaPointTrajectoryGenerator(String namePrefix, boolean allowMultipleFrames, ReferenceFrame referenceFrame, YoVariableRegistry parentRegistry,
         boolean visualize, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      super(allowMultipleFrames, referenceFrame);
      registry = new YoVariableRegistry(namePrefix + getClass().getSimpleName());
      trajectoryTime = new DoubleYoVariable(namePrefix + "TrajectoryTime", registry);
      currentTime = new DoubleYoVariable(namePrefix + "Time", registry);
      trajectoryAcclerationCost = new DoubleYoVariable(namePrefix + "TrajectoryAccelerationCost", registry);
      trajectoryAcclerationCost.set(1.0);
      distancePolynomial = new YoPolynomial(namePrefix + "DistancePolynomial", 6, registry);

      xPolynomial1 = new YoPolynomial(namePrefix + "XPositionSpline1", 4, registry);
      yPolynomial1 = new YoPolynomial(namePrefix + "YPositionSpline1", 4, registry);
      zPolynomial1 = new YoPolynomial(namePrefix + "ZPositionSpline1", 4, registry);

      xPolynomial2 = new YoPolynomial(namePrefix + "XPositionSpline2", 6, registry);
      yPolynomial2 = new YoPolynomial(namePrefix + "YPositionSpline2", 6, registry);
      zPolynomial2 = new YoPolynomial(namePrefix + "ZPositionSpline2", 6, registry);

      xPolynomial3 = new YoPolynomial(namePrefix + "XPositionSpline3", 4, registry);
      yPolynomial3 = new YoPolynomial(namePrefix + "YPositionSpline3", 4, registry);
      zPolynomial3 = new YoPolynomial(namePrefix + "ZPositionSpline3", 4, registry);

      initialPosition = new YoFramePointInMultipleFrames(namePrefix + "InitialPosition", registry, referenceFrame);
      finalPosition = new YoFramePointInMultipleFrames(namePrefix + "FinalPosition", registry, referenceFrame);
      firstViaPosition = new YoFramePointInMultipleFrames(namePrefix + "FirstViaPoint", registry, referenceFrame);
      secondViaPosition = new YoFramePointInMultipleFrames(namePrefix + "SecondViaPoint", registry, referenceFrame);
      initialDirection = new YoFrameVectorInMultipleFrames(namePrefix + "InitialDirection", registry, referenceFrame);
      finalDirection = new YoFrameVectorInMultipleFrames(namePrefix + "FinalDirection", registry, referenceFrame);
      currentPosition = new YoFramePointInMultipleFrames(namePrefix + "CurrentPosition", registry, referenceFrame);
      currentVelocity = new YoFrameVectorInMultipleFrames(namePrefix + "CurrentVelocity", registry, referenceFrame);
      currentAcceleration = new YoFrameVectorInMultipleFrames(namePrefix + "CurrentAcceleration", registry, referenceFrame);

      leaveDistance = new DoubleYoVariable(namePrefix + "LeaveDistance", registry);
      approachDistance = new DoubleYoVariable(namePrefix + "ApproachDistance", registry);
      finalVelocity = new DoubleYoVariable(namePrefix + "FinalVelocity", registry);

      registerMultipleFramesHolders(initialPosition, finalPosition, currentPosition, currentVelocity, currentAcceleration);

      parentRegistry.addChild(registry);

      this.visualize = visualize && yoGraphicsListRegistry != null;

      if (this.visualize)
      {
         final YoGraphicPosition currentPositionViz = new YoGraphicPosition(namePrefix + "CurrentPosition", currentPosition, 0.025, YoAppearance.Blue());
         final YoGraphicPosition initialPositionViz = new YoGraphicPosition(namePrefix + "InitialPosition", initialPosition, 0.02, YoAppearance.BlueViolet());
         final YoGraphicPosition finalPositionViz = new YoGraphicPosition(namePrefix + "FinalPosition", finalPosition, 0.02, YoAppearance.Red());
         final YoGraphicVector initialDirectionViz = new YoGraphicVector(namePrefix + "InitialDirection",
               initialPosition.buildUpdatedYoFramePointForVisualizationOnly(), initialDirection.buildUpdatedYoFrameVectorForVisualizationOnly(), 0.2,
               YoAppearance.BlueViolet());
         final YoGraphicVector finalDirectionViz = new YoGraphicVector(namePrefix + "FinalDirection",
               finalPosition.buildUpdatedYoFramePointForVisualizationOnly(), finalDirection.buildUpdatedYoFrameVectorForVisualizationOnly(), 0.2,
               YoAppearance.Red());
         YoGraphicsList yoGraphicsList = new YoGraphicsList(namePrefix + "FinalApproachTraj");
         yoGraphicsList.add(currentPositionViz);
         yoGraphicsList.add(initialPositionViz);
         yoGraphicsList.add(finalPositionViz);
         yoGraphicsList.add(initialDirectionViz);
         yoGraphicsList.add(finalDirectionViz);
         yoGraphicsListRegistry.registerYoGraphicsList(yoGraphicsList);

         bagOfBalls = new BagOfBalls(numberOfBalls, 0.01, yoGraphicsList.getLabel(), registry, yoGraphicsListRegistry);

         showViz = new BooleanYoVariable(namePrefix + "ShowViz", registry);
         showViz.addVariableChangedListener(new VariableChangedListener()
         {
            @Override
            public void variableChanged(YoVariable<?> v)
            {
               boolean visible = showViz.getBooleanValue();
               currentPositionViz.setVisible(visible);
               initialPositionViz.setVisible(visible);
               finalPositionViz.setVisible(visible);
               initialDirectionViz.setVisible(visible);
               finalDirectionViz.setVisible(visible);
               bagOfBalls.setVisible(visible);
               if (!visible)
                  bagOfBalls.hideAll();
            }
         });
         showViz.notifyVariableChangedListeners();
      }
      else
      {
         bagOfBalls = null;
         showViz = null;
      }
   }

   private final Vector3d tempVector = new Vector3d();
   private final AxisAngle4d tempAxisAngle = new AxisAngle4d();

   public void setInitialLeadOut(FramePoint initialPosition, FrameVector initialDirection, double leaveDistance)
   {
      this.initialPosition.set(initialPosition);
      this.initialDirection.set(initialDirection);
      this.initialDirection.normalize();
      this.initialDirection.get(tempVector);
      GeometryTools.getRotationBasedOnNormal(tempAxisAngle, tempVector);
      this.leaveDistance.set(leaveDistance);
   }

   public void setFinalLeadIn(FramePoint finalPosition, FrameVector finalDirection, double approachDistance)
   {
      this.finalPosition.set(finalPosition);
      this.finalDirection.set(finalDirection);
      this.finalDirection.normalize();
      this.finalDirection.get(tempVector);
      tempVector.negate();
      GeometryTools.getRotationBasedOnNormal(tempAxisAngle, tempVector);
      this.approachDistance.set(approachDistance);
   }

   public void setFinalVelocity(double finalVelocity)
   {
      this.finalVelocity.set(finalVelocity);
   }

   public void setTrajectoryTime(double newTrajectoryTime)
   {
      trajectoryTime.set(newTrajectoryTime);
   }

   double[] timeValues = null;

   @Override
   public void initialize()
   {
      timeValues = computePolynomialsAndReturnTimeValues();
      if (visualize)
         visualizeTrajectory();
      currentTime.set(0.0);
      currentPosition.set(initialPosition);
      currentVelocity.setToZero();
      currentAcceleration.setToZero();

   }

   @Override
   public void compute(double time)
   {
      if (timeValues == null)
         initialize();
      currentTime.set(time);
      double endTime = timeValues[timeValues.length - 1];
      //Determine values at a given time point
      double clippedTime = MathTools.clipToMinMax(time, 0.0, endTime);

      YoPolynomial currentXPolynomial;
      YoPolynomial currentYPolynomial;
      YoPolynomial currentZPolynomial;
      if (clippedTime < timeValues[1])
      {
         currentXPolynomial = xPolynomial1;
         currentYPolynomial = yPolynomial1;
         currentZPolynomial = zPolynomial1;
      }
      else if (clippedTime < timeValues[2])
      {
         currentXPolynomial = xPolynomial2;
         currentYPolynomial = yPolynomial2;
         currentZPolynomial = zPolynomial2;
      }
      else
      {
         currentXPolynomial = xPolynomial3;
         currentYPolynomial = yPolynomial3;
         currentZPolynomial = zPolynomial3;
      }

      currentXPolynomial.compute(clippedTime);
      currentYPolynomial.compute(clippedTime);
      currentZPolynomial.compute(clippedTime);

      //set position, velocity, and acceleration
      currentPosition.set(currentXPolynomial.getPosition(), currentYPolynomial.getPosition(), currentZPolynomial.getPosition());
      currentVelocity.set(currentXPolynomial.getVelocity(), currentYPolynomial.getVelocity(), currentZPolynomial.getVelocity());
      currentAcceleration.set(currentXPolynomial.getAcceleration(), currentYPolynomial.getAcceleration(), currentZPolynomial.getAcceleration());
   }

   private double[] computePolynomialsAndReturnTimeValues()
   {
      updateViaPoints();

      double[] distances = getDistances();
      double totalDistance = 0;
      for (int i = 0; i < distances.length; i++)
      {
         totalDistance += distances[i];
      }
      double endTime = trajectoryTime.getDoubleValue();

      double[] interpolationValues = getInterpolationValuesBasedOnDistance(distances);

      double initialVelocity = 0;
      double initialAcceleration = 0;
      double finalVelocity = this.finalVelocity.getDoubleValue();
      double finalAcceleration = 0.0;
      double[] speedValues = new double[4];
      getInterpolationValuesBasedOnTimeCurve(interpolationValues, endTime, totalDistance, initialVelocity, initialAcceleration, finalVelocity,
            finalAcceleration, speedValues);

      //start of new stuff
      Vector3d velocityAtWaypoint1 = initialDirection.getVector3dCopy();
      Vector3d velocityAtWaypoint2 = finalDirection.getVector3dCopy();
      velocityAtWaypoint1.scale(speedValues[1]);
      velocityAtWaypoint2.scale(speedValues[2]);
      double compensatoryDistance = getCompensatoryDistanceBasedOnVelocityChange(velocityAtWaypoint1, velocityAtWaypoint2);

      double[] adjustedInterpolationValues = getInterpolationValuesBasedOnDistanceWithExtra(distances, compensatoryDistance);

      double[] thirdPassInterpolationValues = getInterpolationValuesBasedOnTimeCurve(adjustedInterpolationValues, endTime, totalDistance, initialVelocity,
            initialAcceleration, finalVelocity, finalAcceleration, speedValues);

      double[] timeValues = new double[4];
      for (int i = 0; i < interpolationValues.length; i++)
      {
         timeValues[i] = endTime * thirdPassInterpolationValues[i];
      }

      fitPolynomials(timeValues, initialVelocity, initialAcceleration, finalVelocity, finalAcceleration);
      return timeValues;
   }

   private void updateViaPoints()
   {
      MathTools.checkIfInRange(trajectoryTime.getDoubleValue(), 0.0, Double.POSITIVE_INFINITY);
      firstViaPosition.set(initialDirection);
      firstViaPosition.scale(leaveDistance.getDoubleValue());
      firstViaPosition.add(initialPosition);

      secondViaPosition.set(finalDirection);
      secondViaPosition.scale(-1 * approachDistance.getDoubleValue());
      secondViaPosition.add(finalPosition);
   }

   private double[] getDistances()
   {
      double[] distances = new double[3];
      distances[0] = firstViaPosition.distance(initialPosition);
      distances[1] = secondViaPosition.distance(firstViaPosition);
      distances[2] = finalPosition.distance(secondViaPosition);
      return distances;
   }

   private double[] getInterpolationValuesBasedOnDistance(double[] distances)
   {
      double[] interpolationValues = new double[4];
      double totalDistance = 0;
      for (int i = 0; i < distances.length; i++)
      {
         totalDistance += distances[i];
      }
      interpolationValues[0] = 0.0;
      interpolationValues[1] = distances[0] / totalDistance;
      interpolationValues[2] = 1 - distances[2] / totalDistance;
      interpolationValues[3] = 1;

      return interpolationValues;
   }

   private double[] getInterpolationValuesBasedOnDistanceWithExtra(double[] distances, double extraDistanceFactor)
   {
      double[] alteredDistances = Arrays.copyOf(distances, distances.length);
      alteredDistances[1] += extraDistanceFactor;
      return getInterpolationValuesBasedOnDistance(alteredDistances);
   }

   private double getCompensatoryDistanceBasedOnVelocityChange(Vector3d velocityAtTime1, Vector3d velocityAtTime2)
   {
      double velocityExponent = 2.0;

      double dotProduct = velocityAtTime1.dot(velocityAtTime2);
      double speed1 = velocityAtTime1.length();
      double speed2 = velocityAtTime2.length();
      if (speed1 < 1e-10 || speed2 < 1e-10)
         return 0;
      dotProduct /= (speed1 * speed2);
      double angle = Math.acos(dotProduct);

      //TODO should be a better approximation for this
      double averageVelocity = (speed1 + speed2) / 2;

      double extraDistance = trajectoryAcclerationCost.getDoubleValue() * angle * Math.pow(averageVelocity, velocityExponent);
      return extraDistance;
   }

   private double[] getInterpolationValuesBasedOnTimeCurve(double[] interpolationValues, double endTime, double totalDistance, double initialVelocity,
         double initialAcceleration, double finalVelocity, double finalAcceleration, double[] waypointSpeedsToPack)
   {
      distancePolynomial.setQuintic(0, endTime, 0, initialVelocity, initialAcceleration, totalDistance, finalVelocity, finalAcceleration);
      PolynomialFunction distancePoly = new PolynomialFunction(distancePolynomial.getCoefficients());
      LaguerreSolver distSolver = new LaguerreSolver();

      double[] newInterpolationValues = Arrays.copyOf(interpolationValues, interpolationValues.length);
      if (interpolationValues[1] != 0)
      {
         PolynomialFunction poly1 = distancePoly.subtract(new PolynomialFunction(new double[] { interpolationValues[1] * totalDistance }));
         newInterpolationValues[1] = distSolver.solve(1000, poly1, 0, endTime) / endTime;
      }

      if (interpolationValues[2] != 0)
      {
         PolynomialFunction poly2 = distancePoly.subtract(new PolynomialFunction(new double[] { interpolationValues[2] * totalDistance }));
         newInterpolationValues[2] = distSolver.solve(1000, poly2, 0, endTime) / endTime;
      }

      PolynomialFunction speedPoly = distancePoly.polynomialDerivative();
      waypointSpeedsToPack[0] = initialVelocity;
      waypointSpeedsToPack[1] = speedPoly.value(interpolationValues[1] * endTime);
      waypointSpeedsToPack[2] = speedPoly.value(interpolationValues[2] * endTime);
      waypointSpeedsToPack[3] = finalVelocity;
      return newInterpolationValues;
   }

   private void fitPolynomials(double[] timeValues, double initialVelocity, double initialAcceleration, double finalVelocity, double finalAcceleration)
   {
      //fit three splines
      if (timeValues[0] == timeValues[1])
      {
         xPolynomial1.setConstant(initialPosition.getX());
         yPolynomial1.setConstant(initialPosition.getY());
         zPolynomial1.setConstant(initialPosition.getZ());
      }
      else
      {
         xPolynomial1.setCubicThreeInitialConditionsFinalPosition(timeValues[0], timeValues[1], initialPosition.getX(),
               initialVelocity * initialDirection.getX(), 0, firstViaPosition.getX());
         yPolynomial1.setCubicThreeInitialConditionsFinalPosition(timeValues[0], timeValues[1], initialPosition.getY(),
               initialVelocity * initialDirection.getY(), 0, firstViaPosition.getY());
         zPolynomial1.setCubicThreeInitialConditionsFinalPosition(timeValues[0], timeValues[1], initialPosition.getZ(),
               initialVelocity * initialDirection.getZ(), 0, firstViaPosition.getZ());

      }
      xPolynomial1.compute(timeValues[1]);
      yPolynomial1.compute(timeValues[1]);
      zPolynomial1.compute(timeValues[1]);

      if (timeValues[2] == timeValues[3])
      {
         xPolynomial3.setConstant(finalPosition.getX());
         yPolynomial3.setConstant(finalPosition.getY());
         zPolynomial3.setConstant(finalPosition.getZ());
      }
      else
      {
         xPolynomial3.setCubicInitialPositionThreeFinalConditions(timeValues[2], timeValues[3], secondViaPosition.getX(), finalPosition.getX(), finalVelocity
               * finalDirection.getX(), 0);
         yPolynomial3.setCubicInitialPositionThreeFinalConditions(timeValues[2], timeValues[3], secondViaPosition.getY(), finalPosition.getY(), finalVelocity
               * finalDirection.getY(), 0);
         zPolynomial3.setCubicInitialPositionThreeFinalConditions(timeValues[2], timeValues[3], secondViaPosition.getZ(), finalPosition.getZ(), finalVelocity
               * finalDirection.getZ(), 0);
      }
      xPolynomial3.compute(timeValues[2]);
      yPolynomial3.compute(timeValues[2]);
      zPolynomial3.compute(timeValues[2]);

      if (timeValues[1] == timeValues[2])
      {
         xPolynomial2.setConstant(firstViaPosition.getX());
         yPolynomial2.setConstant(firstViaPosition.getY());
         zPolynomial2.setConstant(firstViaPosition.getZ());
      }
      else
      {
         xPolynomial2.setQuintic(timeValues[1], timeValues[2], xPolynomial1.getPosition(), xPolynomial1.getVelocity(), xPolynomial1.getAcceleration(),
               xPolynomial3.getPosition(), xPolynomial3.getVelocity(), xPolynomial3.getAcceleration());
         yPolynomial2.setQuintic(timeValues[1], timeValues[2], yPolynomial1.getPosition(), yPolynomial1.getVelocity(), yPolynomial1.getAcceleration(),
               yPolynomial3.getPosition(), yPolynomial3.getVelocity(), yPolynomial3.getAcceleration());
         zPolynomial2.setQuintic(timeValues[1], timeValues[2], zPolynomial1.getPosition(), zPolynomial1.getVelocity(), zPolynomial1.getAcceleration(),
               zPolynomial3.getPosition(), zPolynomial3.getVelocity(), zPolynomial3.getAcceleration());
      }
   }

   private void visualizeTrajectory()
   {
      for (int i = 0; i < numberOfBalls; i++)
      {
         double t = i / ((double) numberOfBalls - 1) * trajectoryTime.getDoubleValue();
         compute(t);
         currentPosition.getFrameTupleIncludingFrame(ballPosition);
         ballPosition.changeFrame(ReferenceFrame.getWorldFrame());
         bagOfBalls.setBallLoop(ballPosition);
      }
   }

   @Override
   public void showVisualization()
   {
      if (!visualize)
         return;

      showViz.set(true);
   }

   @Override
   public void hideVisualization()
   {
      if (!visualize)
         return;

      showViz.set(false);
   }

   @Override
   public void get(FramePoint positionToPack)
   {
      currentPosition.getFrameTupleIncludingFrame(positionToPack);
   }

   @Override
   public void packVelocity(FrameVector velocityToPack)
   {
      currentVelocity.getFrameTupleIncludingFrame(velocityToPack);
   }

   @Override
   public void packAcceleration(FrameVector accelerationToPack)
   {
      currentAcceleration.getFrameTupleIncludingFrame(accelerationToPack);
   }

   @Override
   public void packLinearData(FramePoint positionToPack, FrameVector velocityToPack, FrameVector accelerationToPack)
   {
      get(positionToPack);
      packVelocity(velocityToPack);
      packAcceleration(accelerationToPack);
   }

   @Override
   public boolean isDone()
   {
      return currentTime.getDoubleValue() >= trajectoryTime.getDoubleValue();
   }

   @Override
   public String toString()
   {
      String ret = "";

      ReferenceFrame currentFrame = initialPosition.getReferenceFrame();

      ret += "Current time: " + currentTime.getDoubleValue() + ", trajectory time: " + trajectoryTime.getDoubleValue();
      ret += "\nCurrent position: " + currentPosition.toStringForASingleReferenceFrame(currentFrame);
      ret += "\nCurrent velocity: " + currentVelocity.toStringForASingleReferenceFrame(currentFrame);
      ret += "\nCurrent acceleration: " + currentAcceleration.toStringForASingleReferenceFrame(currentFrame);
      return ret;
   }
}
