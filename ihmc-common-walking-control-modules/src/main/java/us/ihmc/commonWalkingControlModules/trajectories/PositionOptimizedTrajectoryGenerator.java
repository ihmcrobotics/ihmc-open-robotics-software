package us.ihmc.commonWalkingControlModules.trajectories;

import static us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinitionFactory.newYoGraphicPolynomial3D;

import java.util.ArrayList;
import java.util.EnumMap;
import java.util.List;

import gnu.trove.list.array.TDoubleArrayList;
import gnu.trove.map.TIntIntMap;
import gnu.trove.map.hash.TIntIntHashMap;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPolynomial3D;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPolynomial3D.TrajectoryColorType;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.SCS2YoGraphicHolder;
import us.ihmc.robotics.math.trajectories.generators.TrajectoryPointOptimizer;
import us.ihmc.robotics.math.trajectories.interfaces.FixedFramePositionTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.yoVariables.YoPolynomial;
import us.ihmc.robotics.math.trajectories.yoVariables.YoPolynomial3D;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinitionFactory;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicGroupDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoListDefinition;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

/**
 * This class is a wrapper for the TrajectoryPointOptimizer. It was made for trajectories in 3d
 * space and creates third order trajectories. It can be used either to generate waypoint times as
 * velocities, or it can serve as an actual trajectory with the optimization taking place when the
 * trajectory is initialized. The trajectory is continuous in acceleration but does not have zero
 * initial and final acceleration. The optimization finds waypoint times and velocities such that
 * the overall squared acceleration is minimized.
 *
 * @author gwiedebach
 */
public class PositionOptimizedTrajectoryGenerator implements FixedFramePositionTrajectoryGenerator, SCS2YoGraphicHolder
{
   public static final int dimensions = 3;
   public final ReferenceFrame trajectoryFrame;

   private final String namePrefix;

   private final TrajectoryPointOptimizer optimizer;
   private final YoInteger maxIterations;
   private final RecyclingArrayList<TDoubleArrayList> coefficients;
   private final EnumMap<Axis3D, ArrayList<YoPolynomial>> trajectories = new EnumMap<>(Axis3D.class);
   private final double[] tempCoeffs = new double[TrajectoryPointOptimizer.coefficients];

   private final FramePoint3D initialPosition = new FramePoint3D();
   private final FrameVector3D initialVelocity = new FrameVector3D();
   private final FramePoint3D finalPosition = new FramePoint3D();
   private final FrameVector3D finalVelocity = new FrameVector3D();
   private final FramePoint3D waypointPosition = new FramePoint3D();
   private final RecyclingArrayList<TDoubleArrayList> waypointPositions;
   private final TIntIntMap indexMap = new TIntIntHashMap(10, 0.5f, -1, -1);

   private final TDoubleArrayList waypointVelocity = new TDoubleArrayList(dimensions);

   private final YoRegistry registry;
   private final YoBoolean isDone;
   private final YoBoolean optimizeInOneTick;
   private final YoBoolean hasConverged;
   private final YoInteger segments;
   private final YoInteger activeSegment;
   private final ArrayList<YoDouble> waypointTimes = new ArrayList<>();

   private final YoFramePoint3D desiredPosition;
   private final YoFrameVector3D desiredVelocity;
   private final YoFrameVector3D desiredAcceleration;

   private final YoGraphicPolynomial3D trajectoryViz;

   private final YoDouble maxSpeed;
   private final YoDouble maxSpeedTime;
   private final FrameVector3D tempVelocity = new FrameVector3D();

   private boolean visualize = true;

   public PositionOptimizedTrajectoryGenerator()
   {
      this("", null, ReferenceFrame.getWorldFrame());
   }

   public PositionOptimizedTrajectoryGenerator(int maxIterations, int maxWaypoints)
   {
      this(maxIterations, maxWaypoints, ReferenceFrame.getWorldFrame());
   }

   public PositionOptimizedTrajectoryGenerator(int maxIterations, int maxWaypoints, ReferenceFrame trajectoryFrame)
   {
      this("", null, null, maxIterations, maxWaypoints, trajectoryFrame);
   }

   public PositionOptimizedTrajectoryGenerator(String namePrefix, YoRegistry parentRegistry, ReferenceFrame trajectoryFrame)
   {
      this(namePrefix, parentRegistry, null, TrajectoryPointOptimizer.maxIterations, TrajectoryPointOptimizer.maxWaypoints, trajectoryFrame);
   }

   public PositionOptimizedTrajectoryGenerator(String namePrefix,
                                               YoRegistry parentRegistry,
                                               YoGraphicsListRegistry graphicsListRegistry,
                                               ReferenceFrame trajectoryFrame)
   {
      this(namePrefix, parentRegistry, graphicsListRegistry, TrajectoryPointOptimizer.maxIterations, TrajectoryPointOptimizer.maxWaypoints, trajectoryFrame);
   }

   public PositionOptimizedTrajectoryGenerator(String namePrefix,
                                               YoRegistry parentRegistry,
                                               YoGraphicsListRegistry graphicsListRegistry,
                                               int maxIterations,
                                               int maxWaypoints)
   {
      this(namePrefix, parentRegistry, graphicsListRegistry, maxIterations, maxWaypoints, ReferenceFrame.getWorldFrame());
   }

   public PositionOptimizedTrajectoryGenerator(String namePrefix,
                                               YoRegistry parentRegistry,
                                               YoGraphicsListRegistry graphicsListRegistry,
                                               int maxIterations,
                                               int maxWaypoints,
                                               ReferenceFrame trajectoryFrame)
   {
      this.namePrefix = namePrefix;
      this.trajectoryFrame = trajectoryFrame;

      coefficients = new RecyclingArrayList<>(0, () ->
      {
         TDoubleArrayList ret = new TDoubleArrayList(TrajectoryPointOptimizer.coefficients);
         for (int i = 0; i < TrajectoryPointOptimizer.coefficients; i++)
            ret.add(0.0);
         return ret;
      });

      waypointPositions = new RecyclingArrayList<>(0, () ->
      {
         TDoubleArrayList ret = new TDoubleArrayList(dimensions);
         for (int i = 0; i < dimensions; i++)
            ret.add(0.0);
         return ret;
      });

      registry = new YoRegistry(namePrefix + "Trajectory");
      optimizer = new TrajectoryPointOptimizer(namePrefix, dimensions, registry);
      this.maxIterations = new YoInteger(namePrefix + "MaxIterations", registry);
      this.maxIterations.set(maxIterations);
      isDone = new YoBoolean(namePrefix + "IsDone", registry);
      optimizeInOneTick = new YoBoolean(namePrefix + "OptimizeInOneTick", registry);
      hasConverged = new YoBoolean(namePrefix + "HasConverged", registry);
      segments = new YoInteger(namePrefix + "Segments", registry);
      activeSegment = new YoInteger(namePrefix + "ActiveSegment", registry);

      optimizeInOneTick.set(maxIterations >= 0);
      hasConverged.set(optimizeInOneTick.getBooleanValue());

      desiredPosition = new YoFramePoint3D(namePrefix + "DesiredPosition", trajectoryFrame, registry);
      desiredVelocity = new YoFrameVector3D(namePrefix + "DesiredVelocity", trajectoryFrame, registry);
      desiredAcceleration = new YoFrameVector3D(namePrefix + "DesiredAcceleration", trajectoryFrame, registry);

      for (Axis3D axis : Axis3D.values)
      {
         ArrayList<YoPolynomial> segments = new ArrayList<>();
         trajectories.put(axis, segments);
      }

      while (waypointTimes.size() <= maxWaypoints)
         extendBySegment(registry);

      reset();
      if (parentRegistry != null)
         parentRegistry.addChild(registry);

      if (graphicsListRegistry != null)
      {
         List<YoPolynomial3D> yoPolynomial3Ds = YoPolynomial3D.createYoPolynomial3DList(trajectories.get(Axis3D.X),
                                                                                        trajectories.get(Axis3D.Y),
                                                                                        trajectories.get(Axis3D.Z));
         trajectoryViz = new YoGraphicPolynomial3D(namePrefix + "Trajectory", null, yoPolynomial3Ds, waypointTimes, 0.01, 25, 8, registry);
         graphicsListRegistry.registerYoGraphic(namePrefix + "Trajectory", trajectoryViz);

         trajectoryViz.setColorType(TrajectoryColorType.ACCELERATION_BASED);
      }
      else
         trajectoryViz = null;

      maxSpeed = new YoDouble("MaxVelocity", registry);
      maxSpeedTime = new YoDouble("MaxVelocityTime", registry);
   }

   private void extendBySegment(YoRegistry registry)
   {
      int size = waypointTimes.size() + 1;
      for (Axis3D axis : Axis3D.values)
         trajectories.get(axis).add(new YoPolynomial(namePrefix + "Segment" + size + "Axis" + axis.ordinal(), TrajectoryPointOptimizer.coefficients, registry));
      waypointTimes.add(new YoDouble(namePrefix + "WaypointTime" + size, registry));
      waypointPositions.add();
   }

   /**
    * Resets the optimizer and removes all waypoints as well as previous start and end conditions.
    */
   public void reset()
   {
      initialPosition.setToNaN(trajectoryFrame);
      initialVelocity.setToNaN(trajectoryFrame);
      finalPosition.setToNaN(trajectoryFrame);
      finalVelocity.setToNaN(trajectoryFrame);
      segments.set(1);

      this.waypointPositions.clear();
      optimizer.setWaypoints(waypointPositions);
      coefficients.clear();
      coefficients.add();
   }

   /**
    * Set the desired position and velocity at the start and end points of the trajectory.
    *
    * @param initialPosition
    * @param initialVelocity
    * @param finalPosition
    * @param finalVelocity
    */
   public void setEndpointConditions(FramePoint3DReadOnly initialPosition,
                                     FrameVector3DReadOnly initialVelocity,
                                     FramePoint3DReadOnly finalPosition,
                                     FrameVector3DReadOnly finalVelocity)
   {
      this.initialPosition.setIncludingFrame(initialPosition);
      this.initialVelocity.setIncludingFrame(initialVelocity);
      this.initialPosition.changeFrame(trajectoryFrame);
      this.initialVelocity.changeFrame(trajectoryFrame);

      this.finalPosition.setIncludingFrame(finalPosition);
      this.finalVelocity.setIncludingFrame(finalVelocity);
      this.finalPosition.changeFrame(trajectoryFrame);
      this.finalVelocity.changeFrame(trajectoryFrame);

      for (Axis3D axis : Axis3D.values)
      {
         optimizer.setEndPoints(axis.ordinal(),
                                this.initialPosition.getElement(axis),
                                this.initialVelocity.getElement(axis),
                                this.finalPosition.getElement(axis),
                                this.finalVelocity.getElement(axis));
      }
   }

   /**
    * Sets the weights to use for the endpoint conditions.
    * <p>
    * Weights should be in <tt>[0, &infin;[</tt>. A weight set to {@link Double#POSITIVE_INFINITY} will
    * set up a hard constraint while any other real value will set up the condition as an objective.
    * </p>
    * 
    * @param initialPositionWeight the weight for the initial position condition. If {@code null}, it
    *                              is set up as a hard constraint. Not modified.
    * @param initialVelocityWeight the weight for the initial velocity condition. If {@code null}, it
    *                              is set up as a hard constraint. Not modified.
    * @param finalPositionWeight   the weight for the final position condition. If {@code null}, it is
    *                              set up as a hard constraint. Not modified.
    * @param finalVelocityWeight   the weight for the final velocity condition. If {@code null}, it is
    *                              set up as a hard constraint. Not modified.
    */
   public void setEndpointWeights(Tuple3DReadOnly initialPositionWeight,
                                  Tuple3DReadOnly initialVelocityWeight,
                                  Tuple3DReadOnly finalPositionWeight,
                                  Tuple3DReadOnly finalVelocityWeight)
   {
      for (Axis3D axis : Axis3D.values)
      {
         optimizer.setEndPointWeights(axis.ordinal(),
                                      initialPositionWeight == null ? Double.POSITIVE_INFINITY : initialPositionWeight.getElement(axis),
                                      initialVelocityWeight == null ? Double.POSITIVE_INFINITY : initialVelocityWeight.getElement(axis),
                                      finalPositionWeight == null ? Double.POSITIVE_INFINITY : finalPositionWeight.getElement(axis),
                                      finalVelocityWeight == null ? Double.POSITIVE_INFINITY : finalVelocityWeight.getElement(axis));
      }
   }

   /**
    * Set the positions of the waypoints on the trajectory.
    *
    * @param waypointPositions
    */
   public void setWaypoints(List<? extends FramePoint3DReadOnly> waypointPositions)
   {
      if (waypointPositions.size() > waypointTimes.size())
         throw new RuntimeException("Too many waypoints");

      this.waypointPositions.clear();
      coefficients.clear();
      indexMap.clear();

      coefficients.add();
      int optimizerIndex = 0;
      for (int i = 0; i < waypointPositions.size(); i++)
      {
         waypointPosition.setIncludingFrame(waypointPositions.get(i));
         waypointPosition.changeFrame(trajectoryFrame);

         if (i > 0 && waypointPosition.epsilonEquals(waypointPositions.get(i - 1), 1.0e-4))
         {
            optimizerIndex--;
            indexMap.put(i, optimizerIndex);
            continue;
         }

         indexMap.put(i, optimizerIndex);
         optimizerIndex++;

         TDoubleArrayList waypoint = this.waypointPositions.add();
         for (Axis3D axis : Axis3D.values)
            waypoint.set(axis.ordinal(), this.waypointPosition.getElement(axis.ordinal()));
         coefficients.add();
      }

      optimizer.setWaypoints(this.waypointPositions);
      segments.set(coefficients.size());
   }

   /**
    * This method initialized the trajectory and does the optimization. This has to be called after
    * setting the end point conditions and waypoints. It has to be called regardless of whether is
    * class is used as an actual trajectory or just to compute optimal waypoint times and velocities.
    */
   public void initialize()
   {
      if (initialPosition.containsNaN())
         throw new RuntimeException("Does not have valid enpoint conditions. Did you call setEndpointConditions?");

      if (optimizeInOneTick.getBooleanValue())
      {
         optimizer.compute(maxIterations.getIntegerValue());
         hasConverged.set(true);
      }
      else
      {
         hasConverged.set(false);
         optimizer.compute(0);
      }

      updateVariablesFromOptimizer();
   }

   private void updateVariablesFromOptimizer()
   {
      for (int i = 0; i < segments.getIntegerValue() - 1; i++)
         waypointTimes.get(i).set(optimizer.getWaypointTime(i));

      waypointTimes.get(segments.getIntegerValue() - 1).set(1.0);

      for (int i = segments.getIntegerValue(); i < waypointTimes.size(); i++)
         waypointTimes.get(i).set(Double.NaN);

      for (int dimension = 0; dimension < Axis3D.values.length; dimension++)
      {
         optimizer.getPolynomialCoefficients(coefficients, dimension);
         Axis3D axis = Axis3D.values[dimension];
         for (int i = 0; i < segments.getIntegerValue(); i++)
         {
            coefficients.get(i).toArray(tempCoeffs);
            YoPolynomial trajectory = trajectories.get(axis).get(i);
            trajectory.setDirectlyReverse(tempCoeffs);
            double startTime = i > 0 ? waypointTimes.get(i - 1).getDoubleValue() : 0.0;
            trajectory.getTimeInterval().setInterval(startTime, waypointTimes.get(i).getDoubleValue());
         }
      }

      isDone.set(false);

      if (visualize)
         visualize();
      else
         hide();
   }

   public void setShouldVisualize(boolean shouldVisualize)
   {
      this.visualize = shouldVisualize;
   }

   private void visualize()
   {
      if (trajectoryViz == null)
         return;

      trajectoryViz.showGraphic();
   }

   private void hide()
   {
      if (trajectoryViz == null)
         return;

      trajectoryViz.hideGraphic();
   }

   /**
    * Attempt at improving the trajectory if iterative improvement is desired.
    * 
    * @return whether an optimization step was done or not.
    */
   public boolean doOptimizationUpdate()
   {
      if (!hasConverged.getBooleanValue())
      {
         hasConverged.set(optimizer.doFullTimeUpdate());
         updateVariablesFromOptimizer();
      }

      return !hasConverged();
   }

   /**
    * Evaluates the trajectory at the given dimensionless time. Time is assumed to go from 0.0 at the
    * start of the trajectory to 1.0 at the end.
    *
    * @param time
    */
   public void compute(double time)
   {
      doOptimizationUpdate();
      isDone.set(time > 1.0);

      if (time < 0.0)
      {
         desiredPosition.set(initialPosition);
         desiredVelocity.setToZero();
         desiredAcceleration.setToZero();
         return;
      }
      if (time > 1.0)
      {
         desiredPosition.set(finalPosition);
         desiredVelocity.setToZero();
         desiredAcceleration.setToZero();
         return;
      }

      time = MathTools.clamp(time, 0.0, 1.0);

      int activeSegment = 0;
      for (int i = 0; i < segments.getIntegerValue() - 1; i++)
      {
         double waypointTime = waypointTimes.get(i).getDoubleValue();
         if (time > waypointTime)
            activeSegment = i + 1;
         else
            break;
      }
      this.activeSegment.set(activeSegment);

      for (int dimension = 0; dimension < Axis3D.values.length; dimension++)
      {
         Axis3D axis = Axis3D.values[dimension];
         YoPolynomial polynomial = trajectories.get(axis).get(activeSegment);
         polynomial.compute(time);
         desiredPosition.setElement(dimension, polynomial.getValue());
         desiredVelocity.setElement(dimension, polynomial.getVelocity());
         desiredAcceleration.setElement(dimension, polynomial.getAcceleration());
      }
   }

   /**
    * Call this function after initialize to retrieve the optimal waypoint time for a given waypoint
    * index.
    *
    * @param waypointIndex
    * @return
    */
   public double getWaypointTime(int waypointIndex)
   {
      return optimizer.getWaypointTime(indexMap.get(waypointIndex));
   }

   /**
    * Call this function after initialize to retrieve the optimal waypoint velocity for a given
    * waypoint index.
    *
    * @param waypointIndex
    * @param waypointVelocityToPack
    */
   public void getWaypointVelocity(int waypointIndex, FrameVector3D waypointVelocityToPack)
   {
      optimizer.getWaypointVelocity(this.waypointVelocity, indexMap.get(waypointIndex));
      waypointVelocityToPack.setToZero(trajectoryFrame);
      for (int d = 0; d < Axis3D.values.length; d++)
         waypointVelocityToPack.setElement(d, this.waypointVelocity.get(d));
   }

   /**
    * Call this function after initialize to retrieve the optimal initial position.
    * <p>
    * Note that this method is only useful when the initial position is configured as an objective, in
    * which case it can differ from the given position in
    * {@link #setEndpointConditions(FramePoint3DReadOnly, FrameVector3DReadOnly, FramePoint3DReadOnly, FrameVector3DReadOnly)}.
    * </p>
    * 
    * @param initialPositionToPack
    */
   public void getInitialPosition(FrameVector3DBasics initialPositionToPack)
   {
      optimizer.getStartPosition(waypointVelocity);
      initialPositionToPack.setReferenceFrame(trajectoryFrame);
      for (Axis3D axis : Axis3D.values)
         initialPositionToPack.setElement(axis, waypointVelocity.get(axis.ordinal()));
   }

   /**
    * Call this function after initialize to retrieve the optimal initial velocity.
    * <p>
    * Note that this method is only useful when the initial velocity is configured as an objective, in
    * which case it can differ from the given velocity in
    * {@link #setEndpointConditions(FramePoint3DReadOnly, FrameVector3DReadOnly, FramePoint3DReadOnly, FrameVector3DReadOnly)}.
    * </p>
    * 
    * @param initialVelocityToPack
    */
   public void getInitialVelocity(FrameVector3DBasics initialVelocityToPack)
   {
      optimizer.getStartVelocity(waypointVelocity);
      initialVelocityToPack.setReferenceFrame(trajectoryFrame);
      for (Axis3D axis : Axis3D.values)
         initialVelocityToPack.setElement(axis, waypointVelocity.get(axis.ordinal()));
   }

   /**
    * Call this function after finalize to retrieve the optimal final position.
    * <p>
    * Note that this method is only useful when the final position is configured as an objective, in
    * which case it can differ from the given position in
    * {@link #setEndpointConditions(FramePoint3DReadOnly, FrameVector3DReadOnly, FramePoint3DReadOnly, FrameVector3DReadOnly)}.
    * </p>
    * 
    * @param finalPositionToPack
    */
   public void getFinalPosition(FrameVector3DBasics finalPositionToPack)
   {
      optimizer.getTargetPosition(waypointVelocity);
      finalPositionToPack.setReferenceFrame(trajectoryFrame);
      for (Axis3D axis : Axis3D.values)
         finalPositionToPack.setElement(axis, waypointVelocity.get(axis.ordinal()));
   }

   /**
    * Call this function after initialize to retrieve the optimal final velocity.
    * <p>
    * Note that this method is only useful when the final velocity is configured as an objective, in
    * which case it can differ from the given velocity in
    * {@link #setEndpointConditions(FramePoint3DReadOnly, FrameVector3DReadOnly, FramePoint3DReadOnly, FrameVector3DReadOnly)}.
    * </p>
    * 
    * @param finalVelocityToPack
    */
   public void getFinalVelocity(FrameVector3DBasics finalVelocityToPack)
   {
      optimizer.getTargetVelocity(waypointVelocity);
      finalVelocityToPack.setReferenceFrame(trajectoryFrame);
      for (Axis3D axis : Axis3D.values)
         finalVelocityToPack.setElement(axis, waypointVelocity.get(axis.ordinal()));
   }

   public boolean isDone()
   {
      return isDone.getBooleanValue();
   }

   @Override
   public FramePoint3DReadOnly getPosition()
   {
      return desiredPosition;
   }

   @Override
   public FrameVector3DReadOnly getVelocity()
   {
      return desiredVelocity;
   }

   @Override
   public FrameVector3DReadOnly getAcceleration()
   {
      return desiredAcceleration;
   }

   public EnumMap<Axis3D, ArrayList<YoPolynomial>> getTrajectories()
   {
      return trajectories;
   }

   public void informDone()
   {
      desiredPosition.setToZero();
      desiredVelocity.setToZero();
      desiredAcceleration.setToZero();
   }

   public void showVisualization()
   {
      if (trajectoryViz == null)
         return;
      trajectoryViz.showGraphic();
   }

   public void hideVisualization()
   {
      if (trajectoryViz == null)
         return;
      trajectoryViz.hideGraphic();
   }

   /**
    * Returns whether the trajectory optimization has converged or not. This is useful when
    * continuously improving the solution quality instead of waiting for the optimizer to finish in the
    * initialize method.
    *
    * @return whether the optimizer has converged or not
    */
   public boolean hasConverged()
   {
      return hasConverged.getBooleanValue();
   }

   /**
    * Numerically compute the maximum speed along the trajectory and the time at which this speed
    * occurs.
    */
   public void computeMaxSpeed()
   {
      computeMaxSpeed(1.0E-5);
   }

   /**
    * Numerically compute the maximum speed along the trajectory and the time at which this speed
    * occurs. The time precision can be specified.
    */
   public void computeMaxSpeed(double timeIncrement)
   {
      maxSpeed.set(Double.NEGATIVE_INFINITY);
      maxSpeedTime.set(Double.NaN);

      for (double time = 0.0; time <= 1.0; time += timeIncrement)
      {
         compute(time);
         tempVelocity.setIncludingFrame(getVelocity());
         double speed = tempVelocity.norm();
         if (speed > maxSpeed.getDoubleValue())
         {
            maxSpeed.set(speed);
            maxSpeedTime.set(time);
         }
      }
   }

   public double getMaxSpeed()
   {
      return maxSpeed.getDoubleValue();
   }

   public double getMaxSpeedTime()
   {
      return maxSpeedTime.getDoubleValue();
   }

   @Override
   public YoGraphicDefinition getSCS2YoGraphics()
   {
      YoGraphicGroupDefinition group = new YoGraphicGroupDefinition(namePrefix + getClass().getSimpleName());
      for (int i = 0; i < trajectories.get(Axis3D.X).size(); i++)
      {
         YoPolynomial xPolynomial = trajectories.get(Axis3D.X).get(i);
         YoPolynomial yPolynomial = trajectories.get(Axis3D.Y).get(i);
         YoPolynomial zPolynomial = trajectories.get(Axis3D.Z).get(i);
         YoListDefinition coefficientsX = YoGraphicDefinitionFactory.toYoListDefinition(xPolynomial.getYoCoefficients(),
                                                                                        xPolynomial.getYoNumberOfCoefficients());
         YoListDefinition coefficientsY = YoGraphicDefinitionFactory.toYoListDefinition(yPolynomial.getYoCoefficients(),
                                                                                        yPolynomial.getYoNumberOfCoefficients());
         YoListDefinition coefficientsZ = YoGraphicDefinitionFactory.toYoListDefinition(zPolynomial.getYoCoefficients(),
                                                                                        zPolynomial.getYoNumberOfCoefficients());
         YoDouble startTime = i == 0 ? null : waypointTimes.get(i - 1); // null will be considered as 0
         YoDouble endTime = waypointTimes.get(i);
         group.addChild(newYoGraphicPolynomial3D(namePrefix + "Trajectory"
               + i, coefficientsX, coefficientsY, coefficientsZ, startTime, 0, endTime, 0, 0.01, ColorDefinitions.DodgerBlue()));
      }
      return group;
   }
}
