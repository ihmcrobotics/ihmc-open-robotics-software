package us.ihmc.commonWalkingControlModules.trajectories;

import java.util.ArrayList;
import java.util.EnumMap;

import gnu.trove.list.array.TDoubleArrayList;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.BagOfBalls;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.IntegerYoVariable;
import us.ihmc.robotics.geometry.Direction;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.lists.GenericTypeBuilder;
import us.ihmc.robotics.lists.RecyclingArrayList;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.math.trajectories.YoPolynomial;
import us.ihmc.robotics.math.trajectories.waypoints.PolynomialOrder;
import us.ihmc.robotics.math.trajectories.waypoints.TrajectoryPointOptimizer;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.referenceFrames.XYPlaneFrom3PointsFrame;

/**
 * Wrapper for TrajectoryPointOptimizer for the generation of swing trajectories. This is more efficient then
 * using the PositionOptimizedTrajectoryGenerator since it assumes that the trajectory can be reduced to a
 * two dimensional problem by smart choice of the reference frame the computation is done in. Use this class only
 * if your swing lies in a plane.
 *
 * Aside from the reduction to two dimensions this class is similar to PositionOptimizedTrajectoryGenerator
 *
 * @author gwiedebach
 *
 */
public class Position2dOptimizedSwingTrajectoryGenerator implements WaypointTrajectoryGenerator
{
   private static final int maxWaypoints = 12;
   private static final int dimensions = 2;
   private static final PolynomialOrder order = PolynomialOrder.ORDER3;
   private static final ReferenceFrame trajectoryFrame = ReferenceFrame.getWorldFrame();

   private final String namePrefix;

   private final TrajectoryPointOptimizer optimizer;
   private final IntegerYoVariable maxIterations;
   private final RecyclingArrayList<TDoubleArrayList> coefficients;
   private final EnumMap<Direction, ArrayList<YoPolynomial>> trajectories = new EnumMap<>(Direction.class);
   private final double[] tempCoeffs = new double[order.getCoefficients()];

   private final FramePoint initialPosition = new FramePoint();
   private final FrameVector initialVelocity = new FrameVector();
   private final FramePoint finalPosition = new FramePoint();
   private final FrameVector finalVelocity = new FrameVector();
   private final FramePoint waypointPosition = new FramePoint();
   private final RecyclingArrayList<TDoubleArrayList> waypointPositions;

   private final TDoubleArrayList initialPositionArray = new TDoubleArrayList(dimensions);
   private final TDoubleArrayList initialVelocityArray = new TDoubleArrayList(dimensions);
   private final TDoubleArrayList finalPositionArray = new TDoubleArrayList(dimensions);
   private final TDoubleArrayList finalVelocityArray = new TDoubleArrayList(dimensions);

   private final YoVariableRegistry registry;
   private final BooleanYoVariable isDone;
   private final IntegerYoVariable segments;
   private final IntegerYoVariable activeSegment;
   private final ArrayList<DoubleYoVariable> waypointTimes = new ArrayList<>();

   private final YoFramePoint desiredPosition;
   private final YoFrameVector desiredVelocity;
   private final YoFrameVector desiredAcceleration;

   private final FramePoint tempPos = new FramePoint();
   private final FrameVector tempVel = new FrameVector();
   private final FrameVector tempAcc = new FrameVector();

   private final int markers = 30;
   private final BagOfBalls trajectoryViz;
   private final FramePoint ballPosition = new FramePoint();

   private final XYPlaneFrom3PointsFrame swingFrame = new XYPlaneFrom3PointsFrame(trajectoryFrame, "swingFrame");
   private final FramePoint trajectoryOrigin = new FramePoint();
   private final FrameVector direction = new FrameVector();
   private final FramePoint pointOnX = new FramePoint();
   private final FramePoint pointOnY = new FramePoint();

   public Position2dOptimizedSwingTrajectoryGenerator()
   {
      this("", new YoVariableRegistry(""));
   }

   public Position2dOptimizedSwingTrajectoryGenerator(String namePrefix, YoVariableRegistry parentRegistry)
   {
      this(namePrefix, parentRegistry, null, TrajectoryPointOptimizer.maxIterations);
   }

   public Position2dOptimizedSwingTrajectoryGenerator(String namePrefix, YoVariableRegistry parentRegistry, YoGraphicsListRegistry graphicsListRegistry)
   {
      this(namePrefix, parentRegistry, graphicsListRegistry, TrajectoryPointOptimizer.maxIterations);
   }

   public Position2dOptimizedSwingTrajectoryGenerator(String namePrefix, YoVariableRegistry parentRegistry,
         YoGraphicsListRegistry graphicsListRegistry, int maxIterations)
   {
      this.namePrefix = namePrefix;

      coefficients = new RecyclingArrayList<>(new GenericTypeBuilder<TDoubleArrayList>()
      {
         @Override
         public TDoubleArrayList newInstance()
         {
            TDoubleArrayList ret = new TDoubleArrayList(order.getCoefficients());
            for (int i = 0; i < order.getCoefficients(); i++)
               ret.add(0.0);
            return ret;
         }
      });

      waypointPositions = new RecyclingArrayList<>(new GenericTypeBuilder<TDoubleArrayList>()
      {
         @Override
         public TDoubleArrayList newInstance()
         {
            TDoubleArrayList ret = new TDoubleArrayList(dimensions);
            for (int i = 0; i < dimensions; i++)
               ret.add(0.0);
            return ret;
         }
      });

      registry = new YoVariableRegistry(namePrefix + "Trajectory");
      optimizer = new TrajectoryPointOptimizer(dimensions, order, registry);
      this.maxIterations = new IntegerYoVariable(namePrefix + "MaxIterations", registry);
      this.maxIterations.set(maxIterations);
      isDone = new BooleanYoVariable(namePrefix + "IsDone", registry);
      segments = new IntegerYoVariable(namePrefix + "Segments", registry);
      activeSegment = new IntegerYoVariable(namePrefix + "ActiveSegment", registry);

      desiredPosition = new YoFramePoint(namePrefix + "DesiredPosition", trajectoryFrame, registry);
      desiredVelocity = new YoFrameVector(namePrefix + "DesiredVelocity", trajectoryFrame, registry);
      desiredAcceleration = new YoFrameVector(namePrefix + "DesiredAcceleration", trajectoryFrame, registry);

      for (int i = 0; i < dimensions; i++)
      {
         initialPositionArray.add(0.0);
         initialVelocityArray.add(0.0);
         finalPositionArray.add(0.0);
         finalVelocityArray.add(0.0);
      }

      for (Direction axis : Direction.values)
      {
         ArrayList<YoPolynomial> segments = new ArrayList<>();
         segments.add(new YoPolynomial(namePrefix + "Segment" + 0 + "Axis" + axis.getIndex(), order.getCoefficients(), registry));
         trajectories.put(axis, segments);
      }
      segments.set(1);

      while (waypointTimes.size() < maxWaypoints)
         extendBySegment(registry);

      parentRegistry.addChild(registry);

      if (graphicsListRegistry != null)
         trajectoryViz = new BagOfBalls(markers, 0.01, namePrefix + "Trajectory", YoAppearance.Black(), registry, graphicsListRegistry);
      else
         trajectoryViz = null;
   }

   private void extendBySegment(YoVariableRegistry registry)
   {
      int size = waypointTimes.size() + 1;
      for (Direction axis : Direction.values)
         trajectories.get(axis).add(new YoPolynomial(namePrefix + "Segment" + size + "Axis" + axis.getIndex(), order.getCoefficients(), registry));
      waypointTimes.add(new DoubleYoVariable(namePrefix + "WaypointTime" + size, registry));
      waypointPositions.add();
   }

   @Override
   public void setEndpointConditions(FramePoint initialPosition, FrameVector initialVelocity, FramePoint finalPosition, FrameVector finalVelocity)
   {
      this.initialPosition.setIncludingFrame(initialPosition);
      this.initialVelocity.setIncludingFrame(initialVelocity);
      this.initialPosition.changeFrame(trajectoryFrame);
      this.initialVelocity.changeFrame(trajectoryFrame);

      this.finalPosition.setIncludingFrame(finalPosition);
      this.finalVelocity.setIncludingFrame(finalVelocity);
      this.finalPosition.changeFrame(trajectoryFrame);
      this.finalVelocity.changeFrame(trajectoryFrame);

      // find the direction of the swing to reduce the dimensionality of the problem.
      trajectoryOrigin.setIncludingFrame(this.initialPosition);
      pointOnX.setIncludingFrame(this.finalPosition);
      pointOnX.setZ(trajectoryOrigin.getZ());
      pointOnY.setIncludingFrame(trajectoryOrigin);
      direction.sub(this.finalPosition, this.initialPosition);
      pointOnY.add(direction.getY(), -direction.getX(), 0.0);
      swingFrame.setPoints(trajectoryOrigin, pointOnX, pointOnY);
      swingFrame.update();

      this.initialPosition.changeFrame(swingFrame);
      this.initialVelocity.changeFrame(swingFrame);
      this.finalPosition.changeFrame(swingFrame);
      this.finalVelocity.changeFrame(swingFrame);

      initialPositionArray.set(0, this.initialPosition.getX());
      initialVelocityArray.set(0, this.initialVelocity.getX());
      finalPositionArray.set(0, this.finalPosition.getX());
      finalVelocityArray.set(0, this.finalVelocity.getX());

      initialPositionArray.set(1, this.initialPosition.getZ());
      initialVelocityArray.set(1, this.initialVelocity.getZ());
      finalPositionArray.set(1, this.finalPosition.getZ());
      finalVelocityArray.set(1, this.finalVelocity.getZ());

      optimizer.setEndPoints(initialPositionArray, initialVelocityArray, finalPositionArray, finalVelocityArray);
   }

   @Override
   public void setWaypoints(ArrayList<FramePoint> waypointPositions)
   {
      if (waypointPositions.size() > maxWaypoints)
         throw new RuntimeException("Too many waypoints");

      this.waypointPositions.clear();
      coefficients.clear();

      coefficients.add();
      for (int i = 0; i < waypointPositions.size(); i++)
      {
         waypointPosition.setIncludingFrame(waypointPositions.get(i));
         waypointPosition.changeFrame(swingFrame);
         TDoubleArrayList waypoint = this.waypointPositions.add();

         waypoint.set(0, this.waypointPosition.getX());
         waypoint.set(1, this.waypointPosition.getZ());
         coefficients.add();
      }

      optimizer.setWaypoints(this.waypointPositions);
      segments.set(waypointPositions.size() + 1);
   }

   @Override
   public void initialize()
   {
      optimizer.compute(maxIterations.getIntegerValue());

      for (int i = 0; i < segments.getIntegerValue()-1; i++)
         waypointTimes.get(i).set(optimizer.getWaypointTime(i));

      for (int dimension = 0; dimension < dimensions; dimension++)
      {
         optimizer.getPolynomialCoefficients(coefficients, dimension);
         Direction axis;
         if (dimension == 0) axis = Direction.X;
         else if (dimension == 1) axis = Direction.Z;
         else axis = null;

         for (int i = 0; i < segments.getIntegerValue(); i++)
         {
            coefficients.get(i).toArray(tempCoeffs);
            trajectories.get(axis).get(i).setDirectlyReverse(tempCoeffs);
         }
      }

      for (int i = 0; i < segments.getIntegerValue(); i++)
      {
         trajectories.get(Direction.Y).get(i).setConstant(0.0);
      }

      visualize();
   }

   private void visualize()
   {
      if (trajectoryViz == null)
         return;

      for (int i = 0; i < markers; i++)
      {
         double time = (double) i / (double) markers;
         compute(time);
         getPosition(ballPosition);
         trajectoryViz.setBall(ballPosition, i);
      }
   }

   @Override
   public void compute(double time)
   {
      time = MathTools.clamp(time, 0.0, 1.0);
      isDone.set(time == 1.0);

      int activeSegment = 0;
      for (int i = 0; i < segments.getIntegerValue()-1; i++)
      {
         double waypointTime = waypointTimes.get(i).getDoubleValue();
         if (time > waypointTime)
            activeSegment = i+1;
         else
            break;
      }
      this.activeSegment.set(activeSegment);

      tempPos.setToZero(swingFrame);
      tempVel.setToZero(swingFrame);
      tempAcc.setToZero(swingFrame);

      for (int dimension = 0; dimension < Direction.values.length; dimension++)
      {
         Direction axis = Direction.values[dimension];
         YoPolynomial polynomial = trajectories.get(axis).get(activeSegment);
         polynomial.compute(time);
         tempPos.set(axis, polynomial.getPosition());
         tempVel.set(axis, polynomial.getVelocity());
         tempAcc.set(axis, polynomial.getAcceleration());
      }

      tempPos.changeFrame(trajectoryFrame);
      tempVel.changeFrame(trajectoryFrame);
      tempAcc.changeFrame(trajectoryFrame);
      desiredPosition.set(tempPos);
      desiredVelocity.set(tempVel);
      desiredAcceleration.set(tempAcc);
   }

   @Override
   public boolean isDone()
   {
      return isDone.getBooleanValue();
   }

   @Override
   public void getPosition(FramePoint positionToPack)
   {
      desiredPosition.getFrameTupleIncludingFrame(positionToPack);
   }

   @Override
   public void getVelocity(FrameVector velocityToPack)
   {
      desiredVelocity.getFrameTupleIncludingFrame(velocityToPack);
   }

   @Override
   public void getAcceleration(FrameVector accelerationToPack)
   {
      desiredAcceleration.getFrameTupleIncludingFrame(accelerationToPack);
   }

   @Override
   public void getLinearData(FramePoint positionToPack, FrameVector velocityToPack, FrameVector accelerationToPack)
   {
      getPosition(positionToPack);
      getVelocity(velocityToPack);
      getAcceleration(accelerationToPack);
   }

   @Override
   public void informDone()
   {
      desiredPosition.setToZero(true);
      desiredVelocity.setToZero(true);
      desiredAcceleration.setToZero(true);
   }

   @Override
   public void showVisualization()
   {
   }

   @Override
   public void hideVisualization()
   {
      if (trajectoryViz == null)
         return;
      trajectoryViz.hideAll();
   }

   @Override
   public double getMaxSpeed()
   {
      // TODO Auto-generated method stub
      return Double.NaN;
   }

   @Override
   public void computeMaxSpeed()
   {
      // TODO Auto-generated method stub

   }

}