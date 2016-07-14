package us.ihmc.commonWalkingControlModules.trajectories;

import java.util.ArrayList;
import java.util.EnumMap;

import gnu.trove.list.array.TDoubleArrayList;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
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
import us.ihmc.robotics.math.trajectories.PositionTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.YoPolynomial;
import us.ihmc.robotics.math.trajectories.waypoints.PolynomialOrder;
import us.ihmc.robotics.math.trajectories.waypoints.TrajectoryPointOptimizer;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.BagOfBalls;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicsListRegistry;

public class PositionOptimizedTrajectoryGenerator implements PositionTrajectoryGenerator
{
   private static final int maxWaypoints = 12;
   private static final int dimensions = 3;
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

   private final int markers = 30;
   private final BagOfBalls trajectoryViz;
   private final FramePoint ballPosition = new FramePoint();

   public PositionOptimizedTrajectoryGenerator(String namePrefix, YoVariableRegistry parentRegistry, YoGraphicsListRegistry graphicsListRegistry,
         int maxIterations)
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

      for (Direction axis : Direction.values)
      {
         initialPositionArray.set(axis.getIndex(), this.initialPosition.get(axis));
         initialVelocityArray.set(axis.getIndex(), this.initialVelocity.get(axis));
         finalPositionArray.set(axis.getIndex(), this.finalPosition.get(axis));
         finalVelocityArray.set(axis.getIndex(), this.finalVelocity.get(axis));
      }

      optimizer.setEndPoints(initialPositionArray, initialVelocityArray, finalPositionArray, finalVelocityArray);
   }

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
         waypointPosition.changeFrame(trajectoryFrame);
         TDoubleArrayList waypoint = this.waypointPositions.add();
         for (Direction axis : Direction.values)
            waypoint.set(axis.getIndex(), this.waypointPosition.get(axis));
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

      for (int dimension = 0; dimension < Direction.values.length; dimension++)
      {
         optimizer.getPolynomialCoefficients(coefficients, dimension);
         Direction axis = Direction.values[dimension];
         for (int i = 0; i < segments.getIntegerValue(); i++)
         {
            coefficients.get(i).toArray(tempCoeffs);
            trajectories.get(axis).get(i).setDirectlyReverse(tempCoeffs);
         }
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
      time = MathTools.clipToMinMax(time, 0.0, 1.0);
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

      for (int dimension = 0; dimension < Direction.values.length; dimension++)
      {
         Direction axis = Direction.values[dimension];
         YoPolynomial polynomial = trajectories.get(axis).get(activeSegment);
         polynomial.compute(time);
         desiredPosition.set(axis, polynomial.getPosition());
         desiredVelocity.set(axis, polynomial.getVelocity());
         desiredAcceleration.set(axis, polynomial.getAcceleration());
      }
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
   }

}
