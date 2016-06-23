package us.ihmc.commonWalkingControlModules.trajectories;

import java.util.ArrayList;
import java.util.EnumMap;

import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.IntegerYoVariable;
import us.ihmc.robotics.geometry.Direction;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
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
   private static final int dimensions = 3;
   private static final PolynomialOrder order = PolynomialOrder.ORDER5;
   private static final ReferenceFrame trajectoryFrame = ReferenceFrame.getWorldFrame();

   private final String namePrefix;

   private final TrajectoryPointOptimizer optimizer;
   private final ArrayList<double[]> coefficients = new ArrayList<>();
   private final EnumMap<Direction, ArrayList<YoPolynomial>> trajectories = new EnumMap<>(Direction.class);

   private final FramePoint initialPosition = new FramePoint();
   private final FrameVector initialVelocity = new FrameVector();
   private final FramePoint finalPosition = new FramePoint();
   private final FrameVector finalVelocity = new FrameVector();
   private final FramePoint waypointPosition = new FramePoint();
   private final ArrayList<double[]> waypointPositions = new ArrayList<>();

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

   public PositionOptimizedTrajectoryGenerator(String namePrefix, YoVariableRegistry parentRegistry, YoGraphicsListRegistry graphicsListRegistry)
   {
      this.namePrefix = namePrefix;

      registry = new YoVariableRegistry(namePrefix + "Trajectory");
      optimizer = new TrajectoryPointOptimizer(dimensions, order, registry);
      isDone = new BooleanYoVariable(namePrefix + "IsDone", registry);
      segments = new IntegerYoVariable(namePrefix + "Segments", registry);
      activeSegment = new IntegerYoVariable(namePrefix + "ActiveSegment", registry);

      desiredPosition = new YoFramePoint(namePrefix + "DesiredPosition", trajectoryFrame, registry);
      desiredVelocity = new YoFrameVector(namePrefix + "DesiredVelocity", trajectoryFrame, registry);
      desiredAcceleration = new YoFrameVector(namePrefix + "DesiredAcceleration", trajectoryFrame, registry);

      for (Direction axis : Direction.values)
      {
         ArrayList<YoPolynomial> segments = new ArrayList<>();
         segments.add(new YoPolynomial("", order.getCoefficients(), new YoVariableRegistry("")));
         trajectories.put(axis, segments);
      }
      coefficients.add(new double[order.getCoefficients()]);
      segments.set(1);

      parentRegistry.addChild(registry);

      if (graphicsListRegistry != null)
         trajectoryViz = new BagOfBalls(markers, 0.01, namePrefix + "Trajectory", registry, graphicsListRegistry);
      else
         trajectoryViz = null;
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

      optimizer.setEndPoints(initialPosition.toArray(), initialVelocity.toArray(), finalPosition.toArray(), finalVelocity.toArray());
   }

   public void setWaypoints(ArrayList<FramePoint> waypointPositions)
   {
      this.waypointPositions.clear();
      coefficients.clear();

      coefficients.add(new double[order.getCoefficients()]);
      for (int i = 0; i < waypointPositions.size(); i++)
      {
         waypointPosition.setIncludingFrame(waypointPositions.get(i));
         waypointPosition.changeFrame(trajectoryFrame);
         this.waypointPositions.add(waypointPosition.toArray());
         coefficients.add(new double[order.getCoefficients()]);
      }

      optimizer.setWaypoints(this.waypointPositions);
      segments.set(waypointPositions.size() + 1);

      while (waypointTimes.size() < waypointPositions.size())
         extendBySegment();
   }

   private void extendBySegment()
   {
      int size = waypointTimes.size() + 1;
      for (Direction axis : Direction.values)
         trajectories.get(axis).add(new YoPolynomial("", order.getCoefficients(), new YoVariableRegistry("")));
      waypointTimes.add(new DoubleYoVariable(namePrefix + "WaypointTime" + size, registry));
   }

   @Override
   public void initialize()
   {
      optimizer.compute();

      for (int i = 0; i < segments.getIntegerValue()-1; i++)
         waypointTimes.get(i).set(optimizer.getWaypointTime(i));

      for (int dimension = 0; dimension < Direction.values.length; dimension++)
      {
         optimizer.getPolynomialCoefficients(coefficients, dimension);
         Direction axis = Direction.values[dimension];
         for (int i = 0; i < segments.getIntegerValue(); i++)
            trajectories.get(axis).get(i).setDirectlyReverse(coefficients.get(i));
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
      // TODO Auto-generated method stub

   }

   @Override
   public void hideVisualization()
   {
      // TODO Auto-generated method stub

   }

}
