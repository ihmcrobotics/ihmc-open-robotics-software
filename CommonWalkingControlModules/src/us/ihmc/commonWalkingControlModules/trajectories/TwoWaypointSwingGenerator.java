package us.ihmc.commonWalkingControlModules.trajectories;

import java.util.ArrayList;

import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.dataStructures.variable.IntegerYoVariable;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.math.trajectories.PositionTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.YoPolynomial;
import us.ihmc.robotics.math.trajectories.waypoints.TrajectoryPointOptimizer;
import us.ihmc.robotics.math.trajectories.waypoints.TrajectoryPointOptimizer.PolynomialOrder;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.trajectories.TrajectoryType;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.BagOfBalls;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicsListRegistry;

public class TwoWaypointSwingGenerator implements PositionTrajectoryGenerator
{

   private final YoVariableRegistry registry;

   private final ReferenceFrame trajectoryFrame;

   private final DoubleYoVariable stepTime;
   private final DoubleYoVariable timeIntoStep;
   private final BooleanYoVariable isDone;
   private final EnumYoVariable<TrajectoryType> trajectoryType;

   private final FramePoint initialPosition = new FramePoint();
   private final FrameVector initialVelocity = new FrameVector();
   private final FramePoint finalPosition = new FramePoint();
   private final FrameVector finalVelocity = new FrameVector();
   private final ArrayList<double[]> waypoints = new ArrayList<>();
   private final FramePoint waypointAPosition = new FramePoint();
   private final FramePoint waypointBPosition = new FramePoint();

   private final int markers = 20;
   private final BagOfBalls trajectoryViz;
   private final FramePoint ballPosition = new FramePoint();

   private final TrajectoryPointOptimizer trajectoryPointOptimizer;
   private final ArrayList<YoPolynomial> xTrajectory = new ArrayList<>();
   private final ArrayList<YoPolynomial> yTrajectory = new ArrayList<>();
   private final ArrayList<YoPolynomial> zTrajectory = new ArrayList<>();
   private final ArrayList<DoubleYoVariable> trajectoryTimes = new ArrayList<>();
   private final IntegerYoVariable index;
   private final double[] times = new double[2];
   private final ArrayList<double[]> coefficients = new ArrayList<>();

   private final YoFramePoint desiredPosition;
   private final YoFrameVector desiredVelocity;
   private final YoFrameVector desiredAcceleration;

   public TwoWaypointSwingGenerator(String namePrefix, ReferenceFrame trajectoryFrame, YoVariableRegistry parentRegistry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      registry = new YoVariableRegistry(namePrefix + getClass().getSimpleName());
      parentRegistry.addChild(registry);

      this.trajectoryFrame = trajectoryFrame;

      stepTime = new DoubleYoVariable(namePrefix + "StepTime", registry);
      timeIntoStep = new DoubleYoVariable(namePrefix + "TimeIntoStep", registry);
      isDone = new BooleanYoVariable(namePrefix + "IsDone", registry);
      trajectoryType = new EnumYoVariable<>(namePrefix + "TrajectoryType", registry, TrajectoryType.class);

      PolynomialOrder order = PolynomialOrder.ORDER5;
      trajectoryPointOptimizer = new TrajectoryPointOptimizer(3, order, new YoVariableRegistry(""));
      waypoints.add(new double[3]);
      waypoints.add(new double[3]);
      coefficients.add(new double[order.getCoefficients()]);
      coefficients.add(new double[order.getCoefficients()]);
      coefficients.add(new double[order.getCoefficients()]);

      xTrajectory.add(new YoPolynomial(namePrefix + "X1", order.getCoefficients(), new YoVariableRegistry("")));
      xTrajectory.add(new YoPolynomial(namePrefix + "X2", order.getCoefficients(), new YoVariableRegistry("")));
      xTrajectory.add(new YoPolynomial(namePrefix + "X3", order.getCoefficients(), new YoVariableRegistry("")));
      yTrajectory.add(new YoPolynomial(namePrefix + "Y1", order.getCoefficients(), new YoVariableRegistry("")));
      yTrajectory.add(new YoPolynomial(namePrefix + "Y2", order.getCoefficients(), new YoVariableRegistry("")));
      yTrajectory.add(new YoPolynomial(namePrefix + "Y3", order.getCoefficients(), new YoVariableRegistry("")));
      zTrajectory.add(new YoPolynomial(namePrefix + "Z1", order.getCoefficients(), new YoVariableRegistry("")));
      zTrajectory.add(new YoPolynomial(namePrefix + "Z2", order.getCoefficients(), new YoVariableRegistry("")));
      zTrajectory.add(new YoPolynomial(namePrefix + "Z3", order.getCoefficients(), new YoVariableRegistry("")));
      trajectoryTimes.add(new DoubleYoVariable(namePrefix + "Time1", registry));
      trajectoryTimes.add(new DoubleYoVariable(namePrefix + "Time2", registry));
      trajectoryTimes.add(new DoubleYoVariable(namePrefix + "Time3", registry));
      index = new IntegerYoVariable(namePrefix + "TrajectoryIndex", registry);

      desiredPosition = new YoFramePoint(namePrefix + "DesiredPosition", trajectoryFrame, registry);
      desiredVelocity = new YoFrameVector(namePrefix + "DesiredVelocity", trajectoryFrame, registry);
      desiredAcceleration = new YoFrameVector(namePrefix + "DesiredAcceleration", trajectoryFrame, registry);

      if (yoGraphicsListRegistry != null)
      {
         trajectoryViz = new BagOfBalls(markers, 0.01, namePrefix + "Trajectory", registry, yoGraphicsListRegistry);
      }
      else
      {
         trajectoryViz = null;
      }
   }

   public void setStepTime(double stepTime)
   {
      this.stepTime.set(stepTime);
   }

   public void setInitialConditions(FramePoint initialPosition, FrameVector initialVelocity)
   {
      this.initialPosition.setIncludingFrame(initialPosition);
      this.initialVelocity.setIncludingFrame(initialVelocity);
      this.initialPosition.changeFrame(trajectoryFrame);
      this.initialVelocity.changeFrame(trajectoryFrame);
   }

   public void setFinalConditions(FramePoint finalPosition, FrameVector finalVelocity)
   {
      this.finalPosition.setIncludingFrame(finalPosition);
      this.finalVelocity.setIncludingFrame(finalVelocity);
      this.finalPosition.changeFrame(trajectoryFrame);
      this.finalVelocity.changeFrame(trajectoryFrame);
   }

   public void setTrajectoryType(TrajectoryType trajectoryType)
   {
      this.trajectoryType.set(trajectoryType);
   }

   public void informDone()
   {
      desiredPosition.setToZero(true);
      desiredVelocity.setToZero(true);
      desiredAcceleration.setToZero(true);
   }

   @Override
   public void initialize()
   {
      timeIntoStep.set(0.0);
      isDone.set(false);

      // TODO: get this from parameters and do checks on swing height
      double swingHeight = 0.1;
      double[] proportions = {0.15, 0.85};

      waypointAPosition.interpolate(initialPosition, finalPosition, proportions[0]);
      waypointBPosition.interpolate(initialPosition, finalPosition, proportions[1]);

      switch (trajectoryType.getEnumValue())
      {
      case OBSTACLE_CLEARANCE:
         double maxZ = Math.max(initialPosition.getZ(), finalPosition.getZ());
         waypointAPosition.setZ(maxZ + swingHeight);
         waypointBPosition.setZ(maxZ + swingHeight);
         break;
      case DEFAULT:
         waypointAPosition.add(0.0, 0.0, swingHeight);
         waypointBPosition.add(0.0, 0.0, swingHeight);
         break;
      default:
         throw new RuntimeException("Trajectory type not implemented");
      }

      trajectoryPointOptimizer.setEndPoints(initialPosition.toArray(), initialVelocity.toArray(), finalPosition.toArray(), finalVelocity.toArray());
      waypoints.set(0, waypointAPosition.toArray());
      waypoints.set(1, waypointBPosition.toArray());
      trajectoryPointOptimizer.setWaypoints(waypoints);
      trajectoryPointOptimizer.compute();

      trajectoryPointOptimizer.getWaypointTimes(times);
      trajectoryTimes.get(0).set(times[0]);
      trajectoryTimes.get(1).set(times[1]);
      trajectoryTimes.get(2).set(1.0);

      trajectoryPointOptimizer.getPolynomialCoefficients(coefficients, 0);
      xTrajectory.get(0).setDirectlyReverse(coefficients.get(0));
      xTrajectory.get(1).setDirectlyReverse(coefficients.get(1));
      xTrajectory.get(2).setDirectlyReverse(coefficients.get(2));

      trajectoryPointOptimizer.getPolynomialCoefficients(coefficients, 1);
      yTrajectory.get(0).setDirectlyReverse(coefficients.get(0));
      yTrajectory.get(1).setDirectlyReverse(coefficients.get(1));
      yTrajectory.get(2).setDirectlyReverse(coefficients.get(2));

      trajectoryPointOptimizer.getPolynomialCoefficients(coefficients, 2);
      zTrajectory.get(0).setDirectlyReverse(coefficients.get(0));
      zTrajectory.get(1).setDirectlyReverse(coefficients.get(1));
      zTrajectory.get(2).setDirectlyReverse(coefficients.get(2));

      visualize();
   }

   private void visualize()
   {
      if (trajectoryViz == null)
         return;

      for (int i = 0; i < markers; i++)
      {
         double percent = (double) i / (double) markers;
         double time = percent * stepTime.getDoubleValue();
         compute(time);
         getPosition(ballPosition);
         trajectoryViz.setBall(ballPosition, i);
      }
   }

   @Override
   public void compute(double time)
   {
      double trajectoryTime = stepTime.getDoubleValue();
      isDone.set(time >= trajectoryTime);

      time = MathTools.clipToMinMax(time, 0.0, trajectoryTime);
      timeIntoStep.set(time);

      double percent = time / trajectoryTime;
      int index;
      if (percent < trajectoryTimes.get(0).getDoubleValue())
         index = 0;
      else if (percent < trajectoryTimes.get(1).getDoubleValue())
         index = 1;
      else
         index = 2;

      this.index.set(index);

      YoPolynomial xPoly = xTrajectory.get(index);
      YoPolynomial yPoly = yTrajectory.get(index);
      YoPolynomial zPoly = zTrajectory.get(index);

      xPoly.compute(percent);
      yPoly.compute(percent);
      zPoly.compute(percent);

      desiredPosition.set(xPoly.getPosition(), yPoly.getPosition(), zPoly.getPosition());
      desiredVelocity.set(xPoly.getVelocity(), yPoly.getVelocity(), zPoly.getVelocity());
      desiredAcceleration.set(xPoly.getAcceleration(), yPoly.getAcceleration(), zPoly.getAcceleration());
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


