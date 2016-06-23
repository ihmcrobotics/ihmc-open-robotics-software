package us.ihmc.commonWalkingControlModules.trajectories;

import java.util.ArrayList;

import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
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
import us.ihmc.robotics.trajectories.TwoWaypointTrajectoryGeneratorParameters;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.BagOfBalls;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicsListRegistry;

public class TwoWaypointSwingGenerator implements PositionTrajectoryGenerator
{
   private static final int numberWaypoints = 2;
   private static final int numberIntervals = numberWaypoints + 1;
   private static final PolynomialOrder order = PolynomialOrder.ORDER5;
   private static final double defaultSwingHeight = 0.1;
   private static final double minSwingHeight = 0.1;

   private final YoVariableRegistry registry;

   private final ReferenceFrame trajectoryFrame;

   private final DoubleYoVariable stepTime;
   private final DoubleYoVariable timeIntoStep;
   private final BooleanYoVariable isDone;
   private final EnumYoVariable<TrajectoryType> trajectoryType;
   private final DoubleYoVariable swingHeight;

   private final FramePoint initialPosition = new FramePoint();
   private final FrameVector initialVelocity = new FrameVector();
   private final FramePoint finalPosition = new FramePoint();
   private final FrameVector finalVelocity = new FrameVector();
   private final ArrayList<FramePoint> waypointPositions = new ArrayList<>();

   private final int markers = 30;
   private final BagOfBalls trajectoryViz;
   private final BagOfBalls waypointViz;
   private final FramePoint ballPosition = new FramePoint();

   private final TrajectoryPointOptimizer trajectoryPointOptimizer;
   private final ArrayList<double[]> waypoints = new ArrayList<>();
   private final double[] times = new double[numberWaypoints];
   private final ArrayList<double[]> coefficients = new ArrayList<>();

   private final ArrayList<YoPolynomial> xTrajectory = new ArrayList<>();
   private final ArrayList<YoPolynomial> yTrajectory = new ArrayList<>();
   private final ArrayList<YoPolynomial> zTrajectory = new ArrayList<>();
   private final ArrayList<DoubleYoVariable> trajectoryTimes = new ArrayList<>();
   private final IntegerYoVariable index;

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
      swingHeight = new DoubleYoVariable(namePrefix + "SwingHeight", registry);
      swingHeight.set(defaultSwingHeight);

      trajectoryPointOptimizer = new TrajectoryPointOptimizer(3, order, new YoVariableRegistry(""));
      for (int i = 0; i < numberWaypoints; i++)
      {
         waypoints.add(new double[3]);
         waypointPositions.add(new FramePoint());
      }

      for (int i = 0; i < numberIntervals; i++)
      {
         coefficients.add(new double[order.getCoefficients()]);
         xTrajectory.add(new YoPolynomial(namePrefix + "X" + i, order.getCoefficients(), new YoVariableRegistry("")));
         yTrajectory.add(new YoPolynomial(namePrefix + "Y" + i, order.getCoefficients(), new YoVariableRegistry("")));
         zTrajectory.add(new YoPolynomial(namePrefix + "Z" + i, order.getCoefficients(), new YoVariableRegistry("")));
         trajectoryTimes.add(new DoubleYoVariable(namePrefix + "Time" + i, registry));
      }

      index = new IntegerYoVariable(namePrefix + "TrajectoryIndex", registry);

      desiredPosition = new YoFramePoint(namePrefix + "DesiredPosition", trajectoryFrame, registry);
      desiredVelocity = new YoFrameVector(namePrefix + "DesiredVelocity", trajectoryFrame, registry);
      desiredAcceleration = new YoFrameVector(namePrefix + "DesiredAcceleration", trajectoryFrame, registry);

      if (yoGraphicsListRegistry != null)
      {
         trajectoryViz = new BagOfBalls(markers, 0.01, namePrefix + "Trajectory", registry, yoGraphicsListRegistry);
         waypointViz = new BagOfBalls(numberWaypoints, 0.02, namePrefix + "Waypoints", YoAppearance.White(), registry, yoGraphicsListRegistry);
      }
      else
      {
         trajectoryViz = null;
         waypointViz = null;
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

   public void setSwingHeight(double swingHeight)
   {
      if (Double.isNaN(swingHeight))
         this.swingHeight.set(defaultSwingHeight);
      else if (swingHeight < minSwingHeight)
         this.swingHeight.set(minSwingHeight);
      else
         this.swingHeight.set(swingHeight);
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
      double[] proportions;

      switch (trajectoryType.getEnumValue())
      {
      case OBSTACLE_CLEARANCE:
         double maxZ = Math.max(initialPosition.getZ(), finalPosition.getZ());
         proportions = TwoWaypointTrajectoryGeneratorParameters.getStepOnOrOffProportionsThroughTrajectoryForGroundClearance();
         for (int i = 0; i < numberWaypoints; i++)
         {
            waypointPositions.get(i).interpolate(initialPosition, finalPosition, proportions[i]);
            waypointPositions.get(i).setZ(maxZ + swingHeight.getDoubleValue());
            waypoints.set(i, waypointPositions.get(i).toArray());
         }
         break;
      case PUSH_RECOVERY:
         proportions = TwoWaypointTrajectoryGeneratorParameters.getPushRecoveryProportionsThroughTrajectoryForGroundClearance();
         for (int i = 0; i < numberWaypoints; i++)
         {
            waypointPositions.get(i).interpolate(initialPosition, finalPosition, proportions[i]);
            waypointPositions.get(i).add(0.0, 0.0, swingHeight.getDoubleValue());
            waypoints.set(i, waypointPositions.get(i).toArray());
         }
         break;
      case BASIC:
      case DEFAULT:
         proportions = TwoWaypointTrajectoryGeneratorParameters.getDefaultProportionsThroughTrajectoryForGroundClearance();
         for (int i = 0; i < numberWaypoints; i++)
         {
            waypointPositions.get(i).interpolate(initialPosition, finalPosition, proportions[i]);
            waypointPositions.get(i).add(0.0, 0.0, swingHeight.getDoubleValue());
            waypoints.set(i, waypointPositions.get(i).toArray());
         }
         break;
      default:
         throw new RuntimeException("Trajectory type not implemented");
      }

      trajectoryPointOptimizer.setEndPoints(initialPosition.toArray(), initialVelocity.toArray(), finalPosition.toArray(), finalVelocity.toArray());
      trajectoryPointOptimizer.setWaypoints(waypoints);
      trajectoryPointOptimizer.compute();

      trajectoryPointOptimizer.getWaypointTimes(times);
      for (int i = 0; i < numberWaypoints; i++)
         trajectoryTimes.get(i).set(times[i]);
      trajectoryTimes.get(2).set(1.0);

      trajectoryPointOptimizer.getPolynomialCoefficients(coefficients, 0);
      for (int i = 0; i < numberIntervals; i++)
         xTrajectory.get(i).setDirectlyReverse(coefficients.get(i));
      trajectoryPointOptimizer.getPolynomialCoefficients(coefficients, 1);
      for (int i = 0; i < numberIntervals; i++)
         yTrajectory.get(i).setDirectlyReverse(coefficients.get(i));
      trajectoryPointOptimizer.getPolynomialCoefficients(coefficients, 2);
      for (int i = 0; i < numberIntervals; i++)
         zTrajectory.get(i).setDirectlyReverse(coefficients.get(i));

      visualize();
   }

   private void visualize()
   {
      if (trajectoryViz == null || waypointViz == null)
         return;

      for (int i = 0; i < markers; i++)
      {
         double percent = (double) i / (double) markers;
         double time = percent * stepTime.getDoubleValue();
         compute(time);
         getPosition(ballPosition);
         trajectoryViz.setBall(ballPosition, i);
      }

      for (int i = 0; i < numberWaypoints; i++)
      {
         waypointViz.setBall(waypointPositions.get(i), i);
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


