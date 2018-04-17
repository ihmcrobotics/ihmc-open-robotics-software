package us.ihmc.quadrupedRobotics.planning.trajectory;

import us.ihmc.commons.MathTools;
import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.Axis;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.BagOfBalls;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPolynomial3D;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.lists.RecyclingArrayList;
import us.ihmc.robotics.math.trajectories.PositionTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.YoPolynomial;
import us.ihmc.robotics.math.trajectories.YoPolynomial3D;
import us.ihmc.robotics.math.trajectories.waypoints.FrameEuclideanTrajectoryPoint;
import us.ihmc.robotics.math.trajectories.waypoints.PolynomialOrder;
import us.ihmc.robotics.trajectories.TrajectoryType;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

import java.util.ArrayList;
import java.util.EnumMap;
import java.util.List;

public class OneWaypointSwingGenerator implements PositionTrajectoryGenerator
{
   private static final int numberWaypoints = 1;
   private static final double defaultWaypointProportion = 0.5;

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoDouble stepTime;
   private final YoDouble timeIntoStep;
   private final YoBoolean isDone;
   private final YoDouble swingHeight;
   private final YoDouble maxSwingHeight;
   private final YoDouble minSwingHeight;

   public static final PolynomialOrder order = PolynomialOrder.ORDER3;
   private final double[] tempCoeffs = new double[order.getCoefficients()];

   private final YoInteger segments;
   private final YoInteger activeSegment;

   private final DoubleProvider waypointProportion;

   private TrajectoryType trajectoryType;

   private final FramePoint3D initialPosition = new FramePoint3D();
   private final FrameVector3D initialVelocity = new FrameVector3D();
   private final FramePoint3D midpointPosition = new FramePoint3D();
   private final FrameVector3D midpointVelocity = new FrameVector3D();
   private final FramePoint3D finalPosition = new FramePoint3D();
   private final FrameVector3D finalVelocity = new FrameVector3D();

   private final FramePoint3D desiredPosition = new FramePoint3D();
   private final FrameVector3D desiredVelocity = new FrameVector3D();
   private final FrameVector3D desiredAcceleration = new FrameVector3D();

   private final FrameVector3D initialVelocityNoTimeDimension = new FrameVector3D();
   private final FrameVector3D midpointVelocityNoTimeDimension = new FrameVector3D();
   private final FrameVector3D finalVelocityNoTimeDimension = new FrameVector3D();

   private final ArrayList<YoDouble> waypointTimes = new ArrayList<>();
   private final EnumMap<Axis, ArrayList<YoPolynomial>> trajectories = new EnumMap<>(Axis.class);

   private final BagOfBalls waypointViz;
   private final YoGraphicPolynomial3D trajectoryViz;

   public OneWaypointSwingGenerator(String namePrefix, double minSwingHeight, double maxSwingHeight, YoVariableRegistry parentRegistry,
                                    YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this(namePrefix, Double.NaN, minSwingHeight, maxSwingHeight, parentRegistry, yoGraphicsListRegistry);
   }

   public OneWaypointSwingGenerator(String namePrefix, double waypointProportion, double minSwingHeight, double maxSwingHeight,
                                    YoVariableRegistry parentRegistry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      YoVariableRegistry registry = new YoVariableRegistry(namePrefix + getClass().getSimpleName());

      stepTime = new YoDouble(namePrefix + "StepTime", registry);
      timeIntoStep = new YoDouble(namePrefix + "TimeIntoStep", registry);
      isDone = new YoBoolean(namePrefix + "IsDone", registry);
      swingHeight = new YoDouble(namePrefix + "SwingHeight", registry);
      swingHeight.set(minSwingHeight);

      segments = new YoInteger(namePrefix + "Segments", registry);
      activeSegment = new YoInteger(namePrefix + "ActiveSegment", registry);

      this.maxSwingHeight = new YoDouble(namePrefix + "MaxSwingHeight", registry);
      this.maxSwingHeight.set(maxSwingHeight);

      this.minSwingHeight = new YoDouble(namePrefix + "MinSwingHeight", registry);
      this.minSwingHeight.set(minSwingHeight);

      if (!Double.isFinite(waypointProportion))
         waypointProportion = defaultWaypointProportion;

      this.waypointProportion = new DoubleParameter(namePrefix + "WaypointProportion", registry, waypointProportion);

      for (Axis axis : Axis.values)
         trajectories.put(axis, new ArrayList<>());

      while (waypointTimes.size() <= numberWaypoints)
         extendBySegment(namePrefix, registry);

      segments.set(1);

      if (yoGraphicsListRegistry != null)
      {
         List<YoPolynomial3D> yoPolynomial3Ds = YoPolynomial3D
               .createYoPolynomial3DList(trajectories.get(Axis.X), trajectories.get(Axis.Y), trajectories.get(Axis.Z));
         trajectoryViz = new YoGraphicPolynomial3D(namePrefix + "Trajectory", null, yoPolynomial3Ds, waypointTimes, 0.01, 50, 8, registry);
         yoGraphicsListRegistry.registerYoGraphic(namePrefix + "Trajectory", trajectoryViz);

         trajectoryViz.setColorType(YoGraphicPolynomial3D.TrajectoryColorType.ACCELERATION_BASED);

         waypointViz = new BagOfBalls(numberWaypoints, 0.02, namePrefix + "Waypoints", YoAppearance.White(), registry, yoGraphicsListRegistry);
      }
      else
      {
         trajectoryViz = null;
         waypointViz = null;
      }

      hideVisualization();

      parentRegistry.addChild(registry);
   }

   private void extendBySegment(String namePrefix, YoVariableRegistry registry)
   {
      int size = waypointTimes.size() + 1;
      for (Axis axis : Axis.values)
         trajectories.get(axis).add(new YoPolynomial(namePrefix + "Segment" + size + "Axis" + axis.ordinal(), order.getCoefficients(), registry));
      waypointTimes.add(new YoDouble(namePrefix + "WaypointTime" + size, registry));
   }

   public void setStepTime(double stepTime)
   {
      this.stepTime.set(stepTime);
   }

   public void setInitialConditions(FramePoint3DReadOnly initialPosition, FrameVector3DReadOnly initialVelocity)
   {
      this.initialPosition.setIncludingFrame(initialPosition);
      this.initialVelocity.setIncludingFrame(initialVelocity);
   }

   public void setFinalConditions(FramePoint3DReadOnly finalPosition, FrameVector3DReadOnly finalVelocity)
   {
      this.finalPosition.setIncludingFrame(finalPosition);
      this.finalVelocity.setIncludingFrame(finalVelocity);
   }

   public void setTrajectoryType(TrajectoryType trajectoryType)
   {
      setTrajectoryType(trajectoryType, null);
   }

   public void setTrajectoryType(TrajectoryType trajectoryType, RecyclingArrayList<FramePoint3D> waypoints)
   {
      if (trajectoryType == TrajectoryType.CUSTOM && waypoints == null)
      {
         PrintTools.warn("Received no waypoints but trajectory type is custom. Using default trajectory.");
         this.trajectoryType = TrajectoryType.DEFAULT;
      }
      else if (trajectoryType == TrajectoryType.CUSTOM && waypoints.size() != numberWaypoints)
      {
         PrintTools.warn("Received unexpected amount of waypoints. Using default trajectory.");
         this.trajectoryType = TrajectoryType.DEFAULT;
      }
      else
      {
         this.trajectoryType = trajectoryType;
      }

      if (this.trajectoryType != TrajectoryType.CUSTOM)
         return;

      for (int i = 0; i < numberWaypoints; i++)
      {
         midpointPosition.setIncludingFrame(waypoints.get(i));
         midpointPosition.changeFrame(worldFrame);
      }
   }

   public void setSwingHeight(double swingHeight)
   {
      if (Double.isNaN(swingHeight))
         this.swingHeight.set(minSwingHeight.getDoubleValue());
      else if (swingHeight < minSwingHeight.getDoubleValue())
         this.swingHeight.set(minSwingHeight.getDoubleValue());
      else if (swingHeight > maxSwingHeight.getDoubleValue())
         this.swingHeight.set(maxSwingHeight.getDoubleValue());
      else
         this.swingHeight.set(swingHeight);
   }

   @Override
   public void initialize()
   {
      timeIntoStep.set(0.0);
      isDone.set(false);

      initialPosition.changeFrame(worldFrame);
      finalPosition.changeFrame(worldFrame);

      waypointTimes.get(0).set(waypointProportion.getValue());
      waypointTimes.get(1).set(1.0);

      midpointPosition.interpolate(initialPosition, finalPosition, waypointProportion.getValue());

      computeMidpointXY();

      double maxStepZ = Math.max(initialPosition.getZ(), finalPosition.getZ());

      switch (trajectoryType)
      {
      case OBSTACLE_CLEARANCE:
         for (int i = 0; i < numberWaypoints; i++)
         {
            midpointPosition.setZ(maxStepZ + swingHeight.getDoubleValue());
         }
         break;
      case DEFAULT:
         for (int i = 0; i < numberWaypoints; i++)
         {
            midpointPosition.addZ(swingHeight.getDoubleValue());
         }
         break;
      case CUSTOM:
         break;
      default:
         throw new RuntimeException("Trajectory type not implemented");
      }

      initialVelocityNoTimeDimension.setIncludingFrame(initialVelocity);
      finalVelocityNoTimeDimension.setIncludingFrame(finalVelocity);

      initialVelocityNoTimeDimension.scale(stepTime.getDoubleValue());
      finalVelocityNoTimeDimension.scale(stepTime.getDoubleValue());

      setupTrajectories();
      visualize();
   }

   private void computeMidpointXY()
   {
      // x
      computeCoefficients(initialPosition.getX(), initialVelocityNoTimeDimension.getX(), finalPosition.getX(), finalVelocityNoTimeDimension.getX());

      double xPosition = getPositionFromCoefficients(waypointProportion.getValue());
      double xVelocity = getVelocityNoTimeDimensionFromCoefficients(waypointProportion.getValue());

      // y
      computeCoefficients(initialPosition.getY(), initialVelocityNoTimeDimension.getY(), finalPosition.getY(), finalVelocityNoTimeDimension.getY());

      double yPosition = getPositionFromCoefficients(waypointProportion.getValue());
      double yVelocity = getVelocityNoTimeDimensionFromCoefficients(waypointProportion.getValue());

      midpointPosition.setX(xPosition);
      midpointPosition.setY(yPosition);

      midpointVelocityNoTimeDimension.set(xVelocity, yVelocity, 0.0);
      midpointVelocity.set(midpointVelocityNoTimeDimension);
      midpointVelocity.scale(1.0 / stepTime.getDoubleValue());
   }

   private void computeCoefficients(double x0, double xd0, double xf, double xdf)
   {
      tempCoeffs[0] = x0;
      tempCoeffs[1] = xd0;
      tempCoeffs[2] = 3 * (xf - x0) - (xdf + 2 * xd0);
      tempCoeffs[3] = 2 * (x0 - xf) + (xd0 + xdf);
   }

   private double getPositionFromCoefficients(double percent)
   {
      return tempCoeffs[0] + tempCoeffs[1] * percent + tempCoeffs[2] * percent * percent + tempCoeffs[3] * Math.pow(percent, 3.0);
   }

   private double getVelocityNoTimeDimensionFromCoefficients(double percent)
   {
      return tempCoeffs[1] + 2.0 * tempCoeffs[2] * percent + 3.0 * tempCoeffs[3] * percent * percent;
   }

   private void setupTrajectories()
   {
      trajectories.get(Axis.X).get(0)
                  .setCubic(0.0, waypointProportion.getValue(), initialPosition.getX(), initialVelocityNoTimeDimension.getX(), midpointPosition.getX(),
                            midpointVelocityNoTimeDimension.getX());
      trajectories.get(Axis.Y).get(0)
                  .setCubic(0.0, waypointProportion.getValue(), initialPosition.getY(), initialVelocityNoTimeDimension.getY(), midpointPosition.getY(),
                            midpointVelocityNoTimeDimension.getY());
      trajectories.get(Axis.Z).get(0)
                  .setCubic(0.0, waypointProportion.getValue(), initialPosition.getZ(), initialVelocityNoTimeDimension.getZ(), midpointPosition.getZ(),
                            midpointVelocityNoTimeDimension.getZ());

      trajectories.get(Axis.X).get(1)
                  .setCubic(waypointProportion.getValue(), 1.0, midpointPosition.getX(), midpointVelocityNoTimeDimension.getX(), finalPosition.getX(),
                            finalVelocityNoTimeDimension.getX());
      trajectories.get(Axis.Y).get(1)
                  .setCubic(waypointProportion.getValue(), 1.0, midpointPosition.getY(), midpointVelocityNoTimeDimension.getY(), finalPosition.getY(),
                            finalVelocityNoTimeDimension.getY());
      trajectories.get(Axis.Z).get(1)
                  .setCubic(waypointProportion.getValue(), 1.0, midpointPosition.getZ(), midpointVelocityNoTimeDimension.getZ(), finalPosition.getZ(),
                            finalVelocityNoTimeDimension.getZ());
   }

   private void visualize()
   {
      if (waypointViz != null)
      {
         for (int i = 0; i < numberWaypoints; i++)
            waypointViz.setBall(midpointPosition, i);
      }

      if (trajectoryViz != null)
      {
         trajectoryViz.showGraphic();
      }
   }

   @Override
   public void compute(double time)
   {
      double trajectoryTime = stepTime.getDoubleValue();
      isDone.set(time >= trajectoryTime);

      time = MathTools.clamp(time, 0.0, trajectoryTime);
      timeIntoStep.set(time);

      double percent = time / trajectoryTime;

      if (percent < 0.0)
      {
         desiredPosition.set(initialPosition);
         desiredVelocity.setToZero();
         desiredAcceleration.setToZero();
         return;
      }
      if (percent > 1.0)
      {
         desiredPosition.set(finalPosition);
         desiredVelocity.setToZero();
         desiredAcceleration.setToZero();
         return;
      }

      percent = MathTools.clamp(percent, 0.0, 1.0);

      int activeSegment = 0;
      for (int i = 0; i < segments.getIntegerValue() - 1; i++)
      {
         double waypointTime = waypointTimes.get(i).getDoubleValue();
         if (percent > waypointTime)
            activeSegment = i + 1;
         else
            break;
      }
      this.activeSegment.set(activeSegment);

      for (int dimension = 0; dimension < Axis.values.length; dimension++)
      {
         Axis axis = Axis.values[dimension];
         YoPolynomial polynomial = trajectories.get(axis).get(activeSegment);
         polynomial.compute(percent);
         desiredPosition.setElement(dimension, polynomial.getPosition());
         desiredVelocity.setElement(dimension, polynomial.getVelocity());
         desiredAcceleration.setElement(dimension, polynomial.getAcceleration());
      }
   }

   @Override
   public boolean isDone()
   {
      return isDone.getBooleanValue();
   }

   @Override
   public void getPosition(FramePoint3D positionToPack)
   {
      positionToPack.setIncludingFrame(desiredPosition);
   }

   @Override
   public void getVelocity(FrameVector3D velocityToPack)
   {
      velocityToPack.setIncludingFrame(desiredVelocity);
      velocityToPack.scale(1.0 / stepTime.getDoubleValue());
   }

   @Override
   public void getAcceleration(FrameVector3D accelerationToPack)
   {
      accelerationToPack.setIncludingFrame(desiredAcceleration);
      accelerationToPack.scale(1.0 / stepTime.getDoubleValue());
      accelerationToPack.scale(1.0 / stepTime.getDoubleValue());
   }

   @Override
   public void getLinearData(FramePoint3D positionToPack, FrameVector3D velocityToPack, FrameVector3D accelerationToPack)
   {
      getPosition(positionToPack);
      getVelocity(velocityToPack);
      getAcceleration(accelerationToPack);
   }

   public void showVisualization()
   {
      if (trajectoryViz != null)
         trajectoryViz.showGraphic();
   }

   public void hideVisualization()
   {
      if (trajectoryViz != null)
         trajectoryViz.hideGraphic();
      if (waypointViz != null)
         waypointViz.hideAll();
   }

   public int getNumberOfWaypoints()
   {
      return numberWaypoints;
   }

   public void getWaypointData(int waypointIndex, FrameEuclideanTrajectoryPoint waypointDataToPack)
   {
      if (waypointIndex >= numberWaypoints)
         throw new RuntimeException("Too few waypoints for this.");

      double waypointTime = waypointTimes.get(0).getDoubleValue() * stepTime.getDoubleValue();

      waypointDataToPack.setToNaN(worldFrame);
      waypointDataToPack.setTime(waypointTime);
      waypointDataToPack.setPosition(midpointPosition);
      waypointDataToPack.setLinearVelocity(midpointVelocity);
   }
}