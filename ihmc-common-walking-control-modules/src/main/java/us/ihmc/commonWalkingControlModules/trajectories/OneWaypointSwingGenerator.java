package us.ihmc.commonWalkingControlModules.trajectories;

import java.util.ArrayList;
import java.util.EnumMap;
import java.util.List;

import us.ihmc.commonWalkingControlModules.trajectories.PositionOptimizedTrajectoryGenerator;
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
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.graphics.YoGraphicPolynomial3D;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.robotics.math.trajectories.PositionTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.YoPolynomial;
import us.ihmc.robotics.math.trajectories.YoPolynomial3D;
import us.ihmc.robotics.math.trajectories.generators.TrajectoryPointOptimizer;
import us.ihmc.robotics.math.trajectories.trajectorypoints.FrameEuclideanTrajectoryPoint;
import us.ihmc.robotics.trajectories.TrajectoryType;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

public class OneWaypointSwingGenerator implements SwingGenerator
{
   private static final int maxTimeIterations = -1; // setting this negative activates continuous updating
   private static final int numberWaypoints = 1;
   private static final double defaultWaypointProportion = 0.5;

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoDouble stepTime;
   private final YoDouble timeIntoStep;
   private final YoBoolean isDone;
   private final YoDouble swingHeight;
   private final YoDouble maxSwingHeight;
   private final YoDouble minSwingHeight;
   private final YoDouble defaultSwingHeight;

   private double waypointProportion;

   private TrajectoryType trajectoryType;
   private final PositionOptimizedTrajectoryGenerator trajectory;

   private final FramePoint3D initialPosition = new FramePoint3D();
   private final FrameVector3D initialVelocity = new FrameVector3D();
   private final FramePoint3D finalPosition = new FramePoint3D();
   private final FrameVector3D finalVelocity = new FrameVector3D();
   private final ArrayList<FramePoint3D> waypointPositions = new ArrayList<>();

   private final FrameVector3D initialVelocityNoTimeDimension = new FrameVector3D();
   private final FrameVector3D finalVelocityNoTimeDimension = new FrameVector3D();
   private final FrameVector3D tempWaypointVelocity = new FrameVector3D();

   private final BagOfBalls waypointViz;

   public OneWaypointSwingGenerator(String namePrefix, double minSwingHeight, double maxSwingHeight, double defaultSwingHeight,
                                    YoVariableRegistry parentRegistry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      YoVariableRegistry registry = new YoVariableRegistry(namePrefix + getClass().getSimpleName());

      stepTime = new YoDouble(namePrefix + "StepTime", registry);
      timeIntoStep = new YoDouble(namePrefix + "TimeIntoStep", registry);
      isDone = new YoBoolean(namePrefix + "IsDone", registry);
      swingHeight = new YoDouble(namePrefix + "SwingHeight", registry);
      swingHeight.set(minSwingHeight);

      this.maxSwingHeight = new YoDouble(namePrefix + "MaxSwingHeight", registry);
      this.maxSwingHeight.set(maxSwingHeight);

      this.minSwingHeight = new YoDouble(namePrefix + "MinSwingHeight", registry);
      this.minSwingHeight.set(minSwingHeight);

      this.defaultSwingHeight = new YoDouble(namePrefix + "DefaultSwingHeight", registry);
      this.defaultSwingHeight.set(defaultSwingHeight);

      waypointProportion = defaultWaypointProportion;

      trajectory = new PositionOptimizedTrajectoryGenerator(namePrefix, registry, yoGraphicsListRegistry, maxTimeIterations, numberWaypoints);


      for (int i = 0; i < numberWaypoints; i++)
         waypointPositions.add(new FramePoint3D());

      if (yoGraphicsListRegistry != null)
         waypointViz = new BagOfBalls(numberWaypoints, 0.02, namePrefix + "Waypoints", YoAppearance.White(), registry, yoGraphicsListRegistry);
      else
         waypointViz = null;

      hideVisualization();

      parentRegistry.addChild(registry);
   }

   @Override
   public void setStepTime(double stepTime)
   {
      this.stepTime.set(stepTime);
   }

   @Override
   public void setInitialConditions(FramePoint3DReadOnly initialPosition, FrameVector3DReadOnly initialVelocity)
   {
      this.initialPosition.setIncludingFrame(initialPosition);
      this.initialVelocity.setIncludingFrame(initialVelocity);
   }

   @Override
   public void setFinalConditions(FramePoint3DReadOnly finalPosition, FrameVector3DReadOnly finalVelocity)
   {
      this.finalPosition.setIncludingFrame(finalPosition);
      this.finalVelocity.setIncludingFrame(finalVelocity);
   }

   @Override
   public void setStanceFootPosition(FramePoint3DReadOnly stanceFootPosition)
   {
   }

   @Override
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
         waypointPositions.get(i).setIncludingFrame(waypoints.get(i));
         waypointPositions.get(i).changeFrame(worldFrame);
      }
   }

   @Override
   public void setSwingHeight(double swingHeight)
   {
      boolean useDefaultSwing = Double.isNaN(swingHeight) || swingHeight <= 0.0;

      if(useDefaultSwing)
         this.swingHeight.set(defaultSwingHeight.getDoubleValue());
      else
         this.swingHeight.set(MathTools.clamp(swingHeight, minSwingHeight.getDoubleValue(), maxSwingHeight.getDoubleValue()));
   }

   @Override
   public void setWaypointProportions(double[] waypointProportions)
   {
      this.waypointProportion = waypointProportions[0];
   }

   public void setWaypointProportion(double waypointProportion)
   {
      this.waypointProportion = waypointProportion;
   }

   @Override
   public void initialize()
   {
      timeIntoStep.set(0.0);
      isDone.set(false);

      initialPosition.changeFrame(worldFrame);
      finalPosition.changeFrame(worldFrame);

      double maxStepZ = Math.max(initialPosition.getZ(), finalPosition.getZ());
      switch (trajectoryType)
      {
      case OBSTACLE_CLEARANCE:
         for (int i = 0; i < numberWaypoints; i++)
         {
            waypointPositions.get(i).interpolate(initialPosition, finalPosition, waypointProportion);
            waypointPositions.get(i).setZ(maxStepZ + swingHeight.getDoubleValue());
         }
         break;
      case DEFAULT:
         for (int i = 0; i < numberWaypoints; i++)
         {
            waypointPositions.get(i).interpolate(initialPosition, finalPosition, waypointProportion);
            waypointPositions.get(i).addZ(swingHeight.getDoubleValue());
         }
         break;
      case CUSTOM:
         break;
      default:
         throw new RuntimeException("Trajectory type not implemented");
      }

      double maxWaypointZ = maxStepZ + maxSwingHeight.getDoubleValue();

      for (int i = 0; i < numberWaypoints; i++)
      {
         waypointPositions.get(i).setZ(Math.min(waypointPositions.get(i).getZ(), maxWaypointZ));
      }

      initialVelocityNoTimeDimension.setIncludingFrame(initialVelocity);
      finalVelocityNoTimeDimension.setIncludingFrame(finalVelocity);

      initialVelocityNoTimeDimension.scale(stepTime.getDoubleValue());
      finalVelocityNoTimeDimension.scale(stepTime.getDoubleValue());

      trajectory.setEndpointConditions(initialPosition, initialVelocityNoTimeDimension, finalPosition, finalVelocityNoTimeDimension);
      trajectory.setWaypoints(waypointPositions);
      trajectory.initialize();

      visualize();
   }

   private void visualize()
   {
      if (waypointViz == null)
         return;

      for (int i = 0; i < numberWaypoints; i++)
         waypointViz.setBall(waypointPositions.get(i), i);
   }

   @Override
   public boolean doOptimizationUpdate()
   {
      return trajectory.doOptimizationUpdate();
   }

   @Override
   public void compute(double time)
   {
      double trajectoryTime = stepTime.getDoubleValue();
      isDone.set(time >= trajectoryTime);

      time = MathTools.clamp(time, 0.0, trajectoryTime);
      timeIntoStep.set(time);

      double percent = time / trajectoryTime;
      trajectory.compute(percent);
   }

   @Override
   public boolean isDone()
   {
      return isDone.getBooleanValue();
   }

   @Override
   public void getPosition(FramePoint3D positionToPack)
   {
      trajectory.getPosition(positionToPack);
   }

   @Override
   public void getVelocity(FrameVector3D velocityToPack)
   {
      trajectory.getVelocity(velocityToPack);
      velocityToPack.scale(1.0 / stepTime.getDoubleValue());
   }

   @Override
   public void getAcceleration(FrameVector3D accelerationToPack)
   {
      trajectory.getAcceleration(accelerationToPack);
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

   @Override
   public void showVisualization()
   {
      trajectory.showVisualization();
   }

   private final FramePoint3D tempPoint3D = new FramePoint3D();
   @Override
   public void hideVisualization()
   {
      waypointViz.hideAll();
      tempPoint3D.setToNaN();
      for (int i = 0; i < numberWaypoints; i++)
         waypointViz.setBall(tempPoint3D, i);
      trajectory.hideVisualization();
   }

   @Override
   public int getNumberOfWaypoints()
   {
      return numberWaypoints;
   }

   @Override
   public void getWaypointData(int waypointIndex, FrameEuclideanTrajectoryPoint waypointDataToPack)
   {
      double waypointTime = stepTime.getDoubleValue() * trajectory.getWaypointTime(waypointIndex);
      trajectory.getWaypointVelocity(waypointIndex, tempWaypointVelocity);
      tempWaypointVelocity.scale(1.0 / stepTime.getDoubleValue());

      waypointDataToPack.setToNaN(worldFrame);
      waypointDataToPack.setTime(waypointTime);
      waypointDataToPack.setPosition(waypointPositions.get(waypointIndex));
      waypointDataToPack.setLinearVelocity(tempWaypointVelocity);
   }
}