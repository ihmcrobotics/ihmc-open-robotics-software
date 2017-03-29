package us.ihmc.commonWalkingControlModules.trajectories;

import java.util.ArrayList;

import us.ihmc.commons.PrintTools;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.BagOfBalls;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.lists.RecyclingArrayList;
import us.ihmc.robotics.math.trajectories.PositionTrajectoryGenerator;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.trajectories.TrajectoryType;

public class TwoWaypointSwingGenerator implements PositionTrajectoryGenerator
{
   private static final int maxTimeIterations = -1; // setting this negative activates continuous updating
   private static final int numberWaypoints = 2;
   private static final double[] waypointProportions = new double[] {0.15, 0.85};

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoVariableRegistry registry;

   private final DoubleYoVariable stepTime;
   private final DoubleYoVariable timeIntoStep;
   private final BooleanYoVariable isDone;
   private final EnumYoVariable<TrajectoryType> trajectoryType;
   private final DoubleYoVariable swingHeight;
   private final DoubleYoVariable maxSwingHeight;
   private final DoubleYoVariable minSwingHeight;

   private final WaypointTrajectoryGenerator trajectory;

   private final FramePoint initialPosition = new FramePoint();
   private final FrameVector initialVelocity = new FrameVector();
   private final FramePoint finalPosition = new FramePoint();
   private final FrameVector finalVelocity = new FrameVector();
   private final ArrayList<FramePoint> waypointPositions = new ArrayList<>();
   private final FramePoint stanceFootPosition = new FramePoint();

   private final BagOfBalls waypointViz;

   public TwoWaypointSwingGenerator(String namePrefix, double minSwingHeight, double maxSwingHeight, YoVariableRegistry parentRegistry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      registry = new YoVariableRegistry(namePrefix + getClass().getSimpleName());
      parentRegistry.addChild(registry);

      stepTime = new DoubleYoVariable(namePrefix + "StepTime", registry);
      timeIntoStep = new DoubleYoVariable(namePrefix + "TimeIntoStep", registry);
      isDone = new BooleanYoVariable(namePrefix + "IsDone", registry);
      trajectoryType = new EnumYoVariable<>(namePrefix + "TrajectoryType", registry, TrajectoryType.class);
      swingHeight = new DoubleYoVariable(namePrefix + "SwingHeight", registry);
      swingHeight.set(minSwingHeight);

      this.maxSwingHeight = new DoubleYoVariable(namePrefix + "MaxSwingHeight", registry);
      this.maxSwingHeight.set(maxSwingHeight);

      this.minSwingHeight = new DoubleYoVariable(namePrefix + "MinSwingHeight", registry);
      this.minSwingHeight.set(minSwingHeight);

      trajectory = new PositionOptimizedTrajectoryGenerator(namePrefix, registry, yoGraphicsListRegistry, maxTimeIterations, numberWaypoints);

      for (int i = 0; i < numberWaypoints; i++)
         waypointPositions.add(new FramePoint());

      if (yoGraphicsListRegistry != null)
         waypointViz = new BagOfBalls(numberWaypoints, 0.02, namePrefix + "Waypoints", YoAppearance.White(), registry, yoGraphicsListRegistry);
      else
         waypointViz = null;
   }

   public void setStepTime(double stepTime)
   {
      this.stepTime.set(stepTime);
   }

   public void setMinimumSwingHeight(double minSwingHeight)
   {
//      this.minSwingHeight = minSwingHeight;
   }

   public void setInitialConditions(FramePoint initialPosition, FrameVector initialVelocity)
   {
      this.initialPosition.setIncludingFrame(initialPosition);
      this.initialVelocity.setIncludingFrame(initialVelocity);
   }

   public void setFinalConditions(FramePoint finalPosition, FrameVector finalVelocity)
   {
      this.finalPosition.setIncludingFrame(finalPosition);
      this.finalVelocity.setIncludingFrame(finalVelocity);
   }

   public void setTrajectoryType(TrajectoryType trajectoryType)
   {
      setTrajectoryType(trajectoryType, null);
   }

   public void setTrajectoryType(TrajectoryType trajectoryType, RecyclingArrayList<FramePoint> waypoints)
   {
      if (trajectoryType == TrajectoryType.CUSTOM && waypoints == null)
      {
         PrintTools.warn("Recieved no waypoints but trajectory type is custom. Using desault trajectory.");
         this.trajectoryType.set(TrajectoryType.DEFAULT);
      }
      else if (trajectoryType == TrajectoryType.CUSTOM && waypoints.size() != numberWaypoints)
      {
         PrintTools.warn("Recieved too many waypoints. Using default trajectory.");
         this.trajectoryType.set(TrajectoryType.DEFAULT);
      }
      else
         this.trajectoryType.set(trajectoryType);

      if (this.trajectoryType.getEnumValue() != TrajectoryType.CUSTOM)
         return;

      for (int i = 0; i < numberWaypoints; i++)
      {
         waypointPositions.get(i).setIncludingFrame(waypoints.get(i));
         waypointPositions.get(i).changeFrame(worldFrame);
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

   public void setStanceFootPosition(FramePoint stanceFootPosition)
   {
      this.stanceFootPosition.setIncludingFrame(stanceFootPosition);
   }

   public void informDone()
   {
      trajectory.informDone();
   }

   @Override
   public void initialize()
   {
      timeIntoStep.set(0.0);
      isDone.set(false);

      initialPosition.changeFrame(worldFrame);
      finalPosition.changeFrame(worldFrame);
      double maxStepZ = Math.max(initialPosition.getZ(), finalPosition.getZ());

      switch (trajectoryType.getEnumValue())
      {
      case OBSTACLE_CLEARANCE:
         for (int i = 0; i < numberWaypoints; i++)
         {
            waypointPositions.get(i).interpolate(initialPosition, finalPosition, waypointProportions[i]);
            waypointPositions.get(i).setZ(maxStepZ + swingHeight.getDoubleValue());
         }
         break;
      case DEFAULT:
         for (int i = 0; i < numberWaypoints; i++)
         {
            waypointPositions.get(i).interpolate(initialPosition, finalPosition, waypointProportions[i]);
            waypointPositions.get(i).add(0.0, 0.0, swingHeight.getDoubleValue());
         }
         break;
      case CUSTOM:
         break;
      default:
         throw new RuntimeException("Trajectory type not implemented");
      }

      stanceFootPosition.changeFrame(worldFrame);
      double maxWaypointZ = Math.max(stanceFootPosition.getZ() + maxSwingHeight.getDoubleValue(), maxStepZ + minSwingHeight.getDoubleValue());
      for (int i = 0; i < numberWaypoints; i++)
      {
         waypointPositions.get(i).setZ(Math.min(waypointPositions.get(i).getZ(), maxWaypointZ));
      }

      trajectory.setEndpointConditions(initialPosition, initialVelocity, finalPosition, finalVelocity);
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
   public void getPosition(FramePoint positionToPack)
   {
      trajectory.getPosition(positionToPack);
   }

   @Override
   public void getVelocity(FrameVector velocityToPack)
   {
      trajectory.getVelocity(velocityToPack);
      velocityToPack.scale(1.0 / stepTime.getDoubleValue());
   }

   @Override
   public void getAcceleration(FrameVector accelerationToPack)
   {
      trajectory.getAcceleration(accelerationToPack);
      accelerationToPack.scale(1.0 / stepTime.getDoubleValue());
      accelerationToPack.scale(1.0 / stepTime.getDoubleValue());
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
      trajectory.showVisualization();
   }

   @Override
   public void hideVisualization()
   {
      trajectory.hideVisualization();
   }

   public static double[] getDefaultWaypointProportions()
   {
      return waypointProportions;
   }

   public double computeAndGetMaxSpeed()
   {
      trajectory.computeMaxSpeed();
      return trajectory.getMaxSpeed();
   }
}