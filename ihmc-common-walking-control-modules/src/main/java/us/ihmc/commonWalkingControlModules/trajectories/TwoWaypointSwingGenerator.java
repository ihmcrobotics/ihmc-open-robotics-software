package us.ihmc.commonWalkingControlModules.trajectories;

import java.util.ArrayList;

import us.ihmc.commons.MathTools;
import us.ihmc.commons.PrintTools;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.BagOfBalls;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.math.trajectories.PositionTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.trajectorypoints.FrameEuclideanTrajectoryPoint;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.trajectories.TrajectoryType;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class TwoWaypointSwingGenerator implements SwingGenerator
{
   private static final int maxTimeIterations = -1; // setting this negative activates continuous updating
   private static final int numberWaypoints = 2;
   private static final double[] defaultWaypointProportions = new double[] {0.15, 0.85};

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoVariableRegistry registry;

   private final YoDouble stepTime;
   private final YoDouble timeIntoStep;
   private final YoBoolean isDone;
   private final YoDouble swingHeight;
   private final YoDouble minSwingHeight;
   private final YoDouble maxSwingHeight;
   private final YoDouble defaultSwingHeight;

   private final double[] waypointProportions = new double[2];

   private TrajectoryType trajectoryType;
   private final PositionOptimizedTrajectoryGenerator trajectory;

   private final FramePoint3D initialPosition = new FramePoint3D();
   private final FrameVector3D initialVelocity = new FrameVector3D();
   private final FramePoint3D finalPosition = new FramePoint3D();
   private final FrameVector3D finalVelocity = new FrameVector3D();
   private final ArrayList<FramePoint3D> waypointPositions = new ArrayList<>();
   private final FramePoint3D stanceFootPosition = new FramePoint3D();

   private final FrameVector3D initialVelocityNoTimeDimension = new FrameVector3D();
   private final FrameVector3D finalVelocityNoTimeDiemension = new FrameVector3D();
   private final FrameVector3D tempWaypointVelocity = new FrameVector3D();

   private final FramePoint3D tempPoint3D = new FramePoint3D();

   private final BagOfBalls waypointViz;

   private RobotSide swingSide = null;
   private ReferenceFrame stanceZUpFrame = null;
   private final Vector2D swingOffset = new Vector2D();
   private final YoDouble minDistanceToStance;
   private final YoBoolean needToAdjustedSwingForSelfCollision;
   private final YoBoolean crossOverStep;

   public TwoWaypointSwingGenerator(String namePrefix, double minSwingHeight, double maxSwingHeight, double defaultSwingHeight, YoVariableRegistry parentRegistry,
                                    YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      registry = new YoVariableRegistry(namePrefix + getClass().getSimpleName());
      parentRegistry.addChild(registry);

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

      this.minDistanceToStance = new YoDouble(namePrefix + "MinDistanceToStance", registry);
      this.minDistanceToStance.set(Double.NEGATIVE_INFINITY);

      for (int i = 0; i < numberWaypoints; i++)
         this.waypointProportions[i] = defaultWaypointProportions[i];

      trajectory = new PositionOptimizedTrajectoryGenerator(namePrefix, registry, yoGraphicsListRegistry, maxTimeIterations, numberWaypoints);

      for (int i = 0; i < numberWaypoints; i++)
         waypointPositions.add(new FramePoint3D());

      if (yoGraphicsListRegistry != null)
         waypointViz = new BagOfBalls(numberWaypoints, 0.02, namePrefix + "Waypoints", YoAppearance.White(), registry, yoGraphicsListRegistry);
      else
         waypointViz = null;

      needToAdjustedSwingForSelfCollision = new YoBoolean(namePrefix + "AdjustedSwing", registry);
      crossOverStep = new YoBoolean(namePrefix + "CrossOverStep", registry);
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
   public void setTrajectoryType(TrajectoryType trajectoryType, RecyclingArrayList<FramePoint3D> waypoints)
   {
      if (trajectoryType == TrajectoryType.CUSTOM && waypoints == null)
      {
         PrintTools.warn("Recieved no waypoints but trajectory type is custom. Using default trajectory.");
         this.trajectoryType = TrajectoryType.DEFAULT;
      }
      else if (trajectoryType == TrajectoryType.CUSTOM && waypoints.size() != numberWaypoints)
      {
         PrintTools.warn("Recieved unexpected amount of waypoints. Using default trajectory.");
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
   public void setStanceFootPosition(FramePoint3DReadOnly stanceFootPosition)
   {
      this.stanceFootPosition.setIncludingFrame(stanceFootPosition);
   }

   public void informDone()
   {
      trajectory.informDone();
   }

   @Override
   public void setWaypointProportions(double[] waypointProportions)
   {
      setWaypointProportions(waypointProportions[0], waypointProportions[1]);
   }

   public void setWaypointProportions(double waypointProportions0, double waypointProportions1)
   {
      this.waypointProportions[0] = waypointProportions0;
      this.waypointProportions[1] = waypointProportions1;
   }

   @Override
   public void initialize()
   {
      timeIntoStep.set(0.0);
      isDone.set(false);

      initialPosition.changeFrame(worldFrame);
      finalPosition.changeFrame(worldFrame);
      stanceFootPosition.changeFrame(worldFrame);

      needToAdjustedSwingForSelfCollision.set(computeSwingAdjustment(initialPosition, finalPosition, stanceFootPosition, swingOffset));

      double maxStepZ = Math.max(initialPosition.getZ(), finalPosition.getZ());
      switch (trajectoryType)
      {
      case OBSTACLE_CLEARANCE:
         for (int i = 0; i < numberWaypoints; i++)
         {
            waypointPositions.get(i).interpolate(initialPosition, finalPosition, waypointProportions[i]);           
            waypointPositions.get(i).addZ(swingHeight.getDoubleValue());
            // waypointPositions.get(i).setZ(maxStepZ + swingHeight.getDoubleValue());
            
            if (needToAdjustedSwingForSelfCollision.getBooleanValue())
            {
               waypointPositions.get(i).add(swingOffset.getX(), swingOffset.getY(), 0.0);
            }
         }
         break;
      case DEFAULT:
         for (int i = 0; i < numberWaypoints; i++)
         {
            waypointPositions.get(i).interpolate(initialPosition, finalPosition, waypointProportions[i]);
            waypointPositions.get(i).addZ(swingHeight.getDoubleValue());
            if (needToAdjustedSwingForSelfCollision.getBooleanValue())
            {
               waypointPositions.get(i).add(swingOffset.getX(), swingOffset.getY(), 0.0);
            }
         }
         break;
      case CUSTOM:
         break;
      default:
         throw new RuntimeException("Trajectory type not implemented");
      }

      double maxWaypointZ;
      if (stanceFootPosition.containsNaN())
         maxWaypointZ = maxStepZ + maxSwingHeight.getDoubleValue();
      else
         maxWaypointZ = Math.max(stanceFootPosition.getZ() + maxSwingHeight.getDoubleValue(), maxStepZ + minSwingHeight.getDoubleValue());

      for (int i = 0; i < numberWaypoints; i++)
      {
         waypointPositions.get(i).setZ(Math.min(waypointPositions.get(i).getZ(), maxWaypointZ));
      }

      initialVelocityNoTimeDimension.setIncludingFrame(initialVelocity);
      finalVelocityNoTimeDiemension.setIncludingFrame(finalVelocity);

      initialVelocityNoTimeDimension.scale(stepTime.getDoubleValue());
      finalVelocityNoTimeDiemension.scale(stepTime.getDoubleValue());

      trajectory.setEndpointConditions(initialPosition, initialVelocityNoTimeDimension, finalPosition, finalVelocityNoTimeDiemension);
      trajectory.setWaypoints(waypointPositions);
      trajectory.initialize();

      visualize();
   }

   private final FrameVector2D xyDistanceToStance = new FrameVector2D();
   private final Point2D stance2D = new Point2D();
   private final Point2D pointA2D = new Point2D();
   private final Point2D pointB2D = new Point2D();
   private final FramePoint2D pointAInStance = new FramePoint2D();
   private final FramePoint2D pointBInStance = new FramePoint2D();
   private final Point2D tempPoint = new Point2D();

   /**
    * Given the start and end point of the swing as well as the position of the stance foot this method will compute
    * whether the nominal swing trajectory will be close to the stance foot. This is an indication that self collision
    * between swing and stance leg will occur. In that case a offset vector is computed and packed that will contain
    * a swing trajectory adjustment that will avoid this.
    */
   private boolean computeSwingAdjustment(FramePoint3D pointA, FramePoint3D pointB, FramePoint3D stance, Vector2D offsetToPack)
   {
      if (swingSide == null || stanceZUpFrame == null)
      {
         offsetToPack.setToZero();
         return false;
      }

      pointA2D.set(pointA);
      pointB2D.set(pointB);
      stance2D.set(stance);
      EuclidGeometryTools.orthogonalProjectionOnLine2D(stance2D, pointA2D, pointB2D, tempPoint);
      boolean smallAngleChange = !EuclidGeometryTools.isPoint2DOnLineSegment2D(tempPoint, pointA2D, pointB2D);

      xyDistanceToStance.setToZero(worldFrame);
      xyDistanceToStance.sub(tempPoint, stance2D);
      xyDistanceToStance.changeFrame(stanceZUpFrame);

      // If the nominal trajectory intersects the negative Y axis of the sole frame for a swing with the left side the step is a cross over step.
      pointAInStance.setIncludingFrame(worldFrame, pointA2D);
      pointBInStance.setIncludingFrame(worldFrame, pointB2D);
      pointAInStance.changeFrame(stanceZUpFrame);
      pointBInStance.changeFrame(stanceZUpFrame);
      boolean trajectoryIntersectsY = EuclidGeometryTools.intersectionBetweenLine2DAndLineSegment2D(0.0, 0.0, 0.0, 1.0, pointAInStance.getX(),
                                                                                                    pointAInStance.getY(), pointBInStance.getX(),
                                                                                                    pointBInStance.getY(), tempPoint);

      boolean crossOver = trajectoryIntersectsY && swingSide.negateIfRightSide(tempPoint.getY()) < 0.0;
      crossOverStep.set(crossOver);

      // Prevent adjusting on side steps or steps that do not change the angle between the feet much.
      if (!crossOver && smallAngleChange)
      {
         offsetToPack.setToZero();
         return false;
      }

      double distance;
      if (crossOver)
      {
         distance = minDistanceToStance.getDoubleValue() + xyDistanceToStance.length();
         xyDistanceToStance.negate();
      }
      else
      {
         distance = minDistanceToStance.getDoubleValue() - xyDistanceToStance.length();
      }

      if (distance < 0.0)
      {
         offsetToPack.setToZero();
         return false;
      }

      xyDistanceToStance.changeFrame(worldFrame);
      xyDistanceToStance.normalize();
      xyDistanceToStance.scale(distance);
      offsetToPack.set(xyDistanceToStance);
      return true;
   }

   /**
    * Calling this method will enable a simple collision avoidance heuristic in the swing generator: if a straight line in the xy plane
    * from the start to the end of the swing is too close to the stance position the trajectory waypoints will be adjusted. To activate
    * this, additional information has to be provided as arguments to this method.
    *
    * @param swingSide the side of the robot that this swing trajectory will be executed on
    * @param stanceZUpFrame the zup frame located at the stance foot sole
    * @param minDistanceToStance the minimum clearance that the swing should have from the stance foot sole point in the xy plane
    */
   public void enableStanceCollisionAvoidance(RobotSide swingSide, ReferenceFrame stanceZUpFrame, double minDistanceToStance)
   {
      this.swingSide = swingSide;
      this.stanceZUpFrame = stanceZUpFrame;
      this.minDistanceToStance.set(minDistanceToStance);
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

   @Override
   public void hideVisualization()
   {
      waypointViz.hideAll();
      tempPoint3D.setToNaN();
      for (int i = 0; i < numberWaypoints; i++)
         waypointViz.setBall(tempPoint3D, i);
      trajectory.hideVisualization();
   }

   public static double[] getDefaultWaypointProportions()
   {
      return defaultWaypointProportions;
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

   public double computeAndGetMaxSpeed()
   {
      trajectory.computeMaxSpeed();
      return trajectory.getMaxSpeed() / stepTime.getDoubleValue();
   }
}