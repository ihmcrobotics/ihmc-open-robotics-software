package us.ihmc.quadrupedPlanning.stepStream.bodyPath;

import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

import controller_msgs.msg.dds.EuclideanTrajectoryPointMessage;
import controller_msgs.msg.dds.QuadrupedBodyPathPlanMessage;
import us.ihmc.euclid.geometry.Pose2D;
import us.ihmc.euclid.geometry.interfaces.Pose2DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose2D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicVector;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsList;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.quadrupedBasics.referenceFrames.QuadrupedReferenceFrames;
import us.ihmc.robotics.math.trajectories.generators.MultipleWaypointsPositionTrajectoryGenerator;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class QuadrupedWaypointBasedBodyPathProvider implements QuadrupedPlanarBodyPathProvider
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final int numberOfVisualizationPoints = 20;

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final AtomicReference<QuadrupedBodyPathPlan> bodyPathPlan = new AtomicReference<>();
   private final YoDouble timestamp;
   private final MultipleWaypointsPositionTrajectoryGenerator trajectory = new MultipleWaypointsPositionTrajectoryGenerator("bodyPath", worldFrame, registry);
   private final FramePoint3D trajectoryValue = new FramePoint3D(worldFrame);
   private final YoFramePoint3D yoTrajectoryValue = new YoFramePoint3D("desiredPlanarPose", worldFrame, registry);

   private final YoGraphicVector[] pathVisualization;
   private final YoFramePoint3D[] visualizationStartPoints;
   private final YoFrameVector3D[] visualizationDirections;

   private final ReferenceFrame supportFrame;
   private final FramePose3D supportPose = new FramePose3D();

   public QuadrupedWaypointBasedBodyPathProvider(QuadrupedReferenceFrames referenceFrames, YoDouble timestamp, YoGraphicsListRegistry graphicsListRegistry, YoRegistry parentRegistry)
   {
      this.timestamp = timestamp;
      this.supportFrame = referenceFrames.getCenterOfFeetZUpFrameAveragingLowestZHeightsAcrossEnds();

      if(graphicsListRegistry == null)
      {
         visualizationStartPoints = null;
         visualizationDirections = null;
         pathVisualization = null;
      }
      else
      {
         YoGraphicsList graphicsList = new YoGraphicsList("bodyPath");
         pathVisualization = new YoGraphicVector[numberOfVisualizationPoints];
         visualizationStartPoints = new YoFramePoint3D[numberOfVisualizationPoints];
         visualizationDirections = new YoFrameVector3D[numberOfVisualizationPoints];
         for (int i = 0; i < numberOfVisualizationPoints; i++)
         {
            visualizationStartPoints[i] = new YoFramePoint3D("bodyPathVizStart" + i, worldFrame, registry);
            visualizationDirections[i] = new YoFrameVector3D("bodyPathVizDirection" + i, worldFrame, registry);
            pathVisualization[i] = new YoGraphicVector("bodyPathViz" + i, visualizationStartPoints[i], visualizationDirections[i], YoAppearance
                  .RGBColor(((double) i) / numberOfVisualizationPoints, ((double) numberOfVisualizationPoints - i) / numberOfVisualizationPoints, 0.0));
            visualizationStartPoints[i].setToNaN();
            graphicsList.add(pathVisualization[i]);
         }

         graphicsListRegistry.registerYoGraphicsList(graphicsList);
      }


      parentRegistry.addChild(registry);
   }

   public void setBodyPathPlan(QuadrupedBodyPathPlan bodyPathPlan)
   {
      this.bodyPathPlan.set(bodyPathPlan);
   }

   @Override
   public void initialize()
   {
      trajectory.clear();
      appendCurrentPoseAsWaypoint();

      QuadrupedBodyPathPlan bodyPathPlan = this.bodyPathPlan.getAndSet(null);

      double currentTime = timestamp.getDoubleValue();
      boolean isExpressedInAbsoluteTime = bodyPathPlan.isExpressedInAbsoluteTime();


      for (int i = 0; i < bodyPathPlan.size(); i++)
      {
         double time = bodyPathPlan.getWaypointTime(i);
         if(!isExpressedInAbsoluteTime)
         {
            time += currentTime;
         }

         if (time < currentTime)
         {
            continue;
         }
         else
         {
            Point2DReadOnly position = bodyPathPlan.getWaypointPose(i).getPosition();
            double yaw = bodyPathPlan.getWaypointPose(i).getYaw();
            Vector2DReadOnly velocity = bodyPathPlan.getWaypointLinearVelocity(i);
            double yawRate = bodyPathPlan.getWaypointYawRate(i);

            Point3DReadOnly trajectoryPositionKnot = new Point3D(position.getX(), position.getY(), yaw);
            Vector3DReadOnly trajectoryVelocityKnot = new Vector3D(velocity.getX(), velocity.getY(), yawRate);

            trajectory.appendWaypoint(time, trajectoryPositionKnot, trajectoryVelocityKnot);
         }
      }



      trajectory.initialize();
      setupGraphics();
   }

   FramePoint3D tempPoint = new FramePoint3D();

   private void setupGraphics()
   {
      if(pathVisualization != null)
      {
         double startTime = timestamp.getDoubleValue();
         double endTime = trajectory.getLastWaypointTime();
         double dt = (endTime - startTime) / (numberOfVisualizationPoints - 1);

         for (int i = 0; i < numberOfVisualizationPoints; i++)
         {
            double t = startTime + dt * i;
            trajectory.compute(t);
            trajectory.getPosition(tempPoint);
            double yaw = tempPoint.getZ();
            tempPoint.setZ(0.02);

            visualizationStartPoints[i].set(tempPoint);
            visualizationDirections[i].set(Math.cos(yaw), Math.sin(yaw), 0.0);
            visualizationDirections[i].scale(0.2);
         }
      }
   }

   private void appendCurrentPoseAsWaypoint()
   {
      supportPose.setToZero(supportFrame);
      supportPose.changeFrame(worldFrame);

      trajectory.appendWaypoint(timestamp.getDoubleValue(), new Point3D(supportPose.getX(), supportPose.getY(), supportPose.getYaw()), new Vector3D());
   }

   @Override
   public void getPlanarPose(double time, FramePose2D poseToPack)
   {
      trajectory.compute(time);
      trajectory.getPosition(trajectoryValue);
      poseToPack.setIncludingFrame(worldFrame, trajectoryValue.getX(), trajectoryValue.getY(), trajectoryValue.getZ());
      yoTrajectoryValue.set(trajectoryValue);
   }

   public boolean bodyPathIsAvailable()
   {
      return bodyPathPlan.get() != null;
   }

   public boolean isDone()
   {
      return timestamp.getDoubleValue() > trajectory.getLastWaypointTime();
   }

   public void setBodyPathPlanMessage(QuadrupedBodyPathPlanMessage bodyPathPlanMessage)
   {

      this.bodyPathPlan.set(convertToBodyPathPlan(bodyPathPlanMessage));
   }

   private static QuadrupedBodyPathPlan convertToBodyPathPlan(QuadrupedBodyPathPlanMessage message)
   {
      QuadrupedBodyPathPlan pathPlan = new QuadrupedBodyPathPlan();
      List<EuclideanTrajectoryPointMessage> trajectoryPoints = message.getBodyPathPoints();
      boolean isExpressedInAbsoluteTime = message.getIsExpressedInAbsoluteTime();

      pathPlan.setExpressedInAbsoluteTime(isExpressedInAbsoluteTime);

      for (int i = 0; i < trajectoryPoints.size(); i++)
      {
         EuclideanTrajectoryPointMessage point = trajectoryPoints.get(i);
         Pose2DReadOnly pose = new Pose2D(point.getPosition().getX(), point.getPosition().getY(), point.getPosition().getZ());
         Vector2DReadOnly linearVelocity = new Vector2D(point.getLinearVelocity().getX(), point.getLinearVelocity().getY());
         double yawRate = point.getLinearVelocity().getZ();
         double time = point.getTime();
         pathPlan.addWaypoint(pose, linearVelocity, yawRate, time);
      }

      return pathPlan;
   }
}
