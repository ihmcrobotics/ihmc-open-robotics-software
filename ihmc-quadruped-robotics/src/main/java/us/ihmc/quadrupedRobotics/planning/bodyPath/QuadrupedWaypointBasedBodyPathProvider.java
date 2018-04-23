package us.ihmc.quadrupedRobotics.planning.bodyPath;

import controller_msgs.msg.dds.EuclideanTrajectoryPointMessage;
import controller_msgs.msg.dds.QuadrupedBodyPathPlanMessage;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose2D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.BagOfBalls;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicVector;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsList;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.quadrupedRobotics.estimator.referenceFrames.QuadrupedReferenceFrames;
import us.ihmc.robotics.math.trajectories.waypoints.MultipleWaypointsPositionTrajectoryGenerator;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoFramePoint3D;
import us.ihmc.yoVariables.variable.YoFrameVector3D;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

public class QuadrupedWaypointBasedBodyPathProvider implements QuadrupedPlanarBodyPathProvider
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final int numberOfVisualizationPoints = 20;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final AtomicReference<QuadrupedBodyPathPlanMessage> bodyPathPlanMessage = new AtomicReference<>();
   private final YoDouble timestamp;
   private final MultipleWaypointsPositionTrajectoryGenerator trajectory = new MultipleWaypointsPositionTrajectoryGenerator("bodyPath", worldFrame, registry);
   private final FramePoint3D trajectoryValue = new FramePoint3D(worldFrame);
   private final YoFramePoint3D yoTrajectoryValue = new YoFramePoint3D("desiredPlanarPose", worldFrame, registry);

   private final YoGraphicVector[] pathVisualization;
   private final YoFramePoint3D[] visualizationStartPoints;
   private final YoFrameVector3D[] visualizationDirections;

   private final ReferenceFrame supportFrame;
   private final FramePose3D supportPose = new FramePose3D();

   public QuadrupedWaypointBasedBodyPathProvider(QuadrupedReferenceFrames referenceFrames, PacketCommunicator packetCommunicator, YoDouble timestamp, YoGraphicsListRegistry graphicsListRegistry, YoVariableRegistry parentRegistry)
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

      packetCommunicator.attachListener(QuadrupedBodyPathPlanMessage.class, bodyPathPlanMessage::set);
      parentRegistry.addChild(registry);
   }

   @Override
   public void initialize()
   {
      trajectory.clear();
      appendCurrentPoseAsWaypoint();

      QuadrupedBodyPathPlanMessage bodyPathPlanMessage = this.bodyPathPlanMessage.getAndSet(null);
      List<EuclideanTrajectoryPointMessage> originalBodyPathPoints = bodyPathPlanMessage.getBodyPathPoints();

      double currentTime = timestamp.getDoubleValue();
      boolean isExpressedInAbsoluteTime = bodyPathPlanMessage.getIsExpressedInAbsoluteTime();
      for (int i = 0; i < originalBodyPathPoints.size(); i++)
      {
         EuclideanTrajectoryPointMessage waypoint = originalBodyPathPoints.get(i);

         if(!isExpressedInAbsoluteTime)
         {
            waypoint.setTime(waypoint.getTime() + currentTime);
         }

         if(waypoint.getTime() < currentTime)
         {
            continue;
         }
         else
         {
            trajectory.appendWaypoint(waypoint.getTime(), waypoint.getPosition(), waypoint.getLinearVelocity());
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
            tempPoint.setZ(0.05);
            double yaw = tempPoint.getZ();

            visualizationStartPoints[i].set(tempPoint);
            visualizationDirections[i].set(Math.cos(yaw), Math.sin(yaw), 0.0);
            visualizationDirections[i].scale(0.05);
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
      return bodyPathPlanMessage.get() != null;
   }

   public boolean isDone()
   {
      return timestamp.getDoubleValue() > trajectory.getLastWaypointTime();
   }

   public void setBodyPathPlanMessage(QuadrupedBodyPathPlanMessage bodyPathPlanMessage)
   {
      this.bodyPathPlanMessage.set(bodyPathPlanMessage);
   }
}
