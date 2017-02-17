package us.ihmc.avatar.networkProcessor.quadTreeHeightMap;

import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

import org.apache.commons.lang3.mutable.MutableBoolean;

import us.ihmc.avatar.networkProcessor.modules.ToolboxController;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.packets.RequestLidarScanMessage;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D32;
import us.ihmc.euclid.tuple4D.Quaternion32;
import us.ihmc.humanoidRobotics.communication.packets.walking.CapturabilityBasedStatus;
import us.ihmc.humanoidRobotics.communication.toolbox.heightQuadTree.command.HeightQuadTreeToolboxRequestCommand;
import us.ihmc.humanoidRobotics.communication.toolbox.heightQuadTree.command.LidarScanCommand;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullRobotModelUtils;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.FrameConvexPolygon2d;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.lists.FrameTupleArrayList;
import us.ihmc.robotics.quadTree.Box;
import us.ihmc.robotics.quadTree.QuadTreeForGroundParameters;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.FloatingInverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.sensors.ForceSensorDefinition;
import us.ihmc.robotics.sensors.IMUDefinition;
import us.ihmc.sensorProcessing.communication.packets.dataobjects.RobotConfigurationData;
import us.ihmc.sensorProcessing.pointClouds.combinationQuadTreeOctTree.QuadTreeForGroundHeightMap;
import us.ihmc.tools.io.printing.PrintTools;

public class HeightQuadTreeToolboxController extends ToolboxController
{
   private static final double RESOLUTION = 0.03;
   private static final boolean DEBUG = false;
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final double QUAD_TREE_EXTENT = 200;

   private final QuadTreeForGroundHeightMap quadTree;

   private float quadtreeHeightThreshold = 0.02f;
   private float quadTreeMaxMultiLevelZChangeToFilterNoise = 0.2f;
   private int maxSameHeightPointsPerNode = 4;
   private double maxAllowableXYDistanceForAPointToBeConsideredClose = 0.2;
   private int maximumNumberOfPoints = maxSameHeightPointsPerNode * 75000;

   private double minRange = 0.2;
   private double maxRange = 5.0;
   private double maxZ = 0.5;

   private final CommandInputManager commandInputManager;
   private final AtomicReference<RobotConfigurationData> robotConfigurationDataToProcess = new AtomicReference<>(null);
   private final AtomicReference<CapturabilityBasedStatus> capturabilityBasedStatusToProcess = new AtomicReference<>(null);
   private final FrameTupleArrayList<FramePoint> contactPoints = FrameTupleArrayList.createFramePointArrayList();
   private final int expectedRobotConfigurationDataHash;
   private final FullHumanoidRobotModel fullRobotModel;
   private final FloatingInverseDynamicsJoint rootJoint;
   private final OneDoFJoint[] oneDoFJoints;
   private final PacketCommunicator packetCommunicator;

   private final Point2D robotPosition2d = new Point2D();
   private final double quadTreeMessageMaxRadius = 5.0;

   public HeightQuadTreeToolboxController(FullHumanoidRobotModel fullRobotModel, PacketCommunicator packetCommunicator, CommandInputManager commandInputManager,
         StatusMessageOutputManager statusOutputManager, YoVariableRegistry parentRegistry)
   {
      super(statusOutputManager, parentRegistry);
      this.fullRobotModel = fullRobotModel;
      this.packetCommunicator = packetCommunicator;
      rootJoint = fullRobotModel.getRootJoint();
      this.commandInputManager = commandInputManager;

      oneDoFJoints = FullRobotModelUtils.getAllJointsExcludingHands(fullRobotModel);
      ForceSensorDefinition[] forceSensorDefinitions = fullRobotModel.getForceSensorDefinitions();
      IMUDefinition[] imuDefinitions = fullRobotModel.getIMUDefinitions();
      expectedRobotConfigurationDataHash = RobotConfigurationData.calculateJointNameHash(oneDoFJoints, forceSensorDefinitions, imuDefinitions);

      Box bounds = new Box(-QUAD_TREE_EXTENT, -QUAD_TREE_EXTENT, QUAD_TREE_EXTENT, QUAD_TREE_EXTENT);
      QuadTreeForGroundParameters quadTreeParameters = new QuadTreeForGroundParameters(RESOLUTION, quadtreeHeightThreshold,
            quadTreeMaxMultiLevelZChangeToFilterNoise, maxSameHeightPointsPerNode, maxAllowableXYDistanceForAPointToBeConsideredClose, maximumNumberOfPoints);

      quadTree = new QuadTreeForGroundHeightMap(bounds, quadTreeParameters);
   }

   @Override
   protected boolean initialize()
   {
      return true;
   }

   private final FramePoint scanPoint = new FramePoint();
   private final MutableBoolean quadTreeUpdateRequested = new MutableBoolean(false);

   @Override
   protected void updateInternal()
   {
      packetCommunicator.send(new RequestLidarScanMessage());

      updateRobotContactPoints();

      // Wait until we receive some contact points.
      if (contactPoints.isEmpty())
         return;

      if (commandInputManager.isNewCommandAvailable(HeightQuadTreeToolboxRequestCommand.class))
      {
         HeightQuadTreeToolboxRequestCommand command = commandInputManager.pollNewestCommand(HeightQuadTreeToolboxRequestCommand.class);

         if (command.isQuadTreeUpdateRequested())
            quadTreeUpdateRequested.setTrue();

         if (command.isClearQuadTreeRequested())
         {
            PrintTools.info("clearing the quad tree!");
            quadTree.clearTree(Double.NaN);
            commandInputManager.flushAllCommands();
            return;
         }
      }

      if (!quadTree.hasPoints())
      {
         for (int contactPointIndex = 0; contactPointIndex < contactPoints.size(); contactPointIndex++)
         {
            FramePoint contactPoint = contactPoints.get(contactPointIndex);
            quadTree.addPoint(contactPoint.getX(), contactPoint.getY(), contactPoint.getZ());
         }
      }

      if (!commandInputManager.isNewCommandAvailable(LidarScanCommand.class))
      {
         return;
      }

      List<LidarScanCommand> newPointClouds = commandInputManager.pollNewCommands(LidarScanCommand.class);

      if (DEBUG)
         PrintTools.debug("Received new point cloud. Number of points: " + newPointClouds.get(0).getNumberOfPoints());

      Point3D lidarPosition = new Point3D();

      for (int pointCloudIndex = 0; pointCloudIndex < newPointClouds.size(); pointCloudIndex++)
      {
         LidarScanCommand scan = newPointClouds.get(pointCloudIndex);
         scan.getLidarPosition(lidarPosition);

         for (int pointIndex = 0; pointIndex < scan.getNumberOfPoints(); pointIndex++)
         {
            scan.getFramePoint(pointIndex, scanPoint);
            scanPoint.changeFrame(worldFrame);

            double distanceFromSensor = scanPoint.distance(lidarPosition);
            if (distanceFromSensor > maxRange || distanceFromSensor < minRange || scanPoint.getZ() > lidarPosition.getZ() + maxZ)
               continue;

            double x = scanPoint.getX();
            double y = scanPoint.getY();
            double z = scanPoint.getZ();
            quadTree.addPoint(x, y, z);
         }
      }

      if (DEBUG)
         PrintTools.debug("Done updating the QuadTree.");

      if (quadTreeUpdateRequested.booleanValue())
      {
         if (DEBUG)
            PrintTools.debug("QuadTree has changed, sending packet");
         Point3D rootJointPosition = new Point3D();
         rootJoint.getTranslation(rootJointPosition);
         robotPosition2d.set(rootJointPosition.getX(), rootJointPosition.getY());
         reportMessage(HeightQuadTreeMessageConverter.convertQuadTreeForGround(quadTree, robotPosition2d, quadTreeMessageMaxRadius));
         quadTreeUpdateRequested.setFalse();
      }
   }

   private final FramePoint2d contactPoint2d = new FramePoint2d();

   private void updateRobotContactPoints()
   {
      RobotConfigurationData robotConfigurationData = robotConfigurationDataToProcess.getAndSet(null);
      if (robotConfigurationData != null)
      {
         if (expectedRobotConfigurationDataHash != robotConfigurationData.jointNameHash)
            throw new RuntimeException("Received a " + RobotConfigurationData.class.getSimpleName() + " that does not match the fullRobotModel.");

         float[] newJointAngles = robotConfigurationData.getJointAngles();
         for (int i = 0; i < newJointAngles.length; i++)
         {
            oneDoFJoints[i].setQ(newJointAngles[i]);
         }

         Vector3D32 translation = robotConfigurationData.getPelvisTranslation();
         rootJoint.setPosition(translation.getX(), translation.getY(), translation.getZ());
         Quaternion32 orientation = robotConfigurationData.getPelvisOrientation();
         rootJoint.setRotation(orientation.getX(), orientation.getY(), orientation.getZ(), orientation.getS());
         rootJoint.getPredecessor().updateFramesRecursively();
      }

      CapturabilityBasedStatus capturabilityBasedStatus = capturabilityBasedStatusToProcess.getAndSet(null);
      if (capturabilityBasedStatus != null)
      {
         contactPoints.clear();

         for (RobotSide robotSide : RobotSide.values)
         {
            ReferenceFrame soleFrame = fullRobotModel.getSoleFrame(robotSide);
            FrameConvexPolygon2d footSupportPolygon = capturabilityBasedStatus.getFootSupportPolygon(robotSide);
            for (int contactPointIndex = 0; contactPointIndex < footSupportPolygon.getNumberOfVertices(); contactPointIndex++)
            {
               footSupportPolygon.getFrameVertex(contactPointIndex, contactPoint2d);
               findProjectionOntoPlaneFrame(soleFrame, contactPoint2d, contactPoints.add());
            }
         }
      }
   }

   private void findProjectionOntoPlaneFrame(ReferenceFrame planeFrame, FramePoint2d pointInWorld, FramePoint pointProjectedOntoPlaneFrameToPack)
   {
      pointInWorld.checkReferenceFrameMatch(worldFrame);

      double z = getPlaneZGivenXY(planeFrame, pointInWorld.getX(), pointInWorld.getY());
      pointProjectedOntoPlaneFrameToPack.setXYIncludingFrame(pointInWorld);
      pointProjectedOntoPlaneFrameToPack.setZ(z);
   }

   private double getPlaneZGivenXY(ReferenceFrame planeFrame, double xWorld, double yWorld)
   {
      RigidBodyTransform fromLocalToWorldTransform = planeFrame.getTransformToWorldFrame();

      // The three components of the plane origin
      double x0 = fromLocalToWorldTransform.getM03();
      double y0 = fromLocalToWorldTransform.getM13();
      double z0 = fromLocalToWorldTransform.getM23();
      // The three components of the plane normal
      double a = fromLocalToWorldTransform.getM02();
      double b = fromLocalToWorldTransform.getM12();
      double c = fromLocalToWorldTransform.getM22();

      // Given the plane equation: a*x + b*y + c*z + d = 0, with d = -(a*x0 + b*y0 + c*z0), we find z:
      double z = a / c * (x0 - xWorld) + b / c * (y0 - yWorld) + z0;
      return z;
   }

   PacketConsumer<RobotConfigurationData> robotConfigurationDataConsumer()
   {
      return new PacketConsumer<RobotConfigurationData>()
      {
         @Override
         public void receivedPacket(RobotConfigurationData packet)
         {
            if (packet != null)
               robotConfigurationDataToProcess.set(packet);
         }
      };
   }

   PacketConsumer<CapturabilityBasedStatus> capturabilityBasedStatusConsumer()
   {
      return new PacketConsumer<CapturabilityBasedStatus>()
      {
         @Override
         public void receivedPacket(CapturabilityBasedStatus packet)
         {
            if (packet != null)
               capturabilityBasedStatusToProcess.set(packet);
         }
      };
   }

   @Override
   protected boolean isDone()
   {
      return false;
   }
}
