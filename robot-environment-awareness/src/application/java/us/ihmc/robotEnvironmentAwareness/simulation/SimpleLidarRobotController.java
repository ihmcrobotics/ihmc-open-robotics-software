package us.ihmc.robotEnvironmentAwareness.simulation;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.LinkedBlockingQueue;

import perception_msgs.msg.dds.LidarScanMessage;
import perception_msgs.msg.dds.PlanarRegionsListMessage;
import us.ihmc.ros2.ROS2PublisherBasics;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.packets.LidarPointCloudCompression;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.Quaternion32;
import us.ihmc.graphicsDescription.yoGraphics.BagOfBalls;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.jMonkeyEngineToolkit.GPULidar;
import us.ihmc.jMonkeyEngineToolkit.Graphics3DAdapter;
import us.ihmc.robotics.graphics.YoGraphicPlanarRegionsList;
import us.ihmc.robotics.lidar.LidarScan;
import us.ihmc.robotics.lidar.LidarScanParameters;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.simulationconstructionset.FloatingJoint;
import us.ihmc.simulationconstructionset.PinJoint;
import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameYawPitchRoll;
import us.ihmc.yoVariables.listener.YoVariableChangedListener;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoVariable;

public class SimpleLidarRobotController implements RobotController
{
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final YoBoolean spinLidar = new YoBoolean("spinLidar", registry);
   private final YoDouble desiredLidarVelocity = new YoDouble("desiredLidarVelocity", registry);
   private final YoDouble lidarRange = new YoDouble("lidarRange", registry);

   private final PinJoint lidarJoint;
   private final double dt;
   private final FloatingJoint rootJoint;

   private final LidarScanParameters lidarScanParameters;
   private final GPULidar gpuLidar;
   private final LinkedBlockingQueue<LidarScan> gpuLidarScanBuffer = new LinkedBlockingQueue<>();
   private final int vizualizeEveryNPoints = 5;
   private final BagOfBalls sweepViz;

   private final YoGraphicPlanarRegionsList yoGraphicPlanarRegionsList = new YoGraphicPlanarRegionsList("region", 100, 150, registry);

   private final ROS2PublisherBasics<LidarScanMessage> lidarScanPublisher;

   public SimpleLidarRobotController(SimpleLidarRobot lidarRobot, double dt, ROS2Node ros2Node, Graphics3DAdapter graphics3dAdapter,
                                     YoGraphicsListRegistry yoGraphicsListRegistry) throws IOException
   {
      this.dt = dt;
      lidarJoint = lidarRobot.getLidarJoint();
      rootJoint = lidarRobot.getRootJoint();

      desiredLidarVelocity.set(LidarFastSimulation.DEFAULT_SPIN_VELOCITY);
      spinLidar.set(true);
      lidarRange.set(30.0);

      final YoFrameYawPitchRoll lidarYawPitchRoll = new YoFrameYawPitchRoll("lidar", null, registry);
      lidarYawPitchRoll.attachVariableChangedListener(new YoVariableChangedListener()
      {
         private final Quaternion localQuaternion = new Quaternion();

         @Override
         public void changed(YoVariable v)
         {
            localQuaternion.set(lidarYawPitchRoll);
            rootJoint.setQuaternion(localQuaternion);
         }
      });

      lidarScanParameters = lidarRobot.getLidarScanParameters();
      gpuLidar = graphics3dAdapter.createGPULidar(lidarScanParameters.getPointsPerSweep(), lidarScanParameters.getScanHeight(),
                                                  lidarScanParameters.getFieldOfView(), lidarScanParameters.getMinRange(), lidarScanParameters.getMaxRange());
      gpuLidar.addGPULidarListener((scan, currentTransform,
                                    time) -> gpuLidarScanBuffer.add(new LidarScan(lidarScanParameters, new RigidBodyTransform(currentTransform),
                                                                                  new RigidBodyTransform(currentTransform), scan)));
      if (LidarFastSimulation.VISUALIZE_GPU_LIDAR)
         sweepViz = BagOfBalls.createRainbowBag(lidarScanParameters.getPointsPerSweep() / vizualizeEveryNPoints, 0.005, "SweepViz", registry,
                                                yoGraphicsListRegistry);
      else
         sweepViz = null;
      yoGraphicPlanarRegionsList.hideGraphicObject();
      yoGraphicsListRegistry.registerYoGraphic("Regions", yoGraphicPlanarRegionsList);

      lidarScanPublisher = ROS2Tools.createPublisher(ros2Node, PerceptionAPI.MULTISENSE_LIDAR_SCAN);
   }

   @Override
   public void initialize()
   {
   }

   @Override
   public void doControl()
   {
      if (spinLidar.getBooleanValue())
      {
         lidarJoint.getQYoVariable().add(desiredLidarVelocity.getDoubleValue() * dt);
      }

      RigidBodyTransform transform = new RigidBodyTransform();
      Point3D32 lidarPosition = new Point3D32();
      Quaternion32 lidarOrientation = new Quaternion32();

      lidarJoint.getTransformToWorld(transform);
      transform.get(lidarOrientation, lidarPosition);

      gpuLidar.setTransformFromWorld(transform, 0);

      while (!gpuLidarScanBuffer.isEmpty())
      {
         LidarScan scan = gpuLidarScanBuffer.poll();

         if (LidarFastSimulation.VISUALIZE_GPU_LIDAR)
         {
            for (int i = 0; i < scan.size(); i += vizualizeEveryNPoints)
            {
               sweepViz.setBallLoop(new FramePoint3D(ReferenceFrame.getWorldFrame(), scan.getPoint(i)));
            }
         }

         List<Point3D> newScan = new ArrayList<>();
         for (int i = 0; i < scan.size(); i++)
         {
            Point3D sensorOrigin = new Point3D();
            sensorOrigin.set(transform.getTranslation());
            Point3D scanPoint = scan.getPoint(i);
            if (sensorOrigin.distance(scanPoint) < lidarRange.getDoubleValue())
            {
               newScan.add(scanPoint);
            }
         }

         LidarScanMessage lidarScanMessage = new LidarScanMessage();
         lidarScanMessage.setRobotTimestamp(-1L);
         lidarScanMessage.getLidarPosition().set(lidarPosition);
         lidarScanMessage.getLidarOrientation().set(lidarOrientation);
         LidarPointCloudCompression.compressPointCloud(scan.size(), lidarScanMessage, (i, j) -> newScan.get(i).getElement32(j));
         lidarScanPublisher.publish(lidarScanMessage);
      }

      yoGraphicPlanarRegionsList.processPlanarRegionsListQueue();
   }

   public void handlePacket(PlanarRegionsListMessage message)
   {
      if (message != null)
         yoGraphicPlanarRegionsList.submitPlanarRegionsListToRender(PlanarRegionMessageConverter.convertToPlanarRegionsList(message));
   }

   @Override
   public YoRegistry getYoRegistry()
   {
      return registry;
   }

   @Override
   public String getName()
   {
      return registry.getName();
   }

   @Override
   public String getDescription()
   {
      return getName();
   }
}
