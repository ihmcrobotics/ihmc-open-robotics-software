package us.ihmc.robotEnvironmentAwareness.simulation;

import java.util.concurrent.ScheduledExecutorService;

import gnu.trove.list.array.TFloatArrayList;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.packets.LidarScanMessage;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.communication.packets.PlanarRegionsListMessage;
import us.ihmc.communication.packets.RequestPlanarRegionsListMessage;
import us.ihmc.communication.packets.RequestPlanarRegionsListMessage.RequestType;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.Quaternion32;
import us.ihmc.graphicsDescription.yoGraphics.BagOfBalls;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPlanarRegionsList;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.jMonkeyEngineToolkit.GPULidar;
import us.ihmc.jMonkeyEngineToolkit.GPULidarScanBuffer;
import us.ihmc.jMonkeyEngineToolkit.Graphics3DAdapter;
import us.ihmc.robotEnvironmentAwareness.tools.ExecutorServiceTools;
import us.ihmc.robotEnvironmentAwareness.tools.ExecutorServiceTools.ExceptionHandling;
import us.ihmc.robotics.lidar.LidarScan;
import us.ihmc.robotics.lidar.LidarScanParameters;
import us.ihmc.robotics.math.frames.YoFrameOrientation;
import us.ihmc.robotics.robotController.RobotController;
import us.ihmc.simulationconstructionset.FloatingJoint;
import us.ihmc.simulationconstructionset.PinJoint;
import us.ihmc.yoVariables.listener.VariableChangedListener;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoVariable;

public class SimpleLidarRobotController implements RobotController
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final ScheduledExecutorService executorService = ExecutorServiceTools.newSingleThreadScheduledExecutor(ThreadTools.getNamedThreadFactory("Messaging"), ExceptionHandling.CATCH_AND_REPORT);

   private final YoBoolean spinLidar = new YoBoolean("spinLidar", registry);
   private final YoDouble desiredLidarVelocity = new YoDouble("desiredLidarVelocity", registry);
   private final YoDouble lidarRange = new YoDouble("lidarRange", registry);

   private final PinJoint lidarJoint;
   private final double dt;
   private final FloatingJoint rootJoint;

   private final LidarScanParameters lidarScanParameters;
   private final GPULidar gpuLidar;
   private final GPULidarScanBuffer gpuLidarScanBuffer;
   private final int vizualizeEveryNPoints = 5;
   private final BagOfBalls sweepViz;

   private final PacketCommunicator packetCommunicator;

   private final YoGraphicPlanarRegionsList yoGraphicPlanarRegionsList = new YoGraphicPlanarRegionsList("region", 100, 150, registry);

   public SimpleLidarRobotController(SimpleLidarRobot lidarRobot, double dt, PacketCommunicator packetCommunicator, Graphics3DAdapter graphics3dAdapter,
         YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this.dt = dt;
      this.packetCommunicator = packetCommunicator;
      lidarJoint = lidarRobot.getLidarJoint();
      rootJoint = lidarRobot.getRootJoint();

      desiredLidarVelocity.set(LidarFastSimulation.DEFAULT_SPIN_VELOCITY);
      spinLidar.set(true);
      lidarRange.set(30.0);

      final YoFrameOrientation lidarYawPitchRoll = new YoFrameOrientation("lidar", null, registry);
      lidarYawPitchRoll.attachVariableChangedListener(new VariableChangedListener()
      {
         private final Quaternion localQuaternion = new Quaternion();

         @Override
         public void notifyOfVariableChange(YoVariable<?> v)
         {
            lidarYawPitchRoll.getQuaternion(localQuaternion);
            rootJoint.setQuaternion(localQuaternion);
         }
      });

      lidarScanParameters = lidarRobot.getLidarScanParameters();
      gpuLidarScanBuffer = new GPULidarScanBuffer(lidarScanParameters);
      gpuLidar = graphics3dAdapter.createGPULidar(gpuLidarScanBuffer, lidarScanParameters);
      if (LidarFastSimulation.VISUALIZE_GPU_LIDAR)
         sweepViz = BagOfBalls.createRainbowBag(lidarScanParameters.getPointsPerSweep() / vizualizeEveryNPoints, 0.005, "SweepViz", registry, yoGraphicsListRegistry);
      else
         sweepViz = null;
      yoGraphicPlanarRegionsList.hideGraphicObject();
      yoGraphicsListRegistry.registerYoGraphic("Regions", yoGraphicPlanarRegionsList);
      packetCommunicator.attachListener(PlanarRegionsListMessage.class, this::handlePacket);
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

         TFloatArrayList newScan = new TFloatArrayList();
         for (int i = 0; i < scan.size(); i++)
         {
            Point3D sensorOrigin = new Point3D();
            transform.getTranslation(sensorOrigin);
            Point3D scanPoint = scan.getPoint(i);
            if (sensorOrigin.distance(scanPoint) < lidarRange.getDoubleValue())
            {
               newScan.add((float) scanPoint.getX());
               newScan.add((float) scanPoint.getY());
               newScan.add((float) scanPoint.getZ());
            }
         }

         LidarScanMessage lidarScanMessage = new LidarScanMessage();
         lidarScanMessage.robotTimestamp = -1L;
         lidarScanMessage.lidarPosition = lidarPosition;
         lidarScanMessage.lidarOrientation = lidarOrientation;
         lidarScanMessage.scan = newScan.toArray();
         executorService.execute(() -> packetCommunicator.send(lidarScanMessage));
      }

      packetCommunicator.send(MessageTools.createRequestPlanarRegionsListMessage(RequestType.CONTINUOUS_UPDATE));
      yoGraphicPlanarRegionsList.processPlanarRegionsListQueue();
   }

   public void handlePacket(PlanarRegionsListMessage message)
   {
      if (message != null)
         yoGraphicPlanarRegionsList.submitPlanarRegionsListToRender(PlanarRegionMessageConverter.convertToPlanarRegionsList(message));
   }

   @Override
   public YoVariableRegistry getYoVariableRegistry()
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
