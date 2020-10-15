package us.ihmc.atlas.behaviors.scsSensorSimulation;

import java.util.Random;
import java.util.concurrent.LinkedBlockingQueue;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.BagOfBalls;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.jMonkeyEngineToolkit.GPULidar;
import us.ihmc.robotics.controllers.PDController;
import us.ihmc.robotics.lidar.LidarScan;
import us.ihmc.robotics.lidar.LidarScanParameters;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class SensorOnlyController implements RobotController
{
   private YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final SensorOnlyRobot robot;
   private final SimulationConstructionSet scs;
   private final LidarScanParameters lidarScanParameters;
   private GPULidar gpuLidar;
   private LinkedBlockingQueue<LidarScan> gpuLidarScanBuffer = new LinkedBlockingQueue<>();
   private YoDouble tauLidarZ;
   private YoDouble tauLidarX;
   private YoDouble qLidarZ;
   private YoDouble qLidarX;

   private final YoFramePoint3D point = new YoFramePoint3D("point", ReferenceFrame.getWorldFrame(), registry);
   private final BagOfBalls bagOfBalls;

   private PDController pdControllerZ;
   private PDController pdControllerX;
   private final YoDouble proportionalGain;
   private final YoDouble derivativeGain;
   private double lastPositionZ;
   private double lastPositionX;
   private double lastTime;
   private double desiredZRate = 0.3;
   private final static double DESIRED_X_RATE = 0.3;

   public SensorOnlyController(SensorOnlyRobot robot, YoGraphicsListRegistry yoGraphicsListRegistry, SimulationConstructionSet scs)
   {
      this.robot = robot;
      this.scs = scs;
      this.lidarScanParameters = robot.getLidarScanParameters();

      YoGraphicPosition yoGraphicPosition = new YoGraphicPosition("point", point, 0.01, YoAppearance.Purple());
      yoGraphicsListRegistry.registerYoGraphic("test", yoGraphicPosition);

      bagOfBalls = new BagOfBalls(lidarScanParameters.getPointsPerSweep(), 0.005, YoAppearance.AliceBlue(), registry, yoGraphicsListRegistry);

      tauLidarZ = (YoDouble) robot.findVariable("tau_gimbalZ");
      tauLidarX = (YoDouble) robot.findVariable("tau_gimbalX");

      qLidarZ = (YoDouble) robot.findVariable("q_gimbalZ");
      qLidarX = (YoDouble) robot.findVariable("q_gimbalX");

      proportionalGain = new YoDouble("lidarPGain", registry);
      derivativeGain = new YoDouble("lidarDGain", registry);

      pdControllerX = new PDController(proportionalGain, derivativeGain, "LidarControllerX", registry);
      pdControllerZ = new PDController(proportionalGain, derivativeGain, "LidarControllerZ", registry);
      pdControllerZ.setProportionalGain(1.0);
      pdControllerZ.setDerivativeGain(1.0);

      lastPositionZ = qLidarZ.getDoubleValue();
      lastPositionX = qLidarX.getDoubleValue();
      lastTime = scs.getTime();
   }

   public void initialize()
   {
      startGPULidar();
   }

   private void startGPULidar()
   {
      gpuLidar = scs.getGraphics3dAdapter().createGPULidar(lidarScanParameters.getPointsPerSweep(), lidarScanParameters.getScanHeight(),
                                                           lidarScanParameters.getFieldOfView(), lidarScanParameters.getMinRange(),
                                                           lidarScanParameters.getMaxRange());
      gpuLidar.addGPULidarListener((scan, currentTransform,
                                    time) -> gpuLidarScanBuffer.add(new LidarScan(lidarScanParameters, new RigidBodyTransform(currentTransform),
                                                                                  new RigidBodyTransform(currentTransform), scan)));
   }

   public YoRegistry getYoRegistry()
   {
      return registry;
   }

   public String getName()
   {
      return registry.getName();
   }

   public String getDescription()
   {
      return getName();
   }

   private final Random random = new Random(1776L);

   public void doControl()
   {
      double currentTime = scs.getTime();
      double dt = currentTime - lastTime;
      lastTime = currentTime;

      double currentZPosition = qLidarZ.getValueAsDouble();
      double desiredZPosition = lastPositionZ + desiredZRate * dt;
      double currentZRate = (currentZPosition - lastPositionZ) * dt;

      double desiredXRate = Math.cos(currentZPosition) * DESIRED_X_RATE;
      double currentXPosition = qLidarX.getValueAsDouble();
      double desiredXPosition = lastPositionX + desiredXRate * dt;
      double currentXRate = (currentXPosition - lastPositionX) * dt;

      double zCorrectionSum = pdControllerZ.compute(currentZPosition, desiredZPosition, currentZRate, desiredZRate);
      double xCorrectionSum = pdControllerX.compute(currentXPosition, desiredXPosition, currentXRate, desiredXRate);

      tauLidarZ.set(zCorrectionSum);
      tauLidarX.set(xCorrectionSum);

      lastPositionX = currentXPosition;
      lastPositionZ = currentZPosition;

      RigidBodyTransform transform = new RigidBodyTransform();

      robot.getLidarXJoint().getTransformToWorld(transform);

      gpuLidar.setTransformFromWorld(transform, 0);

      while (!gpuLidarScanBuffer.isEmpty())
      {
         LidarScan scan = gpuLidarScanBuffer.poll();

         for (int i = 0; i < scan.size(); i++)
         {
            bagOfBalls.setBallLoop(new FramePoint3D(ReferenceFrame.getWorldFrame(), scan.getPoint(i)), YoAppearance.randomColor(random));
         }
      }
   }
}
