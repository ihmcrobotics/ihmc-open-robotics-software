package us.ihmc.exampleSimulations.lidar;

import java.util.Random;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.BagOfBalls;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.jMonkeyEngineToolkit.GPULidar;
import us.ihmc.jMonkeyEngineToolkit.GPULidarScanBuffer;
import us.ihmc.robotics.controllers.PDController;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.lidar.LidarScan;
import us.ihmc.robotics.lidar.LidarScanParameters;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotController.RobotController;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;

public class ExampleLidarController implements RobotController
{
   private YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final ExampleLidarRobot robot;
   private final SimulationConstructionSet scs;
   private final LidarScanParameters lidarScanParameters;
   private GPULidar gpuLidar;
   private GPULidarScanBuffer gpuLidarScanBuffer;
   private DoubleYoVariable tauLidarZ;
   private DoubleYoVariable tauLidarX;
   private DoubleYoVariable qLidarZ;
   private DoubleYoVariable qLidarX;

   private final YoFramePoint point = new YoFramePoint("point", ReferenceFrame.getWorldFrame(), registry);
   private final BagOfBalls bagOfBalls;

   private PDController pdControllerZ;
   private PDController pdControllerX;
   private final DoubleYoVariable proportionalGain;
   private final DoubleYoVariable derivativeGain;
   private double lastPositionZ;
   private double lastPositionX;
   private double lastTime;
   private double desiredZRate = 0.3;
   private final static double DESIRED_X_RATE = 0.3;

   public ExampleLidarController(ExampleLidarRobot robot, YoGraphicsListRegistry yoGraphicsListRegistry, SimulationConstructionSet scs)
   {
      this.robot = robot;
      this.scs = scs;
      this.lidarScanParameters = robot.getLidarScanParameters();

      YoGraphicPosition yoGraphicPosition = new YoGraphicPosition("point", point, 0.01, YoAppearance.Purple());
      yoGraphicsListRegistry.registerYoGraphic("test", yoGraphicPosition);

      bagOfBalls = new BagOfBalls(lidarScanParameters.getPointsPerSweep(), 0.005, YoAppearance.AliceBlue(), registry, yoGraphicsListRegistry);

      tauLidarZ = (DoubleYoVariable) robot.getVariable("tau_gimbalZ");
      tauLidarX = (DoubleYoVariable) robot.getVariable("tau_gimbalX");

      qLidarZ = (DoubleYoVariable) robot.getVariable("q_gimbalZ");
      qLidarX = (DoubleYoVariable) robot.getVariable("q_gimbalX");

      proportionalGain = new DoubleYoVariable("lidarPGain", registry);
      derivativeGain = new DoubleYoVariable("lidarDGain", registry);

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
      gpuLidar = scs.getGraphics3dAdapter().createGPULidar(lidarScanParameters);
      gpuLidarScanBuffer = new GPULidarScanBuffer(lidarScanParameters);
      gpuLidar.addGPULidarListener(gpuLidarScanBuffer);
   }

   public YoVariableRegistry getYoVariableRegistry()
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
            bagOfBalls.setBallLoop(new FramePoint(ReferenceFrame.getWorldFrame(), scan.getPoint(i)), YoAppearance.randomColor(random));
         }
      }
   }
}
