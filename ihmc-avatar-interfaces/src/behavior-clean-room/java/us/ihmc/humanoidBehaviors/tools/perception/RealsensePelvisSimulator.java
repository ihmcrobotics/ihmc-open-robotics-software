package us.ihmc.humanoidBehaviors.tools.perception;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.humanoidBehaviors.tools.RemoteSyncedRobotModel;
import us.ihmc.humanoidBehaviors.tools.SimulatedDepthCamera;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.referenceFrames.TransformReferenceFrame;
import us.ihmc.ros2.Ros2NodeInterface;

import java.util.function.Supplier;

public class RealsensePelvisSimulator implements Supplier<PlanarRegionsList>
{
   private final TransformReferenceFrame realsenseSensorFrame;
   private volatile PlanarRegionsList map;

   private RemoteSyncedRobotModel syncedRobot;
   private MovingReferenceFrame pelvisFrame;
   private SimulatedDepthCamera simulatedDepthCamera;

   private static final double depthOffsetX = 0.058611;
   private static final double depthOffsetZ = 0.01;
   private static final double depthPitchingAngle = 70.0 / 180.0 * Math.PI;
   private static final double depthThickness = 0.0245;
   private static final double pelvisToMountOrigin = 0.19;

   public static final RigidBodyTransform transform = new RigidBodyTransform();
   static
   {
      transform.appendTranslation(pelvisToMountOrigin, 0.0, 0.0);
      transform.appendTranslation(depthOffsetX, 0.0, depthOffsetZ);
      transform.appendPitchRotation(depthPitchingAngle);
      transform.appendTranslation(depthThickness, 0.0, 0.0);

      // Real robot Realsense D435 sensor has an additional frame change to convert to camera frame
      // which is Z forward instead of X forward. We omit that here because the SimulatedDepthCamera
      // algorithm operates if X forward frame (same as robot)
   }

   public RealsensePelvisSimulator(PlanarRegionsList map, DRCRobotModel robotModel, Ros2NodeInterface ros2Node)
   {
      this.map = map;

      syncedRobot = new RemoteSyncedRobotModel(robotModel, ros2Node);
      pelvisFrame = syncedRobot.getReferenceFrames().getPelvisFrame();

      realsenseSensorFrame = new TransformReferenceFrame("Realsense", pelvisFrame, transform);

      double verticalFOV = 80.0; // real fov is 58.0;
      double horizontalFOV = 87.0;
      double range = 1.5;
      simulatedDepthCamera = new SimulatedDepthCamera(verticalFOV, horizontalFOV, range, realsenseSensorFrame);
   }

   @Override
   public PlanarRegionsList get()
   {
      syncedRobot.update();

      if (syncedRobot.hasReceivedFirstMessage())
      {
         return simulatedDepthCamera.filterMapToVisible(map);
      }
      else
      {
         // blank result
         return new PlanarRegionsList();
      }
   }
}
