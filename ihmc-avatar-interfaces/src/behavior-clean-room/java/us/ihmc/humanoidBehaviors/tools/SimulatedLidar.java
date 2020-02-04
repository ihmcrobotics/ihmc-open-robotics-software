package us.ihmc.humanoidBehaviors.tools;

import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.robotics.geometry.PlanarRegionsList;

public class SimulatedLidar
{
   private final ReferenceFrame cameraFrame;

   private final FramePose3D tempFramePose3D = new FramePose3D();

   private FramePose3D tempCameraPose = new FramePose3D();

   public SimulatedLidar(ReferenceFrame cameraFrame)
   {
      this.cameraFrame = cameraFrame;

      // TODO create SCS instance with lidar
   }

   public PlanarRegionsList filterMapToVisible(PlanarRegionsList map)
   {
      return new PlanarRegionsList();
   }
}
