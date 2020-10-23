package us.ihmc.humanoidBehaviors.tools;

import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.robotEnvironmentAwareness.geometry.ConcaveHullFactoryParameters;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PolygonizerParameters;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

/**
 * CPU implementation for lidar scans
 *
 * another option would be to create SCS instance with lidar
 */
public class SimulatedLidar
{
   private final ReferenceFrame cameraFrame;

   private final FramePose3D tempCameraPose = new FramePose3D();
   private final FramePose3D tempFramePose3D = new FramePose3D();

   private final double fov;
   private final double range;
   private final int scanSize;
   private final double angularVelocity;

   private final ConcaveHullFactoryParameters concaveHullFactoryParameters = new ConcaveHullFactoryParameters();
   private final PolygonizerParameters polygonizerParameters = new PolygonizerParameters();
   private HashMap<PlanarRegion, List<Point3D>> pointsInRegions = new HashMap<>();

   public SimulatedLidar(double range,
                         double fov,
                         int scanSize,
                         double angularVelocity,
                         ReferenceFrame cameraFrame)
   {
      this.range = range;
      this.fov = fov;
      this.scanSize = scanSize;
      this.angularVelocity = angularVelocity;
      this.cameraFrame = cameraFrame;




   }

   public PlanarRegionsList filterMapToVisible(PlanarRegionsList map)
   {

      pointsInRegions.clear();

      for (PlanarRegion planarRegion : map.getPlanarRegionsAsList())
      {
         pointsInRegions.put(planarRegion, new ArrayList<>());
      }

      tempCameraPose.setToZero(cameraFrame);
      tempCameraPose.changeFrame(ReferenceFrame.getWorldFrame());

      return new PlanarRegionsList();
   }
}
