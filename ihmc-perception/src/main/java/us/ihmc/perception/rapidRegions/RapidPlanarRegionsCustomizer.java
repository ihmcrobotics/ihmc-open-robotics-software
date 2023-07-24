package us.ihmc.perception.rapidRegions;

import us.ihmc.commons.time.Stopwatch;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.LineSegment2D;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.robotEnvironmentAwareness.geometry.*;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PolygonizerParameters;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PolygonizerTools;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.FramePlanarRegionsList;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.stream.Collectors;

public class RapidPlanarRegionsCustomizer
{
   private final ConcaveHullFactoryParameters concaveHullFactoryParameters;
   private final PolygonizerParameters polygonizerParameters;
   private final Stopwatch stopWatch = new Stopwatch();

   public RapidPlanarRegionsCustomizer()
   {
      this("");
   }

   public RapidPlanarRegionsCustomizer(String version)
   {
      concaveHullFactoryParameters = new ConcaveHullFactoryParameters(version);
      polygonizerParameters = new PolygonizerParameters(version);
   }

   public List<PlanarRegion> convertToPlanarRegions(RapidPlanarRegion rapidRegion)
   {
      List<PlanarRegion> planarRegions = new ArrayList<>();
      List<LineSegment2D> intersections = new ArrayList<>();

      Quaternion orientation = new Quaternion();
      EuclidGeometryTools.axisAngleFromZUpToVector3D(rapidRegion.getNormal()).get(orientation);

      // Get boundary vertices in the local plane frame
      List<Point2D> pointCloudInPlane = getPointCloudInPlane(rapidRegion, rapidRegion.getCenter(), orientation);

      // Get the concave hull(s) of the boundary vertices
      ConcaveHullCollection concaveHullCollection = SimpleConcaveHullFactory.createConcaveHullCollection(pointCloudInPlane,
                                                                                                         intersections,
                                                                                                         concaveHullFactoryParameters);

      // Create and insert planar region(s) into the list
      RigidBodyTransform regionTransformToWorld = new RigidBodyTransform(orientation, rapidRegion.getCenter());
      createAndInsertPlanarRegions(rapidRegion.getId(), regionTransformToWorld, concaveHullCollection, planarRegions);

      return planarRegions;
   }

   private List<Point2D> getPointCloudInPlane(RapidPlanarRegion rapidRegion, Point3D origin, Quaternion orientation)
   {
      return rapidRegion.getBoundaryVertices()
                        .stream()
                        .map(boundaryVertex -> PolygonizerTools.toPointInPlane(new Point3D(boundaryVertex),
                                                               origin,
                                                               orientation))
                        .filter(point2D -> Double.isFinite(point2D.getX()) && Double.isFinite(point2D.getY()))
                        .collect(Collectors.toList());
   }

   public void createCustomPlanarRegionsList(List<RapidPlanarRegion> rapidPlanarRegions, ReferenceFrame cameraFrame, FramePlanarRegionsList regionsToPack)
   {
      stopWatch.start();
      RigidBodyTransform sensorToWorldFrameTransform = new RigidBodyTransform();
      AtomicBoolean listCaughtException = new AtomicBoolean(false);

      List<List<PlanarRegion>> listOfListsOfRegions = rapidPlanarRegions.parallelStream()
                                                                      .map(this::convertToPlanarRegions)
                                                                      .toList();
      regionsToPack.getPlanarRegionsList().clear();
      if (!listCaughtException.get())
      {
         for (List<PlanarRegion> planarRegions : listOfListsOfRegions)
         {
            regionsToPack.getPlanarRegionsList().addPlanarRegions(planarRegions);
         }
      }
      sensorToWorldFrameTransform.set(cameraFrame.getTransformToWorldFrame());
      regionsToPack.setSensorToWorldFrameTransform(sensorToWorldFrameTransform);
      stopWatch.suspend();
   }

   public void createAndInsertPlanarRegions(int regionId, RigidBodyTransform transformToWorld, ConcaveHullCollection concaveHullCollection,
                                            List<PlanarRegion> planarRegions)
   {
      int hullCounter = 0;

      for (ConcaveHull concaveHull : concaveHullCollection)
      {
         if (concaveHull.isEmpty())
            continue;

         // Decompose the concave hulls into convex polygons
         double depthThreshold = polygonizerParameters.getDepthThreshold();
         List<ConvexPolygon2D> decomposedPolygons = new ArrayList<>();
         ConcaveHullDecomposition.recursiveApproximateDecomposition(concaveHull, depthThreshold, decomposedPolygons);

         // Pack the data in PlanarRegion
         PlanarRegion planarRegion = new PlanarRegion(transformToWorld, concaveHull.getConcaveHullVertices(), decomposedPolygons);
         planarRegion.setRegionId(regionId);
         planarRegions.add(planarRegion);

         hullCounter++;
         regionId = 31 * regionId + hullCounter;
      }
   }

   public ConcaveHullFactoryParameters getConcaveHullFactoryParameters()
   {
      return concaveHullFactoryParameters;
   }

   public PolygonizerParameters getPolygonizerParameters()
   {
      return polygonizerParameters;
   }

   public Stopwatch getStopWatch()
   {
      return stopWatch;
   }
}
