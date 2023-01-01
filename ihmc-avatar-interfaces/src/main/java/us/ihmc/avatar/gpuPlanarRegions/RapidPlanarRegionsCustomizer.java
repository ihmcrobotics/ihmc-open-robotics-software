package us.ihmc.avatar.gpuPlanarRegions;

import us.ihmc.commons.MathTools;
import us.ihmc.commons.time.Stopwatch;
import us.ihmc.euclid.exceptions.NotARotationMatrixException;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.LineSegment2D;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.matrix.LinearTransform3D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.log.LogTools;
import us.ihmc.robotEnvironmentAwareness.geometry.*;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PolygonizerParameters;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PolygonizerTools;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsListWithPose;


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
      concaveHullFactoryParameters = new ConcaveHullFactoryParameters("ForGPURegions");
      polygonizerParameters = new PolygonizerParameters("ForGPURegions");
   }

   public RapidPlanarRegionsCustomizer(ConcaveHullFactoryParameters concaveHullFactoryParameters, PolygonizerParameters polygonizerParameters)
   {
      this.concaveHullFactoryParameters = concaveHullFactoryParameters;
      this.polygonizerParameters = polygonizerParameters;
   }

   public List<PlanarRegion> convertToPlanarRegion(GPUPlanarRegion rapidPlanarRegion, ReferenceFrame cameraFrame)
   {
      List<PlanarRegion> planarRegions = new ArrayList<>();
      FrameQuaternion orientation = new FrameQuaternion();
      try
      {
         // Going through LinearTransform3D first prevents NotARotationMatrix exceptions.
         LinearTransform3D linearTransform3D = new LinearTransform3D(EuclidGeometryTools.axisAngleFromZUpToVector3D(rapidPlanarRegion.getNormal()));
         linearTransform3D.normalize();
         orientation.setIncludingFrame(cameraFrame, linearTransform3D.getAsQuaternion());
         orientation.changeFrame(ReferenceFrame.getWorldFrame());

         if (!MathTools.epsilonEquals(rapidPlanarRegion.getNormal().norm(), 1.0, 1e-4))
            throw new RuntimeException("The planar region norm isn't valid");

         FramePoint3D origin = new FramePoint3D(cameraFrame, rapidPlanarRegion.getCenter());
         origin.changeFrame(ReferenceFrame.getWorldFrame());

         List<LineSegment2D> intersections = new ArrayList<>();
         List<Point2D> pointCloudInPlane = getPointCloudInPlane(rapidPlanarRegion, cameraFrame, origin, orientation);
         ConcaveHullCollection concaveHullCollection = SimpleConcaveHullFactory.createConcaveHullCollection(pointCloudInPlane,
                                                                                                            intersections,
                                                                                                            concaveHullFactoryParameters);

         //applyConcaveHullFilters(concaveHullCollection);
         createPlanarRegion(rapidPlanarRegion.getId(), origin, orientation, concaveHullCollection, planarRegions);

      }
      catch (NotARotationMatrixException notARotationMatrixException)
      {
         LogTools.info("Normal = " + rapidPlanarRegion.getNormal().toString(null));
         LogTools.info("Orientation = " + orientation.toString(null));
         LogTools.warn("Not a rotation matrix: {}", rapidPlanarRegion.getNormal());
      }
      catch (RuntimeException e)
      {
         e.printStackTrace();
      }
      return planarRegions;
   }

   private List<Point2D> getPointCloudInPlane(GPUPlanarRegion rapidRegion, ReferenceFrame cameraFrame, FramePoint3D origin, FrameQuaternion orientation)
   {
      return rapidRegion.getBoundaryVertices()
                              .stream()
                              .map(boundaryVertex ->
                                   {
                                      FramePoint3D framePoint3D = new FramePoint3D(cameraFrame,
                                                                                   boundaryVertex);
                                      framePoint3D.changeFrame(ReferenceFrame.getWorldFrame());
                                      return PolygonizerTools.toPointInPlane(framePoint3D,
                                                                             origin,
                                                                             orientation);
                                   })
                              .filter(point2D -> Double.isFinite(point2D.getX()) && Double.isFinite(point2D.getY()))
                              .collect(Collectors.toList());
   }

   public void applyConcaveHullFilters(ConcaveHullCollection concaveHulls)
   {
      // Apply some simple filtering to reduce the number of vertices and hopefully the number of convex polygons.
      double shallowAngleThreshold = polygonizerParameters.getShallowAngleThreshold();
      double peakAngleThreshold = polygonizerParameters.getPeakAngleThreshold();
      double lengthThreshold = polygonizerParameters.getLengthThreshold();

      ConcaveHullPruningFilteringTools.filterOutPeaksAndShallowAngles(shallowAngleThreshold, peakAngleThreshold, concaveHulls);
      ConcaveHullPruningFilteringTools.filterOutShortEdges(lengthThreshold, concaveHulls);
      if (polygonizerParameters.getCutNarrowPassage())
         concaveHulls = ConcaveHullPruningFilteringTools.concaveHullNarrowPassageCutter(lengthThreshold, concaveHulls);
   }

   public void createCustomPlanarRegionsList(List<GPUPlanarRegion> gpuPlanarRegions, ReferenceFrame cameraFrame, PlanarRegionsListWithPose regionsToPack)
   {
      stopWatch.start();
      RigidBodyTransform sensorToWorldFrameTransform = new RigidBodyTransform();
      AtomicBoolean listCaughtException = new AtomicBoolean(false);

      List<List<PlanarRegion>> listOfListsOfRegions = gpuPlanarRegions.parallelStream()
                                                                      .filter(gpuPlanarRegion -> gpuPlanarRegion.getBoundaryVertices().size()
                                                                                                 >= polygonizerParameters.getMinNumberOfNodes())
                                                                      .map(gpuPlanarRegion -> convertToPlanarRegion(gpuPlanarRegion, cameraFrame))
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

   public void createPlanarRegion(int regionId, FramePoint3D origin, FrameQuaternion orientation, ConcaveHullCollection concaveHullCollection,
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
         FramePose3D regionPose = new FramePose3D();
         regionPose.setIncludingFrame(ReferenceFrame.getWorldFrame(), origin, orientation);
         RigidBodyTransform tempTransform = new RigidBodyTransform();
         regionPose.get(tempTransform);
         PlanarRegion planarRegion = new PlanarRegion(tempTransform, concaveHull.getConcaveHullVertices(), decomposedPolygons);
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
