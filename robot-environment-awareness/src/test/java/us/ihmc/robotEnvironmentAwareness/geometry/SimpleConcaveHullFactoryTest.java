package us.ihmc.robotEnvironmentAwareness.geometry;

import static org.junit.Assert.*;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.stream.Collectors;

import org.junit.Test;

import us.ihmc.euclid.Axis;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.LineSegment3D;
import us.ihmc.euclid.geometry.interfaces.Vertex2DSupplier;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PlanarRegionSegmentationRawData;

public class SimpleConcaveHullFactoryTest
{
   @Test
   public void testSimplePointcloudFormingASquare()
   {
      List<Point3D> pointcloud = new ArrayList<>();

      double xOffset = 0.4;
      double yOffset = 0.0;

      double size = 0.1;
      double density = 0.005;

      for (int i = 0; i < size / density; i++)
      {
         for (int j = 0; j < size / density; j++)
         {
            double x = i * density + xOffset;
            double y = j * density + yOffset;
            pointcloud.add(new Point3D(x, y, 0.0));
         }
      }

      PlanarRegionSegmentationRawData data = new PlanarRegionSegmentationRawData(1, Axis.Z, new Point3D(), pointcloud);
      ConcaveHullFactoryParameters parameters = new ConcaveHullFactoryParameters();
      parameters.setTriangulationTolerance(1.0e-3);
      parameters.setEdgeLengthThreshold(1.1 * density);
      ConcaveHullCollection concaveHullCollection = SimpleConcaveHullFactory.createConcaveHullCollection(data.getPointCloudInPlane(),
                                                                                                         data.getIntersectionsInPlane(), parameters);

      assertEquals(1, concaveHullCollection.getNumberOfConcaveHulls());
      ConcaveHull concaveHull = new ArrayList<>(concaveHullCollection.getConcaveHulls()).get(0);

      ConvexPolygon2D convexPolygon = new ConvexPolygon2D(Vertex2DSupplier.asVertex2DSupplier(concaveHull.getConcaveHullVertices()));

      for (int i = 0; i < concaveHull.getNumberOfVertices(); i++)
      {
         assertEquals(0.0, convexPolygon.signedDistance(concaveHull.getVertex(i)), 1.0e-7);
      }
   }

   @Test
   public void testOverlappingLineConstraints()
   {
      List<Point3D> pointcloud = new ArrayList<>();

      double xOffset = 0.4;
      double yOffset = 0.0;

      double size = 0.1;
      double density = 0.005;

      for (int i = 0; i < size / density; i++)
      {
         for (int j = 0; j < size / density; j++)
         {
            double x = i * density + xOffset;
            double y = j * density + yOffset;
            pointcloud.add(new Point3D(x, y, 0.0));
         }
      }

      PlanarRegionSegmentationRawData data = new PlanarRegionSegmentationRawData(1, Axis.Z, new Point3D(), pointcloud);

      List<LineSegment3D> polygon1 = Arrays.asList(new LineSegment3D(0.25, -0.025, 0, 0.25, -0.20, 0), new LineSegment3D(0.35, -0.025, 0, 0.35, -0.20, 0),
                                                   new LineSegment3D(0.25, -0.025, 0, 0.35, -0.025, 0), new LineSegment3D(0.25, -0.20, 0, 0.35, -0.20, 0));

      List<LineSegment3D> polygon2 = polygon1.stream().map(LineSegment3D::new).peek(segment -> segment.translate(0.09, 0.005, 0.0))
                                             .collect(Collectors.toList());

      data.addIntersections(polygon1);
      data.addIntersections(polygon2);

      ConcaveHullFactoryParameters parameters = new ConcaveHullFactoryParameters();
      ConcaveHullCollection concaveHullCollection = SimpleConcaveHullFactory.createConcaveHullCollection(data.getPointCloudInPlane(),
                                                                                                         data.getIntersectionsInPlane(), parameters);

      assertEquals(1, concaveHullCollection.getNumberOfConcaveHulls());

   }

}
