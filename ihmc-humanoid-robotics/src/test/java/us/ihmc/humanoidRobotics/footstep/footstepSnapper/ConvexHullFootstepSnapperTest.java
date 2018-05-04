package us.ihmc.humanoidRobotics.footstep.footstepSnapper;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.Vertex2DSupplier;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.robotics.random.RandomGeometry;


/**
 * Created by agrabertilton on 1/20/15.
 */
public class ConvexHullFootstepSnapperTest
{

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
	public void testBasicCropping()
	{
      QuadTreeFootstepSnappingParameters snappingParameters = new GenericFootstepSnappingParameters();
      ConvexHullFootstepSnapper footstepSnapper = new ConvexHullFootstepSnapper(new SimpleFootstepValueFunction(snappingParameters), snappingParameters);
      List<Point2D> pointsToCrop = new ArrayList<Point2D>();
      pointsToCrop.add(new Point2D(1,1));
      pointsToCrop.add(new Point2D(-1,1));
      pointsToCrop.add(new Point2D(-1,-1));
      pointsToCrop.add(new Point2D(1,-1));
      pointsToCrop.add(new Point2D(1.1,0));

      List<Point2D> finalPoints = footstepSnapper.reduceListOfPointsByArea(pointsToCrop, 4);
      assertTrue(finalPoints.size() == 4.0);
      ConvexPolygon2D endPolygon = new ConvexPolygon2D(Vertex2DSupplier.asVertex2DSupplier(finalPoints));
      assertEquals(4.0, endPolygon.getArea(), 1e-15);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
	public void testRandomCropping()
	{
      QuadTreeFootstepSnappingParameters snappingParameters = new GenericFootstepSnappingParameters();
      ConvexHullFootstepSnapper footstepSnapper = new ConvexHullFootstepSnapper(new SimpleFootstepValueFunction(snappingParameters), snappingParameters);
      List<Point2D> pointsToCrop = new ArrayList<Point2D>();
      Random random = new Random(82368L);
      double maxX = 10;
      double maxY = 10;
      int numPoints = 100;
      for (int i = 0; i < numPoints; i++){
         pointsToCrop.add(RandomGeometry.nextPoint2D(random, maxX, maxY));
      }

      ConvexPolygon2D startPolygon = new ConvexPolygon2D(Vertex2DSupplier.asVertex2DSupplier(pointsToCrop));
      startPolygon.update();
      double startArea = startPolygon.getArea();

      ConvexPolygon2D intermediateStepPolygon = new ConvexPolygon2D(Vertex2DSupplier.asVertex2DSupplier(footstepSnapper.reduceListOfPointsByArea(pointsToCrop, Math.max(4, startPolygon.getNumberOfVertices() / 2))));
      intermediateStepPolygon.update();
      double intermediateStepArea = intermediateStepPolygon.getArea();
      assertTrue(intermediateStepArea <= startArea);
      assertTrue(intermediateStepPolygon.getNumberOfVertices() <= Math.max(4, startPolygon.getNumberOfVertices()));

      ConvexPolygon2D endPolygon = new ConvexPolygon2D(Vertex2DSupplier.asVertex2DSupplier(footstepSnapper.reduceListOfPointsByArea(pointsToCrop, 4)));
      endPolygon.update();
      double endArea = endPolygon.getArea();
      assertTrue(endArea <= startArea);
      assertTrue(endArea <= intermediateStepArea);
      assertTrue(endPolygon.getNumberOfVertices() <= 4);
   }
}
