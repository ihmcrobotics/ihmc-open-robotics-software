package us.ihmc.humanoidOperatorInterface.footstep.footstepSnapper;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.Vertex2DSupplier;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tuple2D.Point2D;

import static org.junit.jupiter.api.Assertions.*;

/**
 * Created by agrabertilton on 1/20/15.
 */
public class ConvexHullFootstepSnapperTest
{

	@Test
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

	@Test
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
         pointsToCrop.add(EuclidCoreRandomTools.nextPoint2D(random, maxX, maxY));
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
