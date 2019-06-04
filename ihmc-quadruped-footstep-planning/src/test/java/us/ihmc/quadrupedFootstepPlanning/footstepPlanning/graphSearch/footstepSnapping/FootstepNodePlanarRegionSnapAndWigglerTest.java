package us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.footstepSnapping;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import us.ihmc.commonWalkingControlModules.polygonWiggling.PolygonWiggler;
import us.ihmc.commonWalkingControlModules.polygonWiggling.WiggleParameters;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.LineSegment2D;
import us.ihmc.euclid.geometry.interfaces.LineSegment2DBasics;
import us.ihmc.euclid.geometry.tools.EuclidGeometryPolygonTools;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DBasics;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.pathPlanning.DataSet;
import us.ihmc.pathPlanning.DataSetIOTools;
import us.ihmc.robotics.geometry.PlanarRegion;

class FootstepNodePlanarRegionSnapAndWigglerTest
{

   @Test
   void testIssue20190603_WigglerViolateDeltaInside()
   {
      DataSet dataSet = DataSetIOTools.loadDataSet("20190603_224047_PlanarRegionWigglerViolateDeltaInside");
      
      PlanarRegion troublesomeRegion = dataSet.getPlanarRegionsList().getPlanarRegionsAsList().stream().filter(region -> region.isPointInside(new Point3D(-0.12, -0.51, -0.49), 0.01)).findFirst().get();

      ConvexPolygon2D planeToWiggleInto = troublesomeRegion.getConvexHull();

      for (int vertexIndex = 0; vertexIndex < planeToWiggleInto.getNumberOfVertices(); vertexIndex++)
      {
         for (double alpha = 0; alpha <= 1.0; alpha += 0.05)
         {
            ConvexPolygon2D polygonToWiggle = new ConvexPolygon2D();
            Point2D pointOnEdge = new Point2D();
            pointOnEdge.interpolate(planeToWiggleInto.getVertex(vertexIndex), planeToWiggleInto.getNextVertex(vertexIndex), alpha);
            LineSegment2DBasics edge = new LineSegment2D();
            planeToWiggleInto.getEdge(vertexIndex, edge);
            Vector2DBasics normal = edge.direction(true);
            normal = EuclidGeometryTools.perpendicularVector2D(normal);
            normal.scale(5.0e-3);
            polygonToWiggle.addVertex(pointOnEdge);
            polygonToWiggle.update();
            WiggleParameters parameters = new WiggleParameters();
            parameters.deltaInside = 0.06;
            parameters.maxX = 0.05;
            parameters.maxY = 0.05;
            parameters.minX = -0.05;
            parameters.minY = -0.05;
            parameters.maxYaw = 0.0;
            parameters.minYaw = -0.0;
            
            ConvexPolygon2D wigglePolygon = PolygonWiggler.wigglePolygon(polygonToWiggle, planeToWiggleInto, parameters);
            
            assertTrue(planeToWiggleInto.signedDistance(wigglePolygon.getCentroid()) < -0.025);
//            assertEquals(-parameters.deltaInside, planeToWiggleInto.signedDistance(wigglePolygon.getCentroid()), 1.0e-3);
         }
      }
   }

}
