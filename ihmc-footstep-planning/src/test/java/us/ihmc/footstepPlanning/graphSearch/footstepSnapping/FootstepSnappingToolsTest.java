package us.ihmc.footstepPlanning.graphSearch.footstepSnapping;

import org.junit.jupiter.api.Test;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.tools.EuclidCoreTestTools;

public class FootstepSnappingToolsTest
{
   @Test
   public void testPolygonReduction()
   {
      ConvexPolygon2D square = new ConvexPolygon2D();

      // test no reduction
      square.addVertex(-0.1, 0.1);
      square.addVertex(-0.1, -0.1);
      square.addVertex(0.1, -0.1);
      square.addVertex(0.1, 0.1);
      square.update();

      ConvexPolygon2D reducedSquare = FootstepSnappingTools.reduceNumberOfVertices(square, 4);

      EuclidCoreTestTools.assertEquals(square, reducedSquare, 1e-5);


      // test pentagon that should reduce to a square.
      ConvexPolygon2D pentagon = new ConvexPolygon2D();
      pentagon.addVertex(-0.1, 0.1);
      pentagon.addVertex(0.075, 0.125);
      pentagon.addVertex(0.125, 0.075);
      pentagon.addVertex(0.1, -0.1);
      pentagon.addVertex(-0.1, -0.1);
      pentagon.update();

      reducedSquare = FootstepSnappingTools.reduceNumberOfVertices(pentagon, 4);
      EuclidCoreTestTools.assertEquals(square, reducedSquare, 1e-5);
   }
}
