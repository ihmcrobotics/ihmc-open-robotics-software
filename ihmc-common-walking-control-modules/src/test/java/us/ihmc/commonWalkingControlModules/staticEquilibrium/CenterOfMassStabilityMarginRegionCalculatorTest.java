package us.ihmc.commonWalkingControlModules.staticEquilibrium;

import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;

import static us.ihmc.commonWalkingControlModules.staticEquilibrium.CenterOfMassStabilityMarginRegionCalculator.*;

public class CenterOfMassStabilityMarginRegionCalculatorTest
{
   @Test
   public void testIndexing()
   {
      for (int index = 0; index < DIRECTIONS_TO_OPTIMIZE; index++)
      {
         String errorMessage = "Invalid index correspondence";

         Assertions.assertTrue(index == getVertexBOfEdge(getEdgeAOfVertex(index)), errorMessage);
         Assertions.assertTrue(index == getVertexAOfEdge(getEdgeBOfVertex(index)), errorMessage);

         Assertions.assertTrue(index == getEdgeAOfVertex(getVertexBOfEdge(index)), errorMessage);
         Assertions.assertTrue(index == getEdgeBOfVertex(getVertexAOfEdge(index)), errorMessage);

         int previousVertex = getVertexAOfEdge(getEdgeAOfVertex(index));
         int previousEdge = getEdgeAOfVertex(getVertexAOfEdge(index));
         int nextVertex = getVertexBOfEdge(getEdgeBOfVertex(index));
         int nextEdge = getEdgeBOfVertex(getVertexBOfEdge(index));

         if (index == 0)
         {
            Assertions.assertTrue(previousVertex == DIRECTIONS_TO_OPTIMIZE - 1, errorMessage);
            Assertions.assertTrue(previousEdge == DIRECTIONS_TO_OPTIMIZE - 1, errorMessage);
         }
         else
         {
            Assertions.assertTrue(previousVertex == index - 1, errorMessage);
            Assertions.assertTrue(previousEdge == index - 1, errorMessage);
         }

         if (index == DIRECTIONS_TO_OPTIMIZE - 1)
         {
            Assertions.assertTrue(nextVertex == 0, errorMessage);
            Assertions.assertTrue(nextEdge == 0, errorMessage);
         }
         else
         {
            Assertions.assertTrue(nextVertex == index + 1, errorMessage);
            Assertions.assertTrue(nextEdge == index + 1, errorMessage);
         }
      }
   }
}
