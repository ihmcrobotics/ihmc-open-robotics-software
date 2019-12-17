package us.ihmc.pathPlanning.graph.search;

import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;

public class AStarIterationDataTest
{
   @Test
   public void testConstructor()
   {
      AStarIterationData<Object> data = new AStarIterationData<>();
      Assertions.assertTrue(data.getValidChildNodes().isEmpty());
      Assertions.assertTrue(data.getInvalidChildNodes().isEmpty());
      Assertions.assertNull(data.getParentNode());
   }

   @Test
   public void testClear()
   {
      AStarIterationData<Object> data = new AStarIterationData<>();
      data.setParentNode(new Object());
      data.getValidChildNodes().add(new Object());
      data.getInvalidChildNodes().add(new Object());
      data.clear();

      Assertions.assertTrue(data.getValidChildNodes().isEmpty());
      Assertions.assertTrue(data.getInvalidChildNodes().isEmpty());
      Assertions.assertNull(data.getParentNode());
   }
}
