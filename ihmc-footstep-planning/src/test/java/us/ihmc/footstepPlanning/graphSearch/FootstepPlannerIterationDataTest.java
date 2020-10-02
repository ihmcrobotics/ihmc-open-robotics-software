package us.ihmc.footstepPlanning.graphSearch;

import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;

public class FootstepPlannerIterationDataTest
{
   @Test
   public void testConstructor()
   {
      FootstepPlannerIterationData<Object> data = new FootstepPlannerIterationData<>();
      Assertions.assertTrue(data.getValidChildNodes().isEmpty());
      Assertions.assertTrue(data.getInvalidChildNodes().isEmpty());
      Assertions.assertNull(data.getParentNode());
   }

   @Test
   public void testClear()
   {
      FootstepPlannerIterationData<Object> data = new FootstepPlannerIterationData<>();
      data.setParentNode(new Object());
      data.getValidChildNodes().add(new Object());
      data.getInvalidChildNodes().add(new Object());
      data.clear();

      Assertions.assertTrue(data.getValidChildNodes().isEmpty());
      Assertions.assertTrue(data.getInvalidChildNodes().isEmpty());
      Assertions.assertNull(data.getParentNode());
   }
}
