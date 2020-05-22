package us.ihmc.footstepPlanning;

import us.ihmc.commons.RandomNumbers;
import us.ihmc.footstepPlanning.graphSearch.parameters.*;
import us.ihmc.tools.property.BooleanStoredPropertyKey;
import us.ihmc.tools.property.DoubleStoredPropertyKey;
import us.ihmc.tools.property.IntegerStoredPropertyKey;
import us.ihmc.tools.property.StoredPropertyKey;
import us.ihmc.yoVariables.providers.DoubleProvider;

import java.util.Random;

import static us.ihmc.robotics.Assert.*;

public class FootstepPlanningTestTools
{
   private final static double epsilon = 1e-7;

   public static FootstepPlannerParametersReadOnly createRandomParameters(Random random)
   {

      FootstepPlannerParametersBasics parameters = new DefaultFootstepPlannerParameters();
      for (int i = 0; i < FootstepPlannerParameterKeys.keys.keys().size(); i++)
      {
         StoredPropertyKey<?> key = FootstepPlannerParameterKeys.keys.keys().get(i);
         if (key.getType() == Double.class)
            parameters.set((DoubleStoredPropertyKey) key, RandomNumbers.nextDouble(random, 0.01, 1.0));
         else if (key.getType() == Integer.class)
            parameters.set((IntegerStoredPropertyKey) key, RandomNumbers.nextInt(random, 1, 10));
         else if (key.getType() == Boolean.class)
            parameters.set((BooleanStoredPropertyKey) key, RandomNumbers.nextBoolean(random, 0.5));
      }


      return parameters;
   }
}
