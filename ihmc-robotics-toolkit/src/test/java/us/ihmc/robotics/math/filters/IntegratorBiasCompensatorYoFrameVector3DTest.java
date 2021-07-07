package us.ihmc.robotics.math.filters;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;

public class IntegratorBiasCompensatorYoFrameVector3DTest
{
   @Test
   public void testCompareAgainst1DFilter()
   {
      Random random = new Random(34);

      double kp = 0.1;
      double ki = 0.01;
      double dt = 0.001;
      YoFrameVector3D position = new YoFrameVector3D("position", ReferenceFrame.getWorldFrame(), null);
      YoFrameVector3D rate = new YoFrameVector3D("rate", ReferenceFrame.getWorldFrame(), null);

      IntegratorBiasCompensatorYoVariable[] oneDimensionFilters = new IntegratorBiasCompensatorYoVariable[3];
      for (int axis = 0; axis < 3; axis++)
      {
         int axisFinal = axis;
         oneDimensionFilters[axis] = new IntegratorBiasCompensatorYoVariable("filter1D[" + axis
               + "]", null, kp, ki, () -> position.getElement(axisFinal), () -> rate.getElement(axisFinal), dt);
      }

      IntegratorBiasCompensatorYoFrameVector3D threeDimensionFilter = new IntegratorBiasCompensatorYoFrameVector3D("filter3D",
                                                                                                                   null,
                                                                                                                   kp,
                                                                                                                   ki,
                                                                                                                   position,
                                                                                                                   rate,
                                                                                                                   dt);

      for (int i = 0; i < 5000; i++)
      {
         rate.set(EuclidCoreRandomTools.nextPoint3D(random, 10));
         position.set(EuclidCoreRandomTools.nextPoint3D(random, 10));

         for (IntegratorBiasCompensatorYoVariable oneDimensionFilter : oneDimensionFilters)
         {
            oneDimensionFilter.update();
         }

         threeDimensionFilter.update();

         double epsilon = 1.0e-12;

         for (int axis = 0; axis < 3; axis++)
         {
            IntegratorBiasCompensatorYoVariable oneDimensionFilter = oneDimensionFilters[axis];

            assertEquals(oneDimensionFilter.getValue(), threeDimensionFilter.getElement(axis), epsilon);
            assertEquals(oneDimensionFilter.getBiasEstimation().getValue(), threeDimensionFilter.getBiasEstimation().getElement(axis), epsilon);
            assertEquals(oneDimensionFilter.getRateEstimation().getValue(), threeDimensionFilter.getRateEstimation().getElement(axis), epsilon);
         }
      }
   }
}
