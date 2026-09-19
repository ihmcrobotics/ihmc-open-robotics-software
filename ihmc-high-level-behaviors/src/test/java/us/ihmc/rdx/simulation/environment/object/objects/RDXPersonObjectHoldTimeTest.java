package us.ihmc.rdx.simulation.environment.object.objects;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;

public class RDXPersonObjectHoldTimeTest
{
   @Test
   public void mixamoClipsFreezeInTheHoldNotOnTheLastFrame()
   {
      assertEquals(1.25f, RDXPersonObject.holdTimeSeconds(2.5f), 1.0e-4f,
                   "point / stop raise until ~0.67s and lower after ~1.92s; 50% is the hold");
   }
}
