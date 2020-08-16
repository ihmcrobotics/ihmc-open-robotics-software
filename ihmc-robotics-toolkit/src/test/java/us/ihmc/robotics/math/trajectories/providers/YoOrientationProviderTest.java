package us.ihmc.robotics.math.trajectories.providers;

import static us.ihmc.robotics.Assert.assertEquals;
import static us.ihmc.robotics.Assert.assertSame;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameYawPitchRoll;
import us.ihmc.yoVariables.registry.YoRegistry;

public class YoOrientationProviderTest
{
   private static final double EPSILON = 1e-10;

   private String namePrefix = "namePrefix";
   private ReferenceFrame referenceFrame;
   private YoRegistry registry;
   private YoFrameYawPitchRoll yoFrameOrientation;
   public FrameQuaternion frameOrientationToPack;

   @BeforeEach
   public void setUp()
   {
      referenceFrame = ReferenceFrameTools.constructARootFrame("rootFrame");
      registry = new YoRegistry("yoVariableRegistry");
      yoFrameOrientation = new YoFrameYawPitchRoll(namePrefix, referenceFrame, registry);
      frameOrientationToPack = new FrameQuaternion();
   }

   @AfterEach
   public void tearDown()
   {
      ReferenceFrameTools.clearWorldFrameTree();
   }

   @Test
   public void testConstructor_Get()
   {
      YoOrientationProvider yoOrientationProvider = new YoOrientationProvider(yoFrameOrientation);
      yoOrientationProvider.getOrientation(frameOrientationToPack);

      YawPitchRoll yawPitchRollToPack = new YawPitchRoll(yoFrameOrientation);

      YawPitchRoll yawPitchRollToPack2 = new YawPitchRoll(frameOrientationToPack);

      for (int i = 0; i < 3; i++)
      {
         assertEquals(yawPitchRollToPack.getElement(i), yawPitchRollToPack2.getElement(i), EPSILON);
      }

      assertSame(yoFrameOrientation.getReferenceFrame(), frameOrientationToPack.getReferenceFrame());
   }
}
