package us.ihmc.robotics.math.trajectories.providers;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertSame;

import org.junit.Before;
import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.math.frames.YoFrameOrientation;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;


public class YoOrientationProviderTest
{
   private static final double EPSILON = 1e-10;
   
   private String namePrefix = "namePrefix";
   private ReferenceFrame referenceFrame;
   private YoVariableRegistry registry; 
   private YoFrameOrientation yoFrameOrientation;
   public FrameOrientation frameOrientationToPack;
   
   @Before
   public void setUp()
   {
      referenceFrame =ReferenceFrame.constructARootFrame("rootFrame");
      registry = new YoVariableRegistry("yoVariableRegistry");
      yoFrameOrientation = new YoFrameOrientation(namePrefix, referenceFrame, registry);
      frameOrientationToPack = new FrameOrientation();
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testConstructor_Get()
   {
      YoOrientationProvider yoOrientationProvider = new YoOrientationProvider(yoFrameOrientation);
      yoOrientationProvider.getOrientation(frameOrientationToPack);
      
      double[] yawPitchRollToPack = new double[3];
      yoFrameOrientation.getYawPitchRoll(yawPitchRollToPack);
      
      double[] yawPitchRollToPack2 = new double[3];
      frameOrientationToPack.getYawPitchRoll(yawPitchRollToPack2);
      
      for(int i = 0; i < yawPitchRollToPack.length; i++)
      {
         assertEquals(yawPitchRollToPack[i], yawPitchRollToPack2[i], EPSILON);
      }
      
      assertSame(yoFrameOrientation.getReferenceFrame(), frameOrientationToPack.getReferenceFrame());
   }
}
