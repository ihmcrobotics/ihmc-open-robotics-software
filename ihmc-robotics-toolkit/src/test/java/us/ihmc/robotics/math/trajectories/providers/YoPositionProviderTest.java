package us.ihmc.robotics.math.trajectories.providers;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertSame;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.robotics.math.frames.YoFramePoint;


public class YoPositionProviderTest
{
   private static final double EPSILON = 1e-10;
   
   private YoFramePoint yoFramePoint;
   private String namePrefix = "namePrefixTEST";
   private ReferenceFrame referenceFrame;
   private YoVariableRegistry registry;
   
   private YoPositionProvider provider;
   
   private static double xValue = Math.random();
   private static double yValue = Math.random();
   private static double zValue = Math.random();
   
   @Before
   public void setUp()
   {
      referenceFrame = ReferenceFrame.constructARootFrame("rootNameTEST");
      registry = new YoVariableRegistry("parentRegistryTEST");
      yoFramePoint = new YoFramePoint(namePrefix, referenceFrame, registry);
      yoFramePoint.set(xValue, yValue, zValue);
   }
   
   @After
   public void tearDown()
   {
      referenceFrame = null;
      registry = null;
      yoFramePoint = null;
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testConstructor()
   {
      provider = new YoPositionProvider(null);
      provider = new YoPositionProvider(yoFramePoint);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testSetAndGet()
   {
      provider = new YoPositionProvider(yoFramePoint);
      
      FramePoint3D result = new FramePoint3D(referenceFrame);
      result.set(5.0, 10.0, 15.0);
      provider.set(result);
      
      FramePoint3D positionToPack = new FramePoint3D();
      provider.getPosition(positionToPack);
      
      assertSame(referenceFrame, positionToPack.getReferenceFrame());
      assertEquals(5.0, positionToPack.getX(), EPSILON);
      assertEquals(10.0, positionToPack.getY(), EPSILON);
      assertEquals(15.0, positionToPack.getZ(), EPSILON);
   }
}
