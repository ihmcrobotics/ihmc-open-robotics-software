package us.ihmc.sensorProcessing.controlFlowPorts;

import static org.junit.Assert.*;

import java.util.Random;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.controlFlow.ControlFlowElement;
import us.ihmc.controlFlow.NullControlFlowElement;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.robotics.random.RandomGeometry;

public class YoFrameQuaternionControlFlowOutputPortTest
{

   private static final double EPS = 1e-15;

	@ContinuousIntegrationTest(estimatedDuration = 0.1)
	@Test(timeout = 30000)
   public void simpleWritingReadingTest()
   {
      ControlFlowElement controlFlowElement = new NullControlFlowElement();
      
      String namePrefix = "test";
      ReferenceFrame frame = ReferenceFrame.getWorldFrame();
      YoVariableRegistry registry = new YoVariableRegistry("blop");
      
      YoFrameQuaternionControlFlowOutputPort controlFlowOutputPort = new YoFrameQuaternionControlFlowOutputPort(controlFlowElement, namePrefix, frame, registry);

      assertEquals(namePrefix, controlFlowOutputPort.getName());
      
      Random rand = new Random(1567);
      
      for (int i = 0; i < 1000; i++)
      {
         FrameQuaternion dataIn = new FrameQuaternion(frame, RandomGeometry.nextQuaternion(rand));
         controlFlowOutputPort.setData(dataIn);
         FrameQuaternion dataOut = controlFlowOutputPort.getData();

         assertTrue("Expected: " + dataIn + ", but was: " + dataOut, dataIn.epsilonEquals(dataOut, EPS));
      }
   }

}
