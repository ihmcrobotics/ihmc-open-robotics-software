package us.ihmc.sensorProcessing.controlFlowPorts;

import static org.junit.Assert.*;

import java.util.Random;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.controlFlow.ControlFlowElement;
import us.ihmc.controlFlow.NullControlFlowElement;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.random.RandomGeometry;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

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
         FrameOrientation dataIn = new FrameOrientation(frame, RandomGeometry.nextQuaternion(rand));
         controlFlowOutputPort.setData(dataIn);
         FrameOrientation dataOut = controlFlowOutputPort.getData();

         assertTrue("Expected: " + dataIn + ", but was: " + dataOut, dataIn.getQuaternionCopy().epsilonEquals(dataOut.getQuaternionCopy(), EPS));
      }
   }

}
