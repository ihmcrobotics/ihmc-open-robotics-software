package us.ihmc.controlFlow;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import java.util.List;

import javax.vecmath.Point3d;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.tools.testing.JUnitTools;
import us.ihmc.tools.thread.ThreadTools;

public class ControlFlowTest
{

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testSimpleControlFlowWithOneNodeAndNoInput()
   {
      boolean VISUALIZE = false;

      double x = 3.0;
      double y = 2.0;

      InNoneOutOneControlFlowElement elementOne = new InNoneOutOneControlFlowElement(x, y);

      ControlFlowGraph controlFlowGraph = new ControlFlowGraph();

      ControlFlowOutputPort<DataTypeOne> graphOutputPort = new ControlFlowOutputPort<DataTypeOne>("graphOutputPort", controlFlowGraph);

      controlFlowGraph.connectOutputPort(graphOutputPort, elementOne.getOutputPort());

      controlFlowGraph.initializeAfterConnections();

      controlFlowGraph.startComputation();
      controlFlowGraph.waitUntilComputationIsDone();

      DataTypeOne dataTypeOne = graphOutputPort.getData();

      List<ControlFlowInputPort<?>> inputPorts = controlFlowGraph.getInputPorts();
      List<ControlFlowOutputPort<?>> outputPorts = controlFlowGraph.getOutputPorts();

      assertEquals(0, inputPorts.size());
      assertEquals(1, outputPorts.size());

      assertTrue(graphOutputPort == outputPorts.get(0));

      assertEquals(0, elementOne.getInputPorts().size());
      assertEquals(1, elementOne.getOutputPorts().size());

      assertTrue(elementOne.getOutputPort() == elementOne.getOutputPorts().get(0));

      assertEquals(x, dataTypeOne.getX(), 1e-7);
      assertEquals(y, dataTypeOne.getY(), 1e-7);

      if (VISUALIZE)
      {
         controlFlowGraph.visualize();
         ThreadTools.sleepForever();
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testSimpleControlFlowWithOneNode()
   {
      boolean VISUALIZE = false;

      InOneOutTwoControlFlowElement elementOne = new InOneOutTwoControlFlowElement();

      ControlFlowGraph controlFlowGraph = new ControlFlowGraph();

      ControlFlowInputPort<DataTypeOne> graphInputPort = new ControlFlowInputPort<DataTypeOne>("graphInputPort", controlFlowGraph);
      ControlFlowOutputPort<DataTypeTwo> graphOutputPort = new ControlFlowOutputPort<DataTypeTwo>("graphOutputPort", controlFlowGraph);

      controlFlowGraph.connectInputPort(graphInputPort, elementOne.getInputPort());
      controlFlowGraph.connectOutputPort(graphOutputPort, elementOne.getOutputPort());

      controlFlowGraph.initializeAfterConnections();

      DataTypeOne dataTypeOne = new DataTypeOne();

      double x = 3.0;
      double y = 2.0;

      dataTypeOne.setX(x);
      dataTypeOne.setY(y);

      graphInputPort.setData(dataTypeOne);

      controlFlowGraph.startComputation();
      controlFlowGraph.waitUntilComputationIsDone();

      DataTypeTwo dataTypeTwo = graphOutputPort.getData();

      List<ControlFlowInputPort<?>> inputPorts = controlFlowGraph.getInputPorts();
      List<ControlFlowOutputPort<?>> outputPorts = controlFlowGraph.getOutputPorts();

      assertEquals(1, inputPorts.size());
      assertEquals(1, outputPorts.size());

      assertTrue(graphInputPort == inputPorts.get(0));
      assertTrue(graphOutputPort == outputPorts.get(0));

      assertEquals(1, elementOne.getInputPorts().size());
      assertEquals(1, elementOne.getOutputPorts().size());

      assertTrue(elementOne.getInputPort() == elementOne.getInputPorts().get(0));
      assertTrue(elementOne.getOutputPort() == elementOne.getOutputPorts().get(0));

      assertEquals((x + y), dataTypeTwo.getZ(), 1e-7);

      if (VISUALIZE)
      {
         controlFlowGraph.visualize();
         ThreadTools.sleepForever();

      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testSimpleControlFlowWithTwoNodes()
   {
      boolean VISUALIZE = false;

      InOneOutTwoControlFlowElement elementOne = new InOneOutTwoControlFlowElement();
      InTwoOutThreeControlFlowElement elementTwo = new InTwoOutThreeControlFlowElement();

      ControlFlowGraph controlFlowGraph = new ControlFlowGraph();

      ControlFlowInputPort<DataTypeOne> graphInputPort = new ControlFlowInputPort<DataTypeOne>("graphInputPort", controlFlowGraph);
      ControlFlowOutputPort<DataTypeThree> graphOutputPort = new ControlFlowOutputPort<DataTypeThree>("graphOutputPort", controlFlowGraph);

      controlFlowGraph.connectInputPort(graphInputPort, elementOne.getInputPort());
      controlFlowGraph.connectElements(elementOne.getOutputPort(), elementTwo.getInputPort());
      controlFlowGraph.connectOutputPort(graphOutputPort, elementTwo.getOutputPort());

      controlFlowGraph.initializeAfterConnections();

      DataTypeOne dataTypeOne = new DataTypeOne();

      double x = 3.0;
      double y = 2.0;

      dataTypeOne.setX(x);
      dataTypeOne.setY(y);

      graphInputPort.setData(dataTypeOne);

      controlFlowGraph.startComputation();
      controlFlowGraph.waitUntilComputationIsDone();

      DataTypeThree dataTypeThree = graphOutputPort.getData();

      assertEquals((x + y) * (x + y), dataTypeThree.getQ(), 1e-7);

      if (VISUALIZE)
      {
         controlFlowGraph.visualize();
         ThreadTools.sleepForever();

      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testSimpleControlFlowWithThreeNodesAndNoInput()
   {
      boolean VISUALIZE = false;

      double x = 3.3;
      double y = 2.5;

      InNoneOutOneControlFlowElement elementZero = new InNoneOutOneControlFlowElement(x, y);
      InOneOutTwoControlFlowElement elementOne = new InOneOutTwoControlFlowElement();
      InTwoOutThreeControlFlowElement elementTwo = new InTwoOutThreeControlFlowElement();

      ControlFlowGraph controlFlowGraph = new ControlFlowGraph();

      ControlFlowOutputPort<DataTypeThree> graphOutputPort = new ControlFlowOutputPort<DataTypeThree>("graphOutputPort", controlFlowGraph);

      controlFlowGraph.connectElements(elementZero.getOutputPort(), elementOne.getInputPort());
      controlFlowGraph.connectElements(elementOne.getOutputPort(), elementTwo.getInputPort());
      controlFlowGraph.connectOutputPort(graphOutputPort, elementTwo.getOutputPort());

      controlFlowGraph.initializeAfterConnections();

      controlFlowGraph.startComputation();
      controlFlowGraph.waitUntilComputationIsDone();

      DataTypeThree dataTypeThree = graphOutputPort.getData();

      assertEquals((x + y) * (x + y), dataTypeThree.getQ(), 1e-7);

      if (VISUALIZE)
      {
         controlFlowGraph.visualize();
         ThreadTools.sleepForever();

      }
   }

// @Test(timeout=300000) // Doesn't even compile!!
// public void testInvalidControlFlow()
// {
//    InOneOutTwoControlFlowElement elementOne = new InOneOutTwoControlFlowElement();
//    InOneOutTwoControlFlowElement elementTwo = new InOneOutTwoControlFlowElement();
//    
//    ControlFlowGraph controlFlowGraph = new ControlFlowGraph();
//          
//    try
//    {
//       controlFlowGraph.connectElements(elementOne, elementOne.getOutputPort(), elementTwo, elementTwo.getInputPort());
//       fail("Should have thrown an Exception");
//    }
//    catch(RuntimeException exception)
//    {
//
//    }
// }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testSimpleMultiportControlFlow()
   {
      InOneAndThreeOutFourAndFiveControlFlowElement controlFlowElement = new InOneAndThreeOutFourAndFiveControlFlowElement();

      ControlFlowGraph controlFlowGraph = new ControlFlowGraph();

      ControlFlowInputPort<DataTypeOne> dataTypeOneInputPort = new ControlFlowInputPort<DataTypeOne>("dataTypeOneInputPort", controlFlowGraph);
      ControlFlowInputPort<DataTypeThree> dataTypeThreeInputPort = new ControlFlowInputPort<DataTypeThree>("dataTypeThreeInputPort", controlFlowGraph);

      ControlFlowOutputPort<DataTypeFour> dataTypeFourOutputPort = new ControlFlowOutputPort<DataTypeFour>("dataTypeFourOutputPort", controlFlowElement);
      ControlFlowOutputPort<DataTypeFive> dataTypeFiveOutputPort = new ControlFlowOutputPort<DataTypeFive>("dataTypeFiveOutputPort", controlFlowElement);


      controlFlowGraph.connectInputPort(dataTypeOneInputPort, controlFlowElement.getDataTypeOneInputPort());
      controlFlowGraph.connectInputPort(dataTypeThreeInputPort, controlFlowElement.getDataTypeThreeInputPort());

      controlFlowGraph.connectOutputPort(dataTypeFourOutputPort, controlFlowElement.getDataTypeFourOutputPort());
      controlFlowGraph.connectOutputPort(dataTypeFiveOutputPort, controlFlowElement.getDataTypeFiveOutputPort());


      controlFlowGraph.initializeAfterConnections();

      List<ControlFlowInputPort<?>> inputPorts = controlFlowGraph.getInputPorts();
      List<ControlFlowOutputPort<?>> outputPorts = controlFlowGraph.getOutputPorts();

      assertEquals(2, inputPorts.size());
      assertEquals(inputPorts.get(0), dataTypeOneInputPort);
      assertEquals(inputPorts.get(1), dataTypeThreeInputPort);

      assertEquals(2, outputPorts.size());
      assertEquals(outputPorts.get(0), dataTypeFourOutputPort);
      assertEquals(outputPorts.get(1), dataTypeFiveOutputPort);

      DataTypeOne dataTypeOne = new DataTypeOne();
      DataTypeThree dataTypeThree = new DataTypeThree();

      double x = 1.0;
      double y = 2.2;
      double q = 3.4;

      dataTypeOne.setX(x);
      dataTypeOne.setY(y);
      dataTypeThree.setQ(q);

      dataTypeOneInputPort.setData(dataTypeOne);
      dataTypeThreeInputPort.setData(dataTypeThree);

      controlFlowGraph.startComputation();
      controlFlowGraph.waitUntilComputationIsDone();

      DataTypeFour dataTypeFour = dataTypeFourOutputPort.getData();
      DataTypeFive dataTypeFive = dataTypeFiveOutputPort.getData();

      JUnitTools.assertTuple3dEquals(new Point3d(x, y, q), dataTypeFour.getPoint(), 1e-7);

      String expectedString = "scale";
      assertTrue(expectedString.equals(dataTypeFive.getString()));
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testMultiportControlFlow()
   {
      boolean VISUALIZE = false;

      InOneAndTwoOutThreeControlFlowElement elementA = new InOneAndTwoOutThreeControlFlowElement();
      InOneAndThreeOutFourAndFiveControlFlowElement elementB = new InOneAndThreeOutFourAndFiveControlFlowElement();
      InThreeAndFiveOutFiveControlFlowElement elementC = new InThreeAndFiveOutFiveControlFlowElement();
      InFourAndFiveOutFourControlFlowElement elementD = new InFourAndFiveOutFourControlFlowElement();

      ControlFlowGraph controlFlowGraph = new ControlFlowGraph();

      ControlFlowInputPort<DataTypeOne> graphDataTypeOneInputPort = new ControlFlowInputPort<DataTypeOne>("graphDataTypeOneInputPort", controlFlowGraph);
      ControlFlowInputPort<DataTypeTwo> graphDataTypeTwoInputPort = new ControlFlowInputPort<DataTypeTwo>("graphDataTypeTwoInputPort", controlFlowGraph);
      ControlFlowInputPort<DataTypeFive> graphDataTypeFiveInputPort = new ControlFlowInputPort<DataTypeFive>("graphDataTypeFiveInputPort", controlFlowGraph);


      controlFlowGraph.connectInputPort(graphDataTypeOneInputPort, elementA.getDataTypeOneInputPort());
      controlFlowGraph.connectInputPort(graphDataTypeOneInputPort, elementB.getDataTypeOneInputPort());
      controlFlowGraph.connectInputPort(graphDataTypeTwoInputPort, elementA.getDataTypeTwoInputPort());
      controlFlowGraph.connectInputPort(graphDataTypeFiveInputPort, elementC.getDataTypeFiveInputPort());

      controlFlowGraph.connectElements(elementA.getDataTypeThreeOutputPort(), elementB.getDataTypeThreeInputPort());
      controlFlowGraph.connectElements(elementA.getDataTypeThreeOutputPort(), elementC.getDataTypeThreeInputPort());
      controlFlowGraph.connectElements(elementB.getDataTypeFourOutputPort(), elementD.getDataTypeFourInputPort());
      controlFlowGraph.connectElements(elementB.getDataTypeFiveOutputPort(), elementD.getDataTypeFiveInputPort());

      ControlFlowOutputPort<DataTypeFour> graphDataTypeFourOutputPort = new ControlFlowOutputPort<DataTypeFour>("graphDataTypeFourOutputPort",
                                                                           controlFlowGraph);
      ControlFlowOutputPort<DataTypeFive> graphDataTypeFiveOutputPort = new ControlFlowOutputPort<DataTypeFive>("graphDataTypeFiveOutputPort",
                                                                           controlFlowGraph);

      controlFlowGraph.connectOutputPort(graphDataTypeFourOutputPort, elementD.getDataTypeFourOutputPort());
      controlFlowGraph.connectOutputPort(graphDataTypeFiveOutputPort, elementC.getDataTypeFiveOutputPort());

      controlFlowGraph.initializeAfterConnections();

      DataTypeOne dataTypeOneInput = new DataTypeOne();
      DataTypeTwo dataTypeTwoInput = new DataTypeTwo();
      DataTypeFive dataTypeFiveInput = new DataTypeFive();

      double x = 1.7;
      double y = 3.9;
      double z = 5.5;
      String string = "testingOneTwoThree";

      dataTypeOneInput.setX(x);
      dataTypeOneInput.setY(y);
      dataTypeTwoInput.setZ(z);
      dataTypeFiveInput.setString(string);

      graphDataTypeOneInputPort.setData(dataTypeOneInput);
      graphDataTypeTwoInputPort.setData(dataTypeTwoInput);
      graphDataTypeFiveInputPort.setData(dataTypeFiveInput);

      controlFlowGraph.startComputation();
      controlFlowGraph.waitUntilComputationIsDone();

      DataTypeFour dataTypeFourOutput = graphDataTypeFourOutputPort.getData();
      DataTypeFive dataTypeFiveOutput = graphDataTypeFiveOutputPort.getData();

      Point3d point3dOut = dataTypeFourOutput.getPoint();
      String stringOut = dataTypeFiveOutput.getString();

      Point3d expectePoint3d = new Point3d(x, y, (x + y) * z);
      expectePoint3d.scale(2.0);
      JUnitTools.assertTuple3dEquals(expectePoint3d, point3dOut, 1e-7);

      assertEquals("testingOneTwoThree, q = " + (x + y) * z, stringOut);

      if (VISUALIZE)
      {
         controlFlowGraph.visualize();
         ThreadTools.sleepForever();
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testMultiportControlFlowTwo()
   {
      boolean VISUALIZE = false;

      InOneOutTwoControlFlowElement elementA = new InOneOutTwoControlFlowElement();
      InTwoOutThreeControlFlowElement elementB = new InTwoOutThreeControlFlowElement();
      InTwoAndThreeOutTwoAndThreeControlFlowElement elementC = new InTwoAndThreeOutTwoAndThreeControlFlowElement();
      InTwoAndThreeOutFourControlFlowElement elementD = new InTwoAndThreeOutFourControlFlowElement();
      InTwoAndThreeOutFourControlFlowElement elementE = new InTwoAndThreeOutFourControlFlowElement();

      ControlFlowGraph controlFlowGraph = new ControlFlowGraph();

      ControlFlowInputPort<DataTypeOne> graphDataTypeOneInputPort = new ControlFlowInputPort<DataTypeOne>("graphDataTypeOneInputPort", controlFlowGraph);
      ControlFlowInputPort<DataTypeTwo> graphDataTypeTwoInputPort = new ControlFlowInputPort<DataTypeTwo>("graphDataTypeTwoInputPort", controlFlowGraph);

      ControlFlowOutputPort<DataTypeFour> graphDataTypeFourOutputPortFromD = new ControlFlowOutputPort<DataTypeFour>("graphDataTypeFourOutputPortFromD",
                                                                                controlFlowGraph);
      ControlFlowOutputPort<DataTypeFour> graphDataTypeFourOutputPortFromE = new ControlFlowOutputPort<DataTypeFour>("graphDataTypeFourOutputPortFromE",
                                                                                controlFlowGraph);

      // Purposefully connect things willy nilly to make sure things are robust to it
      controlFlowGraph.connectElements(elementC.getDataTypeThreeOutputPort(), elementD.getDataTypeThreeInputPort());
      controlFlowGraph.connectElements(elementB.getOutputPort(), elementC.getDataTypeThreeInputPort());

      controlFlowGraph.connectOutputPort(graphDataTypeFourOutputPortFromD, elementD.getDataTypeFourOutputPort());
      controlFlowGraph.connectInputPort(graphDataTypeTwoInputPort, elementB.getInputPort());
      controlFlowGraph.connectElements(elementA.getOutputPort(), elementC.getDataTypeTwoInputPort());
      controlFlowGraph.connectElements(elementB.getOutputPort(), elementE.getDataTypeThreeInputPort());
      controlFlowGraph.connectElements(elementC.getDataTypeTwoOutputPort(), elementE.getDataTypeTwoInputPort());

      controlFlowGraph.connectInputPort(graphDataTypeOneInputPort, elementA.getInputPort());
      controlFlowGraph.connectOutputPort(graphDataTypeFourOutputPortFromE, elementE.getDataTypeFourOutputPort());
      controlFlowGraph.connectElements(elementA.getOutputPort(), elementD.getDataTypeTwoInputPort());

      controlFlowGraph.initializeAfterConnections();

      if (VISUALIZE)
      {
         controlFlowGraph.visualize();
         ThreadTools.sleepForever();
      }

      // Don't include any one of these and it should throw an exception:
      controlFlowGraph = new ControlFlowGraph();

      controlFlowGraph.connectElements(elementC.getDataTypeThreeOutputPort(), elementD.getDataTypeThreeInputPort());
      controlFlowGraph.connectElements(elementB.getOutputPort(), elementC.getDataTypeThreeInputPort());

      controlFlowGraph.connectOutputPort(graphDataTypeFourOutputPortFromD, elementD.getDataTypeFourOutputPort());

//    controlFlowGraph.connectInputPort(graphDataTypeTwoInputPort, elementB.getInputPort());
      controlFlowGraph.connectElements(elementA.getOutputPort(), elementC.getDataTypeTwoInputPort());
      controlFlowGraph.connectElements(elementB.getOutputPort(), elementE.getDataTypeThreeInputPort());
      controlFlowGraph.connectElements(elementC.getDataTypeTwoOutputPort(), elementE.getDataTypeTwoInputPort());

      controlFlowGraph.connectInputPort(graphDataTypeOneInputPort, elementA.getInputPort());
      controlFlowGraph.connectOutputPort(graphDataTypeFourOutputPortFromE, elementE.getDataTypeFourOutputPort());
      controlFlowGraph.connectElements(elementA.getOutputPort(), elementD.getDataTypeTwoInputPort());

      try
      {
         controlFlowGraph.initializeAfterConnections();
         fail();
      }
      catch (RuntimeException exception)
      {
      }

      // Don't include any one of these and it should throw an exception:
      controlFlowGraph = new ControlFlowGraph();

      controlFlowGraph.connectElements(elementC.getDataTypeThreeOutputPort(), elementD.getDataTypeThreeInputPort());
      controlFlowGraph.connectElements(elementB.getOutputPort(), elementC.getDataTypeThreeInputPort());

      controlFlowGraph.connectOutputPort(graphDataTypeFourOutputPortFromD, elementD.getDataTypeFourOutputPort());
      controlFlowGraph.connectInputPort(graphDataTypeTwoInputPort, elementB.getInputPort());
      controlFlowGraph.connectElements(elementA.getOutputPort(), elementC.getDataTypeTwoInputPort());

//    controlFlowGraph.connectElements(elementB.getOutputPort(), elementE.getDataTypeThreeInputPort());
      controlFlowGraph.connectElements(elementC.getDataTypeTwoOutputPort(), elementE.getDataTypeTwoInputPort());

      controlFlowGraph.connectInputPort(graphDataTypeOneInputPort, elementA.getInputPort());
      controlFlowGraph.connectOutputPort(graphDataTypeFourOutputPortFromE, elementE.getDataTypeFourOutputPort());
      controlFlowGraph.connectElements(elementA.getOutputPort(), elementD.getDataTypeTwoInputPort());

      try
      {
         controlFlowGraph.initializeAfterConnections();
         fail();
      }
      catch (RuntimeException exception)
      {
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testCreatingAPortAfterConnectingOtherPorts()
   {
      ControlFlowGraph controlFlowGraph = new ControlFlowGraph();

      AbstractControlFlowElement elementOne = createGenericControlFlowElement();
      AbstractControlFlowElement elementTwo = createGenericControlFlowElement();

      ControlFlowOutputPort<Object> elementOneOutputPortOne = elementOne.createOutputPort();
      ControlFlowInputPort<Object> elementTwoInputPortOne = elementTwo.createInputPort();

      controlFlowGraph.connectElements(elementOneOutputPortOne, elementTwoInputPortOne);

      ControlFlowOutputPort<Object> elementOneOutputPortTwo = elementOne.createOutputPort();
      ControlFlowInputPort<Object> elementTwoInputPortTwo = elementTwo.createInputPort();

      controlFlowGraph.connectElements(elementOneOutputPortTwo, elementTwoInputPortTwo);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testCreatingAnUnconnectedInputPortAfterConnectingTwoPorts()
   {
      ControlFlowGraph controlFlowGraph = new ControlFlowGraph();

      AbstractControlFlowElement elementOne = createGenericControlFlowElement();
      AbstractControlFlowElement elementTwo = createGenericControlFlowElement();

      ControlFlowOutputPort<Object> elementOneOutputPortOne = elementOne.createOutputPort();
      ControlFlowInputPort<Object> elementTwoInputPortOne = elementTwo.createInputPort();

      controlFlowGraph.connectElements(elementOneOutputPortOne, elementTwoInputPortOne);

      @SuppressWarnings("unused")
      ControlFlowInputPort<Object> elementTwoInputPortTwo = elementTwo.createInputPort();

      try
      {
         controlFlowGraph.initializeAfterConnections();
         fail("Unconnected input port");
      }
      catch(RuntimeException e)
      {
      } 
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testConnectingAPortThatIsNotRegistered()
   {
      ControlFlowGraph controlFlowGraph = new ControlFlowGraph();

      AbstractControlFlowElement elementOne = createGenericControlFlowElement();
      ControlFlowOutputPort<Object> danglingOutputPort = new ControlFlowOutputPort<Object>("Dangling", elementOne);

      AbstractControlFlowElement elementTwo = createGenericControlFlowElement();
      ControlFlowInputPort<Object> elementTwoInputPortOne = elementTwo.createInputPort();

      try
      {
         controlFlowGraph.connectElements(danglingOutputPort, elementTwoInputPortOne);
         fail("Need to register ports before connecting them");
      }
      catch (RuntimeException e)
      {
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testRegisterPortWithTheWrongElement()
   {
      AbstractControlFlowElement elementOne = createGenericControlFlowElement();
      ControlFlowOutputPort<Object> outputPort = new ControlFlowOutputPort<Object>("Dangling", elementOne);

      AbstractControlFlowElement elementTwo = createGenericControlFlowElement();

      try
      {
         elementTwo.registerOutputPort(outputPort);
         fail("Need to register ports with the element used in their construction");
      }
      catch (RuntimeException e)
      {
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testCreatingAPortAfterConnectingOtherPortsAndSendData()
   {
      ControlFlowGraph controlFlowGraph = new ControlFlowGraph();

      AbstractControlFlowElement elementOne = createGenericControlFlowElement();
      AbstractControlFlowElement elementTwo = createGenericControlFlowElement();

      ControlFlowOutputPort<DataTypeOne> elementOneOutputPortOne = elementOne.createOutputPort();
      ControlFlowInputPort<DataTypeOne> elementTwoInputPortOne = elementTwo.createInputPort();

      controlFlowGraph.connectElements(elementOneOutputPortOne, elementTwoInputPortOne);

      ControlFlowOutputPort<DataTypeFive> elementOneOutputPortTwo = elementOne.createOutputPort();
      ControlFlowInputPort<DataTypeFive> elementTwoInputPortTwo = elementTwo.createInputPort();

      controlFlowGraph.connectElements(elementOneOutputPortTwo, elementTwoInputPortTwo);
      
      DataTypeOne dataTypeOneOutput = new DataTypeOne();
      DataTypeFive dataTypeFiveOutput = new DataTypeFive();

      double x = 1.7;
      double y = 3.9;
      String string = "testingOneTwoThree"; 
      
      dataTypeOneOutput.setX(x);
      dataTypeOneOutput.setY(y);
      dataTypeFiveOutput.setString(string);
      
      elementOneOutputPortOne.setData(dataTypeOneOutput);
      elementOneOutputPortTwo.setData(dataTypeFiveOutput);
      
      controlFlowGraph.initializeAfterConnections();
      controlFlowGraph.startComputation();
      controlFlowGraph.waitUntilComputationIsDone();
      
      DataTypeOne dataTypeOneInput = elementTwoInputPortOne.getData();
      DataTypeFive dataTypeFiveInput = elementTwoInputPortTwo.getData();

      assertEquals(dataTypeOneOutput.toString(), dataTypeOneInput.toString());
      assertEquals(string, dataTypeFiveInput.getString());
   }



   private AbstractControlFlowElement createGenericControlFlowElement()
   {
      AbstractControlFlowElement elementOne = new AbstractControlFlowElement()
      {
         public void waitUntilComputationIsDone()
         {
         }
         public void startComputation()
         {
         }
         public void initialize()
         {
         }
      };

      return elementOne;
   }

}
