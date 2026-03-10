package us.ihmc.tools.logic;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;
import java.util.concurrent.atomic.AtomicBoolean;

public class OrderedLogicalSequenceTest
{
   /**
    * Test to ensure all logical elements are executed in the correct order
    */
   @Test
   public void testOrderedLogic()
   {
      OrderedLogicalSequence orderedLogicalSequence = new OrderedLogicalSequence();

      AtomicBoolean logicalElement1 = new AtomicBoolean(false);
      AtomicBoolean logicalElement2 = new AtomicBoolean(false);
      AtomicBoolean logicalElement3 = new AtomicBoolean(false);
      AtomicBoolean logicalElement4 = new AtomicBoolean(false);
      AtomicBoolean logicalElement5 = new AtomicBoolean(false);

      orderedLogicalSequence.addLogicalElement(() -> logicalElement1.set(true), null,  null);
      orderedLogicalSequence.addLogicalElement(() -> logicalElement2.set(true), null,  null);
      orderedLogicalSequence.addLogicalElement(() -> logicalElement3.set(true), null,  null);
      orderedLogicalSequence.addLogicalElement(() -> logicalElement4.set(true), null,  null);
      orderedLogicalSequence.addLogicalElement(() -> logicalElement5.set(true), null,  null);

      // Start sequence and check if it started
      orderedLogicalSequence.start();
      assertTrue(orderedLogicalSequence.hasStarted(), "Logical sequence has not started, but it should have!");

      // Update sequence and make sure only first element executed
      assertEquals(0, orderedLogicalSequence.getCurrentLogicalElement(), "Current logical element should be 0, instead it is: " + orderedLogicalSequence.getCurrentLogicalElement());
      orderedLogicalSequence.update();
      assertTrue(logicalElement1.get(), "Logical element 1 should be true, but it is false");
      assertFalse(logicalElement2.get(), "Logical element 2 should be false, but it is true");
      assertFalse(logicalElement3.get(), "Logical element 3 should be false, but it is true");
      assertFalse(logicalElement4.get(), "Logical element 4 should be false, but it is true");
      assertFalse(logicalElement5.get(), "Logical element 5 should be false, but it is true");

      // Update sequence and make sure only first and second elements executed
      assertEquals(1, orderedLogicalSequence.getCurrentLogicalElement(), "Current logical element should be 1, instead it is: " + orderedLogicalSequence.getCurrentLogicalElement());
      orderedLogicalSequence.update();
      assertTrue(logicalElement1.get(), "Logical element 1 should be true, but it is false");
      assertTrue(logicalElement2.get(), "Logical element 2 should be true, but it is false");
      assertFalse(logicalElement3.get(), "Logical element 3 should be false, but it is true");
      assertFalse(logicalElement4.get(), "Logical element 4 should be false, but it is true");
      assertFalse(logicalElement5.get(), "Logical element 5 should be false, but it is true");

      // Update sequence and make sure only first, second, and third elements executed
      assertEquals(2, orderedLogicalSequence.getCurrentLogicalElement(), "Current logical element should be 2, instead it is: " + orderedLogicalSequence.getCurrentLogicalElement());
      orderedLogicalSequence.update();
      assertTrue(logicalElement1.get(), "Logical element 1 should be true, but it is false");
      assertTrue(logicalElement2.get(), "Logical element 2 should be true, but it is false");
      assertTrue(logicalElement3.get(), "Logical element 3 should be true, but it is false");
      assertFalse(logicalElement4.get(), "Logical element 4 should be false, but it is true");
      assertFalse(logicalElement5.get(), "Logical element 5 should be false, but it is true");

      // Update sequence and make sure only first, second, third, and fourth elements executed
      assertEquals(3, orderedLogicalSequence.getCurrentLogicalElement(), "Current logical element should be 3, instead it is: " + orderedLogicalSequence.getCurrentLogicalElement());
      orderedLogicalSequence.update();
      assertTrue(logicalElement1.get(), "Logical element 1 should be true, but it is false");
      assertTrue(logicalElement2.get(), "Logical element 2 should be true, but it is false");
      assertTrue(logicalElement3.get(), "Logical element 3 should be true, but it is false");
      assertTrue(logicalElement4.get(), "Logical element 4 should be true, but it is false");
      assertFalse(logicalElement5.get(), "Logical element 5 should be false, but it is true");

      // Update sequence and make sure all elements executed
      assertEquals(4, orderedLogicalSequence.getCurrentLogicalElement(), "Current logical element should be 4, instead it is: " + orderedLogicalSequence.getCurrentLogicalElement());
      orderedLogicalSequence.update();
      assertTrue(logicalElement1.get(), "Logical element 1 should be true, but it is false");
      assertTrue(logicalElement2.get(), "Logical element 2 should be true, but it is false");
      assertTrue(logicalElement3.get(), "Logical element 3 should be true, but it is false");
      assertTrue(logicalElement4.get(), "Logical element 4 should be true, but it is false");
      assertTrue(logicalElement5.get(), "Logical element 5 should be true, but it is false");

      // Ensure sequence has finished
      assertTrue(orderedLogicalSequence.hasFinished(), "Logical sequence has not finished, but it should have!");

      // Reset logical sequence and ensure that worked
      orderedLogicalSequence.reset();
      assertFalse(orderedLogicalSequence.hasStarted(), "Logical sequence hasStarted() returns true, despite logical sequence being reset");
      assertFalse(orderedLogicalSequence.hasFinished(), "Logical sequence hasFinished() returns true, despite logical sequence being reset");
   }

   /**
    * Test to ensure a successful prerequisite condition results in correct execution
    */
   @Test
   public void testSuccessfulPreRequisite()
   {
      OrderedLogicalSequence orderedLogicalSequence = new OrderedLogicalSequence();

      AtomicBoolean logicalElement1 = new AtomicBoolean(false);
      AtomicBoolean logicalElement2 = new AtomicBoolean(false);

      // Set up logical sequence
      orderedLogicalSequence.addLogicalElement(() -> logicalElement1.set(true), null,  null);
      orderedLogicalSequence.addLogicalElement(() -> logicalElement2.set(true), logicalElement1::get, null);

      // Start sequence and check if it started
      orderedLogicalSequence.start();
      assertTrue(orderedLogicalSequence.hasStarted(), "Logical sequence has not started, but it should have!");

      // Update sequence and make sure only first element executed
      assertEquals(0, orderedLogicalSequence.getCurrentLogicalElement(), "Current logical element should be 0, instead it is: " + orderedLogicalSequence.getCurrentLogicalElement());
      orderedLogicalSequence.update();
      assertTrue(logicalElement1.get(), "Logical element 1 should be true, but it is false");
      assertFalse(logicalElement2.get(), "Logical element 2 should be false, but it is true");

      // Update sequence and make sure both the first and second elements executed
      assertEquals(1, orderedLogicalSequence.getCurrentLogicalElement(), "Current logical element should be 1, instead it is: " + orderedLogicalSequence.getCurrentLogicalElement());
      orderedLogicalSequence.update();
      assertTrue(logicalElement1.get(), "Logical element 1 should be true, but it is false");
      assertTrue(logicalElement2.get(), "Logical element 2 should be true, but it is false");

      // Ensure sequence has finished
      assertTrue(orderedLogicalSequence.hasFinished(), "Logical sequence has not finished, but it should have!");

      // Reset logical sequence and ensure that worked
      orderedLogicalSequence.reset();
      assertFalse(orderedLogicalSequence.hasStarted(), "Logical sequence hasStarted() returns true, despite logical sequence being reset");
      assertFalse(orderedLogicalSequence.hasFinished(), "Logical sequence hasFinished() returns true, despite logical sequence being reset");
   }

   /**
    * Test to ensure an unsuccessful prerequisite condition results in correct execution
    */
   @Test
   public void testUnsuccessfulPreRequisite()
   {
      OrderedLogicalSequence orderedLogicalSequence = new OrderedLogicalSequence();

      AtomicBoolean logicalElement1 = new AtomicBoolean(true);
      AtomicBoolean logicalElement2 = new AtomicBoolean(false);

      // Set up logical sequence
      orderedLogicalSequence.addLogicalElement(() -> logicalElement1.set(false), null,  null);
      orderedLogicalSequence.addLogicalElement(() -> logicalElement2.set(true), logicalElement1::get, null);

      // Start sequence and check if it started
      orderedLogicalSequence.start();
      assertTrue(orderedLogicalSequence.hasStarted(), "Logical sequence has not started, but it should have!");

      // Update sequence and make sure only first element executed
      assertEquals(0, orderedLogicalSequence.getCurrentLogicalElement(), "Current logical element should be 0, instead it is: " + orderedLogicalSequence.getCurrentLogicalElement());
      orderedLogicalSequence.update();
      assertFalse(logicalElement1.get(), "Logical element 1 should be false, but it is true");
      assertFalse(logicalElement2.get(), "Logical element 2 should be false, but it is true");

      // Update sequence and make sure both the first and second elements executed
      assertEquals(1, orderedLogicalSequence.getCurrentLogicalElement(), "Current logical element should be 1, instead it is: " + orderedLogicalSequence.getCurrentLogicalElement());
      orderedLogicalSequence.update();
      assertFalse(logicalElement1.get(), "Logical element 1 should be false, but it is true");
      assertFalse(logicalElement2.get(), "Logical element 2 should STILL be false, but it is true");
      assertEquals(1, orderedLogicalSequence.getCurrentLogicalElement(), "Current logical element should STILL be 1, instead it is: " + orderedLogicalSequence.getCurrentLogicalElement());

      // Ensure sequence has finished
      assertFalse(orderedLogicalSequence.hasFinished(), "Logical sequence has finished, but it SHOULD NOT have!");

      // Reset logical sequence and ensure that worked
      orderedLogicalSequence.reset();
      assertFalse(orderedLogicalSequence.hasStarted(), "Logical sequence hasStarted() returns true, despite logical sequence being reset");
      assertFalse(orderedLogicalSequence.hasFinished(), "Logical sequence hasFinished() returns true, despite logical sequence being reset");
   }

   /**
    * Test to ensure a successful completion condition results in correct execution
    */
   @Test
   public void testSuccessfulCompletion()
   {
      OrderedLogicalSequence orderedLogicalSequence = new OrderedLogicalSequence();

      AtomicBoolean logicalElement1 = new AtomicBoolean(false);
      AtomicBoolean logicalElement2 = new AtomicBoolean(false);

      // Set up logical sequence
      orderedLogicalSequence.addLogicalElement(() -> logicalElement1.set(true), null,  logicalElement1::get);
      orderedLogicalSequence.addLogicalElement(() -> logicalElement2.set(true), null, null);

      // Start sequence and check if it started
      orderedLogicalSequence.start();
      assertTrue(orderedLogicalSequence.hasStarted(), "Logical sequence has not started, but it should have!");

      // Update sequence and make sure only first element executed
      assertEquals(0, orderedLogicalSequence.getCurrentLogicalElement(), "Current logical element should be 0, instead it is: " + orderedLogicalSequence.getCurrentLogicalElement());
      orderedLogicalSequence.update();
      assertTrue(logicalElement1.get(), "Logical element 1 should be true, but it is false");
      assertFalse(logicalElement2.get(), "Logical element 2 should be false, but it is true");

      // Update sequence and make sure both the first and second elements executed
      assertEquals(1, orderedLogicalSequence.getCurrentLogicalElement(), "Current logical element should be 1, instead it is: " + orderedLogicalSequence.getCurrentLogicalElement());
      orderedLogicalSequence.update();
      assertTrue(logicalElement1.get(), "Logical element 1 should be true, but it is false");
      assertTrue(logicalElement2.get(), "Logical element 2 should be true, but it is false");

      // Ensure sequence has finished
      assertTrue(orderedLogicalSequence.hasFinished(), "Logical sequence has not finished, but it should have!");

      // Reset logical sequence and ensure that worked
      orderedLogicalSequence.reset();
      assertFalse(orderedLogicalSequence.hasStarted(), "Logical sequence hasStarted() returns true, despite logical sequence being reset");
      assertFalse(orderedLogicalSequence.hasFinished(), "Logical sequence hasFinished() returns true, despite logical sequence being reset");
   }

   /**
    * Test to ensure an unsuccessful completion condition results in correct execution
    */
   @Test
   public void testUnsuccessfulCompletion()
   {
      OrderedLogicalSequence orderedLogicalSequence = new OrderedLogicalSequence();

      AtomicBoolean logicalElement1 = new AtomicBoolean(true);
      AtomicBoolean logicalElement2 = new AtomicBoolean(false);

      // Set up logical sequence
      orderedLogicalSequence.addLogicalElement(() -> logicalElement1.set(false), null,  logicalElement1::get);
      orderedLogicalSequence.addLogicalElement(() -> logicalElement2.set(true), null, null);

      // Start sequence and check if it started
      orderedLogicalSequence.start();
      assertTrue(orderedLogicalSequence.hasStarted(), "Logical sequence has not started, but it should have!");

      // Update sequence and make sure only first element executed
      assertEquals(0, orderedLogicalSequence.getCurrentLogicalElement(), "Current logical element should be 0, instead it is: " + orderedLogicalSequence.getCurrentLogicalElement());
      orderedLogicalSequence.update();
      assertFalse(logicalElement1.get(), "Logical element 1 should be false, but it is true");
      assertFalse(logicalElement2.get(), "Logical element 2 should be false, but it is true");

      // Update sequence and make sure both the first and second elements executed
      assertEquals(0, orderedLogicalSequence.getCurrentLogicalElement(), "Current logical element should STILL be 0, instead it is: " + orderedLogicalSequence.getCurrentLogicalElement());
      orderedLogicalSequence.update();
      assertFalse(logicalElement1.get(), "Logical element 1 should be false, but it is true");
      assertFalse(logicalElement2.get(), "Logical element 2 should STILL be false, but it is true");

      // Ensure sequence has finished
      assertFalse(orderedLogicalSequence.hasFinished(), "Logical sequence has finished, but it SHOULD NOT have!");

      // Reset logical sequence and ensure that worked
      orderedLogicalSequence.reset();
      assertFalse(orderedLogicalSequence.hasStarted(), "Logical sequence hasStarted() returns true, despite logical sequence being reset");
      assertFalse(orderedLogicalSequence.hasFinished(), "Logical sequence hasFinished() returns true, despite logical sequence being reset");
   }
}
