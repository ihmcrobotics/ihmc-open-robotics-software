package com.yobotics.simulationconstructionset.gui.actions;

import static org.junit.Assert.*;

import org.junit.After;
import org.junit.AfterClass;
import org.junit.Before;
import org.junit.BeforeClass;
import org.junit.Test;

public class ActionsTest
{
   @BeforeClass
   public static void setUpBeforeClass() throws Exception
   {
   }

   @AfterClass
   public static void tearDownAfterClass() throws Exception
   {
   }

   @Before
   public void setUp() throws Exception
   {
   }

   @After
   public void tearDown() throws Exception
   {
   }

   @Test
   public void testAboutAction()
   {
      final boolean[] listenerCalled = new boolean[] {false};

      AboutActionListener aboutActionListener = new AboutActionListener()
      {
         public void showAbout()
         {
            listenerCalled[0] = true;
         }
      };

      AboutAction aboutAction = new AboutAction(aboutActionListener);
      aboutAction.actionPerformed(null);

      assertTrue(listenerCalled[0]);

   }

}
