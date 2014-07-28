package com.yobotics.simulationconstructionset;

import static org.junit.Assert.*;

import java.awt.AWTException;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import org.junit.Test;

import us.ihmc.graphics3DAdapter.graphics.Graphics3DObject;
import us.ihmc.utilities.Axis;
import us.ihmc.utilities.ThreadTools;

import com.yobotics.simulationconstructionset.gui.SimulationGUITestFixture;

public class SimulationConstructionSetTest
{
   
   @Test
   public void testSimulationConstructionSetUsingDirectCalls() throws AWTException
   {
      SimpleRobot simpleRobot = new SimpleRobot();
      
      SimulationConstructionSet scs = new SimulationConstructionSet(simpleRobot);
      scs.setFrameMaximized();
      scs.startOnAThread();
      
      double simulateTime = 1.0;
      
      double startTime = scs.getTime();
      scs.simulate(simulateTime);
      while(scs.isSimulating())
      {
         ThreadTools.sleep(100L);
      }
      
      double endTime = scs.getTime();

      assertEquals(simulateTime, endTime-startTime, 1e-7);

//      ThreadTools.sleepForever();
      
      scs.closeAndDispose();
      scs = null;
   }
 
   @Test
   public void testSimulationConstructionSetUsingGUITestFixture() throws AWTException
   {
      SimpleRobot simpleRobot = new SimpleRobot();
      
      SimulationConstructionSet scs = new SimulationConstructionSet(simpleRobot);
      YoVariableRegistry registryOne = new YoVariableRegistry("RegistryOne");
      EnumYoVariable<Axis> enumForTests = new EnumYoVariable<Axis>("enumForTests", registryOne, Axis.class);
      YoVariableRegistry registryTwo = new YoVariableRegistry("RegistryTwo");
      BooleanYoVariable booleanForTests = new BooleanYoVariable("booleanForTests", registryTwo);
      registryOne.addChild(registryTwo);
      scs.addYoVariableRegistry(registryOne);
      
      scs.setFrameMaximized();
      scs.startOnAThread();
      scs.setSimulateDuration(2.0);

      
      SimulationGUITestFixture testFixture = new SimulationGUITestFixture(scs);
      
      testFixture.removeAllGraphs();
      testFixture.removeAllEntryBoxes();
      
      testFixture.selectNameSpaceTab();
      testFixture.selectNameSpace("root/RegistryOne");
      testFixture.selectVariableInOpenTab("enumForTests");
      ThreadTools.sleep(500);
      
      testFixture.selectNameSpaceTab();
      testFixture.selectNameSpace("root/RegistryOne/RegistryTwo");
      testFixture.selectVariableInOpenTab("booleanForTests");
      ThreadTools.sleep(500);

      testFixture.clickOnUnusedEntryBox();

      assertTrue(booleanForTests.getBooleanValue() == false);
      testFixture.findEntryBoxAndEnterValue("booleanForTests", 1.0);
      assertTrue(booleanForTests.getBooleanValue() == true);

      
      testFixture.selectSearchTab();
      testFixture.enterSearchText("q_");

      testFixture.selectVariableInSearchTab("q_y");
      testFixture.clickNewGraphButton();
      testFixture.middleClickInEmptyGraph();
      
      testFixture.selectVariableInSearchTab("q_z");
      testFixture.clickNewGraphButton();
      testFixture.middleClickInEmptyGraph();
      
      testFixture.removeAllGraphs();

      
      // Setup a few entry boxes:
      enumForTests.set(Axis.X);
      
      testFixture.selectSearchTab();
      testFixture.deleteSearchText();
      testFixture.enterSearchText("enumForTests");
      testFixture.selectVariableInSearchTab("enumForTests");

      testFixture.clickOnUnusedEntryBox();
      
      assertTrue(enumForTests.getEnumValue() == Axis.X);
      testFixture.findEnumEntryBoxAndSelectValue("enumForTests", "Z");
      assertTrue(enumForTests.getEnumValue() == Axis.Z);
      
      // Search for variables, change their values, and plot them:
//      testFixture.selectNameSpaceTab();
//      ThreadTools.sleep(1000);
      
      testFixture.selectSearchTab();
      
      testFixture.deleteSearchText();
      testFixture.enterSearchText("q_");
      
      testFixture.selectVariableInSearchTab("q_x");
      testFixture.clickRemoveEmptyGraphButton();
      testFixture.clickNewGraphButton();
      testFixture.middleClickInEmptyGraph();
      
      testFixture.clickNewGraphButton();
      testFixture.clickNewGraphButton();
      testFixture.selectVariableInSearchTab("q_y");
      testFixture.middleClickInNthGraph(2);
      ThreadTools.sleep(500);
      testFixture.selectVariableInSearchTab("q_z");
      testFixture.middleClickInNthGraph(2);
      
      testFixture.selectVariableAndSetValueInSearchTab("q_z", 1.31);
      DoubleYoVariable q_z = (DoubleYoVariable) scs.getVariable("q_z");
      assertEquals(1.31, q_z.getDoubleValue(), 1e-9);
      
      // Simulate and replay
      ThreadTools.sleep(500);
      testFixture.clickSimulateButton();
      ThreadTools.sleep(500);
      testFixture.clickStopButton();
      ThreadTools.sleep(500);
      testFixture.clickPlayButton();
      ThreadTools.sleep(500);
      testFixture.clickStopButton();
      ThreadTools.sleep(500);

      // Remove variables from graphs:
      testFixture.removeVariableFromNthGraph("q_y", 2);
      testFixture.clickRemoveEmptyGraphButton();
      

      // Go to In/out points, step through data. Add KeyPoints, Verify at the expected indices
      testFixture.clickGotoInPointButton();

      ThreadTools.sleep(100);

      int index = scs.getIndex();
      int inPoint = scs.getInPoint();
      assertEquals(index, inPoint);
      
      // Do some stepping forwards and putting in key points:
      int stepsForward = 8;
      for (int i=0; i<stepsForward; i++)
      {
         testFixture.clickStepForwardButton();
      }
      ThreadTools.sleep(100);
      index = scs.getIndex();
      assertEquals(stepsForward, index);
      testFixture.clickAddKeyPointButton();
      
      for (int i=0; i<stepsForward; i++)
      {
         testFixture.clickStepForwardButton();
      }
      ThreadTools.sleep(100);
      index = scs.getIndex();
      assertEquals(2*stepsForward, index);
      testFixture.clickAddKeyPointButton();
      
      for (int i=0; i<stepsForward; i++)
      {
         testFixture.clickStepForwardButton();
      }
      ThreadTools.sleep(100);
      index = scs.getIndex();
      assertEquals(3*stepsForward, index);
      testFixture.clickAddKeyPointButton();

      // Zoom in and out
      testFixture.clickZoomInButton();
      testFixture.clickZoomInButton();
      testFixture.clickZoomInButton();
      testFixture.clickZoomInButton();
      testFixture.clickZoomOutButton();
      
      testFixture.clickGotoInPointButton();
      testFixture.clickToggleKeyModeButton();
      
      testFixture.clickStepForwardButton();
      ThreadTools.sleep(100);
      index = scs.getIndex();
      assertEquals(stepsForward, index);
      
      testFixture.clickStepForwardButton();
      ThreadTools.sleep(100);
      index = scs.getIndex();
      assertEquals(2*stepsForward, index);
      
      // Toggle a keypoint off:
      testFixture.clickAddKeyPointButton();
      
      testFixture.clickStepBackwardButton();
      ThreadTools.sleep(100);
      index = scs.getIndex();
      assertEquals(stepsForward, index);
      
      testFixture.clickSetInPointButton();
      testFixture.clickStepForwardButton();
      testFixture.clickSetOutPointButton();

      testFixture.clickGotoInPointButton();
      ThreadTools.sleep(100);
      index = scs.getIndex();
      assertEquals(stepsForward, index);
      
      testFixture.clickGotoOutPointButton();
      ThreadTools.sleep(100);
      index = scs.getIndex();
      assertEquals(3*stepsForward, index);
      testFixture.clickGotoInPointButton();

      testFixture.clickToggleKeyModeButton();
      testFixture.clickStepForwardButton();
      ThreadTools.sleep(100);
      index = scs.getIndex();
      assertEquals(stepsForward+1, index);
      
      testFixture.closeAndDispose();
      scs.closeAndDispose();
      scs = null;
      testFixture = null;
   }
  


   public static class SimpleRobot extends Robot
   {
      private static final long serialVersionUID = 43883985473093746L;

      public SimpleRobot()
      {
         super("SimpleRobot");
         
         FloatingJoint rootJoint = new FloatingJoint("root", new Vector3d(), this);
         Link body = new Link("body");
         body.setMassAndRadiiOfGyration(1.0, 0.1, 0.1, 0.1);
         
         rootJoint.setPosition(new Point3d(0.1, 0.2, 1.2));
         
         Graphics3DObject linkGraphics = new Graphics3DObject();
         linkGraphics.addCube(0.1, 0.1, 0.1);
         
         body.setLinkGraphics(linkGraphics);
         rootJoint.setLink(body);
         
         this.addRootJoint(rootJoint);
      }
   }

}
