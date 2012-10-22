package com.yobotics.simulationconstructionset;


import java.awt.AWTException;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import org.junit.After;
import org.junit.Before;
import org.junit.Ignore;
import org.junit.Test;

import us.ihmc.utilities.keyboardAndMouse.GhostMousePlayback;

public class SimulationConstructionSetTest
{

   @Before
   public void setUp() throws Exception
   {
   }

   @After
   public void tearDown() throws Exception
   {
   }
   
   @Ignore
   public void testOne() throws AWTException
   {
      SimpleRobot simpleRobot = new SimpleRobot();
      
      SimulationConstructionSet scs = new SimulationConstructionSet(simpleRobot);
      scs.setFrameMaximized();
      scs.startOnAThread();
      
      java.awt.Robot guiRobot = new java.awt.Robot();
      
      guiRobot.delay(2000);
      
      performActionsRecordedUsingGhostMouse();

//      sleepForever();
   }
   
   @Test
   public void testTwo() throws AWTException
   {
      SimpleRobot simpleRobot = new SimpleRobot();
      
      SimulationConstructionSet scs = new SimulationConstructionSet(simpleRobot);
      scs.setFrameMaximized();
      scs.startOnAThread();
      
      java.awt.Robot guiRobot = new java.awt.Robot();
      
      guiRobot.delay(2000);
      GhostMousePlayback playback = new GhostMousePlayback();      
      playback.load("testFiles/SimulationConstructionSetGUITest/testSCSGUIOne.rms");
      playback.load("testFiles/SimulationConstructionSetGUITest/testSCSGUITwo.rms");
      playback.load("testFiles/SimulationConstructionSetGUITest/testSCSGUIThree.rms");
      playback.playback(2.0);
      
//      sleepForever();
   }
   
  
   private void performActionsRecordedUsingGhostMouse() throws AWTException
   {
      GhostMousePlayback playback = new GhostMousePlayback();      
      
      playback.addPlaybackEvent("{Delay 0.49}");
      playback.addPlaybackEvent("{Move (1117,964)}");
      playback.addPlaybackEvent("{Delay 0.04}");
      playback.addPlaybackEvent("{Move (1070,944)}");
      playback.addPlaybackEvent("{Delay 0.04}");
      playback.addPlaybackEvent("{Move (948,901)}");
      playback.addPlaybackEvent("{Delay 0.04}");
      playback.addPlaybackEvent("{Move (773,850)}");
      playback.addPlaybackEvent("{Delay 0.04}");
      playback.addPlaybackEvent("{Move (655,816)}");
      playback.addPlaybackEvent("{Delay 0.04}");
      playback.addPlaybackEvent("{Move (645,808)}");
      playback.addPlaybackEvent("{Delay 0.53}");
      playback.addPlaybackEvent("{Move (645,812)}");
      playback.addPlaybackEvent("{Delay 0.04}");
      playback.addPlaybackEvent("{Move (690,871)}");
      playback.addPlaybackEvent("{Delay 0.04}");
      playback.addPlaybackEvent("{Move (747,931)}");
      playback.addPlaybackEvent("{Delay 0.04}");
      playback.addPlaybackEvent("{Move (809,1004)}");
      playback.addPlaybackEvent("{Delay 0.04}");
      playback.addPlaybackEvent("{Move (832,1030)}");
      playback.addPlaybackEvent("{Delay 0.04}");
      playback.addPlaybackEvent("{Move (854,1069)}");
      playback.addPlaybackEvent("{Delay 0.04}");
      playback.addPlaybackEvent("{Move (874,1090)}");
      playback.addPlaybackEvent("{Delay 0.05}");
      playback.addPlaybackEvent("{Move (877,1093)}");
      playback.addPlaybackEvent("{Delay 0.04}");
      playback.addPlaybackEvent("{Move (898,1101)}");
      playback.addPlaybackEvent("{Delay 0.04}");
      playback.addPlaybackEvent("{Move (911,1109)}");
      playback.addPlaybackEvent("{Delay 0.12}");
      playback.addPlaybackEvent("{Move (913,1111)}");
      playback.addPlaybackEvent("{Delay 0.48}");
      playback.addPlaybackEvent("{LMouse down (913,1111)}");
      playback.addPlaybackEvent("{Delay 0.08}");
      playback.addPlaybackEvent("{Move (915,1111)}");
      playback.addPlaybackEvent("{Delay 0.01}");
      playback.addPlaybackEvent("{LMouse up (915,1111)}");
      playback.addPlaybackEvent("{Delay 0.16}");
      playback.addPlaybackEvent("{Move (914,1111)}");
      playback.addPlaybackEvent("{Delay 0.04}");
      playback.addPlaybackEvent("{Move (894,1100)}");
      playback.addPlaybackEvent("{Delay 0.04}");
      playback.addPlaybackEvent("{Move (802,1059)}");
      playback.addPlaybackEvent("{Delay 0.04}");
      playback.addPlaybackEvent("{Move (696,1007)}");
      playback.addPlaybackEvent("{Delay 0.04}");
      playback.addPlaybackEvent("{Move (521,926)}");
      playback.addPlaybackEvent("{Delay 0.04}");
      playback.addPlaybackEvent("{Move (397,874)}");
      playback.addPlaybackEvent("{Delay 0.04}");
      playback.addPlaybackEvent("{Move (293,814)}");
      playback.addPlaybackEvent("{Delay 0.04}");
      playback.addPlaybackEvent("{Move (230,778)}");
      playback.addPlaybackEvent("{Delay 0.04}");
      playback.addPlaybackEvent("{Move (217,770)}");
      playback.addPlaybackEvent("{Delay 0.04}");
      playback.addPlaybackEvent("{Move (212,766)}");
      playback.addPlaybackEvent("{Delay 0.04}");
      playback.addPlaybackEvent("{Move (194,761)}");
      playback.addPlaybackEvent("{Delay 0.04}");
      playback.addPlaybackEvent("{Move (167,758)}");
      playback.addPlaybackEvent("{Delay 0.04}");
      playback.addPlaybackEvent("{Move (104,738)}");
      playback.addPlaybackEvent("{Delay 0.04}");
      playback.addPlaybackEvent("{Move (65,725)}");
      playback.addPlaybackEvent("{Delay 0.04}");
      playback.addPlaybackEvent("{Move (51,719)}");
      playback.addPlaybackEvent("{Delay 0.09}");
      playback.addPlaybackEvent("{Move (44,714)}");
      playback.addPlaybackEvent("{Delay 0.09}");
      playback.addPlaybackEvent("{Move (37,712)}");
      playback.addPlaybackEvent("{Delay 0.04}");
      playback.addPlaybackEvent("{Move (32,704)}");
      playback.addPlaybackEvent("{Delay 0.04}");
      playback.addPlaybackEvent("{Move (30,700)}");
      playback.addPlaybackEvent("{Delay 0.08}");
      playback.addPlaybackEvent("{Move (27,696)}");
      playback.addPlaybackEvent("{Delay 0.04}");
      playback.addPlaybackEvent("{Move (22,689)}");
      playback.addPlaybackEvent("{Delay 0.04}");
      playback.addPlaybackEvent("{Move (19,687)}");
      playback.addPlaybackEvent("{Delay 0.15}");
      playback.addPlaybackEvent("{Move (17,681)}");
      playback.addPlaybackEvent("{Delay 0.27}");
      playback.addPlaybackEvent("{LMouse down (17,681)}");
      playback.addPlaybackEvent("{Delay 0.1}");
      playback.addPlaybackEvent("{LMouse up (17,681)}");
      playback.addPlaybackEvent("{Delay 0.56}");
      playback.addPlaybackEvent("{Move (19,681)}");
      playback.addPlaybackEvent("{Delay 0.04}");
      playback.addPlaybackEvent("{Move (33,685)}");
      playback.addPlaybackEvent("{Delay 0.04}");
      playback.addPlaybackEvent("{Move (53,692)}");
      playback.addPlaybackEvent("{Delay 0.04}");
      playback.addPlaybackEvent("{Move (115,707)}");
      playback.addPlaybackEvent("{Delay 0.04}");
      playback.addPlaybackEvent("{Move (201,725)}");
      playback.addPlaybackEvent("{Delay 0.04}");
      playback.addPlaybackEvent("{Move (324,755)}");
      playback.addPlaybackEvent("{Delay 0.04}");
      playback.addPlaybackEvent("{Move (458,786)}");
      playback.addPlaybackEvent("{Delay 0.04}");
      playback.addPlaybackEvent("{Move (552,802)}");
      playback.addPlaybackEvent("{Delay 0.04}");
      playback.addPlaybackEvent("{Move (599,813)}");
      playback.addPlaybackEvent("{Delay 0.67}");
      playback.addPlaybackEvent("{MMouse down (600,813)}");
      playback.addPlaybackEvent("{Delay 0.15}");
      playback.addPlaybackEvent("{MMouse up (600,813)}");
      playback.addPlaybackEvent("{Delay 0.59}");
      playback.addPlaybackEvent("{Move (601,813)}");
      playback.addPlaybackEvent("{Delay 0.04}");
      playback.addPlaybackEvent("{Move (608,811)}");
      playback.addPlaybackEvent("{Delay 0.04}");
      playback.addPlaybackEvent("{Move (635,797)}");
      playback.addPlaybackEvent("{Delay 0.04}");
      playback.addPlaybackEvent("{Move (674,752)}");
      playback.addPlaybackEvent("{Delay 0.04}");
      playback.addPlaybackEvent("{Move (696,721)}");
      playback.addPlaybackEvent("{Delay 0.04}");
      playback.addPlaybackEvent("{Move (703,700)}");
      playback.addPlaybackEvent("{Delay 0.04}");
      playback.addPlaybackEvent("{Move (706,683)}");
      playback.addPlaybackEvent("{Delay 0.04}");
      playback.addPlaybackEvent("{Move (711,659)}");
      playback.addPlaybackEvent("{Delay 0.04}");
      playback.addPlaybackEvent("{Move (718,647)}");
      playback.addPlaybackEvent("{Delay 0.04}");
      playback.addPlaybackEvent("{Move (726,626)}");
      playback.addPlaybackEvent("{Delay 0.04}");
      playback.addPlaybackEvent("{Move (726,616)}");
      playback.addPlaybackEvent("{Delay 0.04}");
      playback.addPlaybackEvent("{Move (726,606)}");
      playback.addPlaybackEvent("{Delay 0.52}");
      playback.addPlaybackEvent("{LMouse down (726,599)}");
      playback.addPlaybackEvent("{Delay 0.1}");
      playback.addPlaybackEvent("{LMouse up (726,599)}");
      playback.addPlaybackEvent("{Delay 0.09}");
      playback.addPlaybackEvent("{Move (727,599)}");
      playback.addPlaybackEvent("{Delay 0.04}");
      playback.addPlaybackEvent("{Move (733,599)}");
      playback.addPlaybackEvent("{Delay 0.04}");
      playback.addPlaybackEvent("{Move (744,599)}");
      playback.addPlaybackEvent("{Delay 0.04}");
      playback.addPlaybackEvent("{Move (757,599)}");
      playback.addPlaybackEvent("{Delay 0.04}");
      playback.addPlaybackEvent("{Move (768,599)}");
      playback.addPlaybackEvent("{Delay 0.04}");
      playback.addPlaybackEvent("{Move (769,599)}");
      playback.addPlaybackEvent("{Delay 0.04}");
      playback.addPlaybackEvent("{Move (771,599)}");
      playback.addPlaybackEvent("{Delay 0.04}");
      playback.addPlaybackEvent("{Move (781,601)}");
      playback.addPlaybackEvent("{Delay 0.04}");
      playback.addPlaybackEvent("{Move (788,602)}");
      playback.addPlaybackEvent("{Delay 0.04}");
      playback.addPlaybackEvent("{Move (793,602)}");
      playback.addPlaybackEvent("{Delay 0.98}");
      playback.addPlaybackEvent("{LMouse down (793,602)}");
      playback.addPlaybackEvent("{Delay 0.18}");
      playback.addPlaybackEvent("{LMouse up (793,602)}");
      playback.addPlaybackEvent("{Delay 0.24}");
      playback.addPlaybackEvent("{Move (792,604)}");
      playback.addPlaybackEvent("{Delay 0.04}");
      playback.addPlaybackEvent("{Move (782,605)}");
      playback.addPlaybackEvent("{Delay 0.1}");
      playback.addPlaybackEvent("{Move (781,605)}");
      playback.addPlaybackEvent("{Delay 0.04}");
      playback.addPlaybackEvent("{Move (771,605)}");
      playback.addPlaybackEvent("{Delay 0.04}");
      playback.addPlaybackEvent("{Move (759,603)}");
      playback.addPlaybackEvent("{Delay 0.05}");
      playback.addPlaybackEvent("{Move (752,601)}");
      playback.addPlaybackEvent("{Delay 0.48}");
      playback.addPlaybackEvent("{Move (751,601)}");
      playback.addPlaybackEvent("{Delay 0.01}");
      playback.addPlaybackEvent("{LMouse down (751,601)}");
      playback.addPlaybackEvent("{Delay 0.1}");
      playback.addPlaybackEvent("{LMouse up (751,601)}");
      playback.addPlaybackEvent("{Delay 0.13}");
      playback.addPlaybackEvent("{Move (753,601)}");
      playback.addPlaybackEvent("{Delay 0.04}");
      playback.addPlaybackEvent("{Move (762,601)}");
      playback.addPlaybackEvent("{Delay 0.04}");
      playback.addPlaybackEvent("{Move (773,601)}");
      playback.addPlaybackEvent("{Delay 0.04}");
      playback.addPlaybackEvent("{Move (789,601)}");
      playback.addPlaybackEvent("{Delay 0.07}");
      playback.addPlaybackEvent("{Move (794,601)}");
      playback.addPlaybackEvent("{Delay 1.13}");
      playback.addPlaybackEvent("{LMouse down (794,601)}");
      playback.addPlaybackEvent("{Delay 0.2}");
      playback.addPlaybackEvent("{LMouse up (794,601)}");
      playback.addPlaybackEvent("{Delay 0.11}");
      playback.addPlaybackEvent("{Move (794,605)}");
      playback.addPlaybackEvent("{Delay 0.04}");
      playback.addPlaybackEvent("{Move (794,622)}");
      playback.addPlaybackEvent("{Delay 0.04}");
      playback.addPlaybackEvent("{Move (771,666)}");
      playback.addPlaybackEvent("{Delay 0.04}");
      playback.addPlaybackEvent("{Move (736,709)}");
      playback.addPlaybackEvent("{Delay 0.04}");
      playback.addPlaybackEvent("{Move (695,749)}");
      playback.addPlaybackEvent("{Delay 0.04}");
      playback.addPlaybackEvent("{Move (678,763)}");
      playback.addPlaybackEvent("{Delay 0.05}");
      playback.addPlaybackEvent("{Move (676,767)}");
      playback.addPlaybackEvent("{Delay 0.05}");
      playback.addPlaybackEvent("{Move (672,769)}");
      playback.addPlaybackEvent("{Delay 0.08}");
      playback.addPlaybackEvent("{Move (670,769)}");
      playback.addPlaybackEvent("{Delay 0.01}");
      playback.addPlaybackEvent("{LMouse down (670,769)}");
      playback.addPlaybackEvent("{Delay 0.07}");
      playback.addPlaybackEvent("{LMouse up (670,769)}");
      playback.addPlaybackEvent("{Delay 0.06}");
      playback.addPlaybackEvent("{Move (671,769)}");
      playback.addPlaybackEvent("{Delay 0.04}");
      playback.addPlaybackEvent("{Move (679,765)}");
      playback.addPlaybackEvent("{Delay 0.04}");
      playback.addPlaybackEvent("{Move (698,757)}");
      playback.addPlaybackEvent("{Delay 0.04}");
      playback.addPlaybackEvent("{Move (709,751)}");
      playback.addPlaybackEvent("{Delay 0.04}");
      playback.addPlaybackEvent("{Move (755,724)}");
      playback.addPlaybackEvent("{Delay 0.04}");
      playback.addPlaybackEvent("{Move (783,704)}");
      playback.addPlaybackEvent("{Delay 0.04}");
      playback.addPlaybackEvent("{Move (797,693)}");
      playback.addPlaybackEvent("{Delay 0.04}");
      playback.addPlaybackEvent("{Move (801,688)}");
      playback.addPlaybackEvent("{Delay 0.04}");
      playback.addPlaybackEvent("{Move (805,685)}");
      playback.addPlaybackEvent("{Delay 0.04}");
      playback.addPlaybackEvent("{Move (814,672)}");
      playback.addPlaybackEvent("{Delay 0.04}");
      playback.addPlaybackEvent("{Move (820,664)}");
      playback.addPlaybackEvent("{Delay 0.04}");
      playback.addPlaybackEvent("{Move (829,648)}");
      playback.addPlaybackEvent("{Delay 0.04}");
      playback.addPlaybackEvent("{Move (837,632)}");
      playback.addPlaybackEvent("{Delay 0.04}");
      playback.addPlaybackEvent("{Move (843,623)}");
      playback.addPlaybackEvent("{Delay 0.13}");
      playback.addPlaybackEvent("{Move (843,616)}");
      playback.addPlaybackEvent("{Delay 0.04}");
      playback.addPlaybackEvent("{Move (841,612)}");
      playback.addPlaybackEvent("{Delay 0.98}");
      playback.addPlaybackEvent("{Move (837,607)}");
      playback.addPlaybackEvent("{Delay 0.01}");
      playback.addPlaybackEvent("{LMouse down (837,607)}");
      playback.addPlaybackEvent("{Delay 0.08}");
      playback.addPlaybackEvent("{LMouse up (837,607)}");
      playback.addPlaybackEvent("{Delay 0.12}");
      playback.addPlaybackEvent("{Move (837,607)}");
      playback.addPlaybackEvent("{Delay 0.04}");
      playback.addPlaybackEvent("{Move (843,622)}");
      playback.addPlaybackEvent("{Delay 0.04}");
      playback.addPlaybackEvent("{Move (871,655)}");
      playback.addPlaybackEvent("{Delay 0.04}");
      playback.addPlaybackEvent("{Move (892,672)}");
      playback.addPlaybackEvent("{Delay 0.04}");
      playback.addPlaybackEvent("{Move (907,675)}");
      playback.addPlaybackEvent("{Delay 0.04}");
      playback.addPlaybackEvent("{Move (916,676)}");
      playback.addPlaybackEvent("{Delay 0.04}");
      playback.addPlaybackEvent("{Move (925,681)}");
      playback.addPlaybackEvent("{Delay 0.04}");
      playback.addPlaybackEvent("{Move (953,698)}");
      playback.addPlaybackEvent("{Delay 0.04}");
      playback.addPlaybackEvent("{Move (976,708)}");
      playback.addPlaybackEvent("{Delay 0.04}");
      playback.addPlaybackEvent("{Move (1012,723)}");
      playback.addPlaybackEvent("{Delay 0.04}");
      playback.addPlaybackEvent("{Move (1052,732)}");
      playback.addPlaybackEvent("{Delay 0.04}");
      playback.addPlaybackEvent("{Move (1069,734)}");
      playback.addPlaybackEvent("{Delay 0.04}");
      playback.addPlaybackEvent("{Move (1095,734)}");
      playback.addPlaybackEvent("{Delay 0.04}");
      playback.addPlaybackEvent("{Move (1107,734)}");
      playback.addPlaybackEvent("{Delay 0.14}");
      playback.addPlaybackEvent("{Move (1111,735)}");
      playback.addPlaybackEvent("{Delay 0.04}");
      playback.addPlaybackEvent("{Move (1120,738)}");
      playback.addPlaybackEvent("{Delay 0.13}");
      playback.addPlaybackEvent("{LMouse down (1121,740)}");
      playback.addPlaybackEvent("{Delay 0.09}");
      playback.addPlaybackEvent("{LMouse up (1121,740)}");
      playback.addPlaybackEvent("{Delay 0.06}");
      playback.addPlaybackEvent("{Move (1121,738)}");
      playback.addPlaybackEvent("{Delay 0.04}");
      playback.addPlaybackEvent("{Move (1114,720)}");
      playback.addPlaybackEvent("{Delay 0.04}");
      playback.addPlaybackEvent("{Move (1101,690)}");
      playback.addPlaybackEvent("{Delay 0.04}");
      playback.addPlaybackEvent("{Move (1085,665)}");
      playback.addPlaybackEvent("{Delay 0.04}");
      playback.addPlaybackEvent("{Move (1080,659)}");
      playback.addPlaybackEvent("{Delay 0.04}");
      playback.addPlaybackEvent("{Move (1073,652)}");
      playback.addPlaybackEvent("{Delay 0.04}");
      playback.addPlaybackEvent("{Move (1062,644)}");
      playback.addPlaybackEvent("{Delay 0.04}");
      playback.addPlaybackEvent("{Move (1053,637)}");
      playback.addPlaybackEvent("{Delay 0.04}");
      playback.addPlaybackEvent("{Move (1042,629)}");
      playback.addPlaybackEvent("{Delay 0.04}");
      playback.addPlaybackEvent("{Move (1031,622)}");
      playback.addPlaybackEvent("{Delay 0.12}");
      playback.addPlaybackEvent("{Move (1019,614)}");
      playback.addPlaybackEvent("{Delay 0.04}");
      playback.addPlaybackEvent("{Move (1018,613)}");
      playback.addPlaybackEvent("{Delay 0.48}");
      playback.addPlaybackEvent("{Move (1018,612)}");
      playback.addPlaybackEvent("{Delay 0.01}");
      playback.addPlaybackEvent("{LMouse down (1018,612)}");
      playback.addPlaybackEvent("{Delay 0.09}");
      playback.addPlaybackEvent("{LMouse up (1018,612)}");
      playback.addPlaybackEvent("{Delay 0.37}");
      playback.addPlaybackEvent("{Move (1015,612)}");
      playback.addPlaybackEvent("{Delay 0.04}");
      playback.addPlaybackEvent("{Move (1001,612)}");
      playback.addPlaybackEvent("{Delay 0.04}");
      playback.addPlaybackEvent("{Move (940,599)}");
      playback.addPlaybackEvent("{Delay 0.04}");
      playback.addPlaybackEvent("{Move (809,566)}");
      playback.addPlaybackEvent("{Delay 0.04}");
      playback.addPlaybackEvent("{Move (676,512)}");
      playback.addPlaybackEvent("{Delay 0.04}");
      playback.addPlaybackEvent("{Move (549,429)}");
      playback.addPlaybackEvent("{Delay 0.04}");
      playback.addPlaybackEvent("{Move (446,344)}");
      playback.addPlaybackEvent("{Delay 0.04}");
      playback.addPlaybackEvent("{Move (368,269)}");
      playback.addPlaybackEvent("{Delay 0.04}");
      playback.addPlaybackEvent("{Move (340,229)}");
      playback.addPlaybackEvent("{Delay 0.04}");
      playback.addPlaybackEvent("{Move (321,194)}");
      playback.addPlaybackEvent("{Delay 0.04}");
      playback.addPlaybackEvent("{Move (301,171)}");
      playback.addPlaybackEvent("{Delay 0.04}");
      playback.addPlaybackEvent("{Move (291,159)}");
      playback.addPlaybackEvent("{Delay 0.04}");
      playback.addPlaybackEvent("{Move (282,147)}");
      playback.addPlaybackEvent("{Delay 0.04}");
      playback.addPlaybackEvent("{Move (272,136)}");
      playback.addPlaybackEvent("{Delay 0.04}");
      playback.addPlaybackEvent("{Move (267,126)}");
      playback.addPlaybackEvent("{Delay 0.04}");
      playback.addPlaybackEvent("{Move (256,102)}");
      playback.addPlaybackEvent("{Delay 0.04}");
      playback.addPlaybackEvent("{Move (253,78)}");
      playback.addPlaybackEvent("{Delay 0.04}");
      playback.addPlaybackEvent("{Move (252,64)}");
      playback.addPlaybackEvent("{Delay 0.04}");
      playback.addPlaybackEvent("{Move (247,53)}");
      playback.addPlaybackEvent("{Delay 0.04}");
      playback.addPlaybackEvent("{Move (240,46)}");
      playback.addPlaybackEvent("{Delay 0.26}");
      playback.addPlaybackEvent("{Move (239,43)}");
      playback.addPlaybackEvent("{Delay 0.01}");
      playback.addPlaybackEvent("{LMouse down (239,43)}");
      playback.addPlaybackEvent("{Delay 0.09}");
      playback.addPlaybackEvent("{LMouse up (239,43)}");
      playback.addPlaybackEvent("{Delay 0.37}");
      playback.addPlaybackEvent("{Move (239,43)}");
      playback.addPlaybackEvent("{Delay 0.06}");
      playback.addPlaybackEvent("{Move (239,47)}");
      playback.addPlaybackEvent("{Delay 0.05}");
      playback.addPlaybackEvent("{Move (240,50)}");
      playback.addPlaybackEvent("{Delay 0.07}");
      playback.addPlaybackEvent("{Move (242,54)}");
      playback.addPlaybackEvent("{Delay 0.47}");
      playback.addPlaybackEvent("{Move (243,58)}");
      playback.addPlaybackEvent("{Delay 0.01}");
      playback.addPlaybackEvent("{LMouse down (243,58)}");
      playback.addPlaybackEvent("{Delay 0.09}");
      playback.addPlaybackEvent("{LMouse up (243,58)}");
      playback.addPlaybackEvent("{Delay 0.19}");
      playback.addPlaybackEvent("{Move (246,63)}");
      playback.addPlaybackEvent("{Delay 0.04}");
      playback.addPlaybackEvent("{Move (256,77)}");
      playback.addPlaybackEvent("{Delay 0.04}");
      playback.addPlaybackEvent("{Move (269,99)}");
      playback.addPlaybackEvent("{Delay 0.04}");
      playback.addPlaybackEvent("{Move (292,133)}");
      playback.addPlaybackEvent("{Delay 0.04}");
      playback.addPlaybackEvent("{Move (338,184)}");
      playback.addPlaybackEvent("{Delay 0.04}");
      playback.addPlaybackEvent("{Move (380,225)}");
      playback.addPlaybackEvent("{Delay 0.04}");
      playback.addPlaybackEvent("{Move (409,263)}");
      playback.addPlaybackEvent("{Delay 0.04}");
      playback.addPlaybackEvent("{Move (450,315)}");
      playback.addPlaybackEvent("{Delay 0.04}");
      playback.addPlaybackEvent("{Move (507,369)}");
      playback.addPlaybackEvent("{Delay 0.04}");
      playback.addPlaybackEvent("{Move (571,422)}");
      playback.addPlaybackEvent("{Delay 0.04}");
      playback.addPlaybackEvent("{Move (643,469)}");
      playback.addPlaybackEvent("{Delay 0.04}");
      playback.addPlaybackEvent("{Move (691,493)}");
      playback.addPlaybackEvent("{Delay 0.04}");
      playback.addPlaybackEvent("{Move (717,507)}");
      playback.addPlaybackEvent("{Delay 0.04}");
      playback.addPlaybackEvent("{Move (746,526)}");
      playback.addPlaybackEvent("{Delay 0.04}");
      playback.addPlaybackEvent("{Move (761,541)}");
      playback.addPlaybackEvent("{Delay 0.04}");
      playback.addPlaybackEvent("{Move (769,555)}");
      playback.addPlaybackEvent("{Delay 0.04}");
      playback.addPlaybackEvent("{Move (785,581)}");
      playback.addPlaybackEvent("{Delay 0.05}");
      playback.addPlaybackEvent("{Move (789,591)}");
      playback.addPlaybackEvent("{Delay 0.06}");
      playback.addPlaybackEvent("{Move (787,593)}");
      playback.addPlaybackEvent("{Delay 0.04}");
      playback.addPlaybackEvent("{Move (778,594)}");
      playback.addPlaybackEvent("{Delay 0.04}");
      playback.addPlaybackEvent("{Move (773,598)}");
      playback.addPlaybackEvent("{Delay 0.04}");
      playback.addPlaybackEvent("{Move (766,602)}");
      playback.addPlaybackEvent("{Delay 0.04}");
      playback.addPlaybackEvent("{Move (759,602)}");
      playback.addPlaybackEvent("{Delay 0.18}");
      playback.addPlaybackEvent("{LMouse down (755,602)}");
      playback.addPlaybackEvent("{Delay 0.09}");
      playback.addPlaybackEvent("{LMouse up (755,602)}");
      playback.addPlaybackEvent("{Delay 0.16}");
      playback.addPlaybackEvent("{Move (757,602)}");
      playback.addPlaybackEvent("{Delay 0.04}");
      playback.addPlaybackEvent("{Move (767,602)}");
      playback.addPlaybackEvent("{Delay 0.08}");
      playback.addPlaybackEvent("{Move (775,603)}");
      playback.addPlaybackEvent("{Delay 0.04}");
      playback.addPlaybackEvent("{Move (789,606)}");
      playback.addPlaybackEvent("{Delay 0.04}");
      playback.addPlaybackEvent("{Move (798,607)}");
      playback.addPlaybackEvent("{Delay 0.16}");
      playback.addPlaybackEvent("{Move (798,607)}");
      playback.addPlaybackEvent("{Delay 0.04}");
      playback.addPlaybackEvent("{Move (795,607)}");
      playback.addPlaybackEvent("{Delay 0.67}");
      playback.addPlaybackEvent("{LMouse down (794,607)}");
      playback.addPlaybackEvent("{Delay 0.2}");
      playback.addPlaybackEvent("{LMouse up (794,607)}");
      playback.addPlaybackEvent("{Delay 0.81}");
      playback.addPlaybackEvent("{Move (794,607)}");
      playback.addPlaybackEvent("{Delay 0.04}");
      playback.addPlaybackEvent("{Move (805,616)}");
      playback.addPlaybackEvent("{Delay 0.04}");
      playback.addPlaybackEvent("{Move (817,627)}");
      playback.addPlaybackEvent("{Delay 0.04}");
      playback.addPlaybackEvent("{Move (838,652)}");
      playback.addPlaybackEvent("{Delay 0.04}");
      playback.addPlaybackEvent("{Move (874,698)}");
      playback.addPlaybackEvent("{Delay 0.04}");
      playback.addPlaybackEvent("{Move (909,739)}");
      playback.addPlaybackEvent("{Delay 0.04}");
      playback.addPlaybackEvent("{Move (953,794)}");
      playback.addPlaybackEvent("{Delay 0.04}");
      playback.addPlaybackEvent("{Move (983,832)}");
      playback.addPlaybackEvent("{Delay 0.04}");
      playback.addPlaybackEvent("{Move (1001,858)}");
      playback.addPlaybackEvent("{Delay 0.04}");
      playback.addPlaybackEvent("{Move (1025,877)}");
      playback.addPlaybackEvent("{Delay 0.04}");
      playback.addPlaybackEvent("{Move (1039,888)}");
      playback.addPlaybackEvent("{Delay 0.04}");
      playback.addPlaybackEvent("{Move (1061,908)}");
      playback.addPlaybackEvent("{Delay 0.04}");
      playback.addPlaybackEvent("{Move (1079,923)}");
      playback.addPlaybackEvent("{Delay 0.04}");
      playback.addPlaybackEvent("{Move (1100,945)}");
      playback.addPlaybackEvent("{Delay 0.05}");
      playback.addPlaybackEvent("{Move (1116,958)}");
      playback.addPlaybackEvent("{Delay 0.04}");
      playback.addPlaybackEvent("{Move (1122,965)}");
      playback.addPlaybackEvent("{Delay 0.53}");
      playback.addPlaybackEvent("{LMouse down (1130,974)}");
      playback.addPlaybackEvent("{Delay 0.1}");
      playback.addPlaybackEvent("{LMouse up (1130,974)}");

      
      playback.playback();
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
         
         LinkGraphics linkGraphics = new LinkGraphics();
         linkGraphics.addCube(0.1, 0.1, 0.1);
         
         body.setLinkGraphics(linkGraphics);
         rootJoint.setLink(body);
         
         this.addRootJoint(rootJoint);
      }
   }
   
   private void sleepForever()
   {
      while(true)
      {
         try
         {
            Thread.sleep(1000);
            
           
//            System.out.println(MouseInfo.getPointerInfo().getLocation());
         } 
         catch (InterruptedException e)
         {
         }
      }
   }

}
