package us.ihmc.atlas.hikSim;

import java.util.ArrayList;

import org.junit.Test;

import us.ihmc.SdfLoader.FullRobotModelVisualizer;
import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.atlas.AtlasWholeBodyIK;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.bambooTools.BambooTools;
import us.ihmc.utilities.Pair;
import us.ihmc.utilities.ThreadTools;
import us.ihmc.utilities.humanoidRobot.partNames.ArmJointName;
import us.ihmc.utilities.humanoidRobot.partNames.LegJointName;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.wholeBodyController.WholeBodyControllerParameters;
import us.ihmc.wholeBodyController.WholeBodyIkSolver;
import us.ihmc.wholeBodyController.WholeBodyIkSolver.ControlledDoF;
import us.ihmc.wholeBodyController.WholeBodyIkSolverTestFactory;

public class AtlasWholeBodyIkSolverTest extends WholeBodyIkSolverTestFactory
{ 
   static private final AtlasRobotModel atlasRobotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_DUAL_ROBOTIQ, AtlasRobotModel.AtlasTarget.SIM, false);
   static private SDFFullRobotModel actualRobotModel =  atlasRobotModel.createFullRobotModel();
   static private WholeBodyIkSolver wholeBodySolver = new AtlasWholeBodyIK(atlasRobotModel);
 
   static private SimulationConstructionSet scs;
   static private boolean VISUALIZE_GUI = false;   
   static FullRobotModelVisualizer modelVisualizer;
   
   public AtlasWholeBodyIkSolverTest() throws InterruptedException
   {
      super(actualRobotModel, wholeBodySolver);
      
      if( scs == null && VISUALIZE_GUI )
      {
         scs = new SimulationConstructionSet( atlasRobotModel.createSdfRobot(false) );
         modelVisualizer = new FullRobotModelVisualizer( scs, actualRobotModel,  0.01 );
         scs.startOnAThread(); 

         Thread.sleep(3000);
      }
   }
   
   @org.junit.AfterClass
   static public void keepAliveTheGUI()
   {
      if (scs != null)
      {
         ThreadTools.sleepForever();
      }
   }
   
   @Override 
   public WholeBodyControllerParameters getRobotModel()
   {
      return atlasRobotModel;
   }
   
   @Override 
   public FullRobotModelVisualizer getFullRobotModelVisualizer()
   {
      return  modelVisualizer;
   }
   
   @Override 
   public SimulationConstructionSet getSimulationConstructionSet()
   {
      return scs;
   }

   @Test(timeout = 150000)   public void testRightHandIn3PMode()
   {
     ArrayList<Pair<ReferenceFrame, ReferenceFrame>> handTargetArray = createHalfCylinderOfTargetPoints(RobotSide.RIGHT);
      executeHandTargetTest(ControlledDoF.DOF_NONE, ControlledDoF.DOF_3P, handTargetArray);      
   }

   @Test(timeout = 150000)   public void testLeftHandIn3PMode()
   {
      ArrayList<Pair<ReferenceFrame, ReferenceFrame>> handTargetArray = createHalfCylinderOfTargetPoints(RobotSide.LEFT);
      this.executeHandTargetTest(ControlledDoF.DOF_3P, ControlledDoF.DOF_NONE, handTargetArray);
   }
   
  /** 
   @Test(timeout = 150000)   public void testRightHandIn3P2RMode()
   {
      ArrayList<Pair<ReferenceFrame, ReferenceFrame>> handTargetArray = createWallOfTargetPoints(RobotSide.RIGHT);
      this.executeHandTargetTest(ControlledDoF.DOF_NONE, ControlledDoF.DOF_3P2R, handTargetArray);
   }

   @Test(timeout = 150000)   public void testLeftHandIn3P2RMode()
   {
      ArrayList<Pair<ReferenceFrame, ReferenceFrame>> handTargetArray = createWallOfTargetPoints(RobotSide.LEFT);
      this.executeHandTargetTest(ControlledDoF.DOF_3P2R, ControlledDoF.DOF_NONE, handTargetArray);
   }
   
   @Test(timeout = 150000)   public void testRightHandIn3P3RMode(){
      ArrayList<Pair<ReferenceFrame, ReferenceFrame>> handTargetArray = createWallOfTargetPoints(RobotSide.RIGHT);
      this.executeHandTargetTest(ControlledDoF.DOF_3P3R, ControlledDoF.DOF_NONE, handTargetArray);
   }
   
   @Test(timeout = 150000)   public void testLeftHandIn3P3RMode(){
      ArrayList<Pair<ReferenceFrame, ReferenceFrame>> handTargetArray = createWallOfTargetPoints(RobotSide.LEFT);
      this.executeHandTargetTest(ControlledDoF.DOF_NONE, ControlledDoF.DOF_3P3R, handTargetArray);
   }

   **/
   public void initializeFullRobotModelJointAngles(SDFFullRobotModel fullRobotModelToInitialize)
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         fullRobotModelToInitialize.getOneDoFJointByName(atlasRobotModel.getJointMap().getLegJointName(robotSide, LegJointName.HIP_YAW)).setQ(0.0); //leg_hpz
         fullRobotModelToInitialize.getOneDoFJointByName(atlasRobotModel.getJointMap().getLegJointName(robotSide, LegJointName.HIP_ROLL)).setQ(robotSide.negateIfRightSide(0.1)); //leg_hpx
         fullRobotModelToInitialize.getOneDoFJointByName(atlasRobotModel.getJointMap().getLegJointName(robotSide, LegJointName.HIP_PITCH)).setQ(-0.3); //leg_hpy
         fullRobotModelToInitialize.getOneDoFJointByName(atlasRobotModel.getJointMap().getLegJointName(robotSide, LegJointName.KNEE)).setQ(0.6); //leg_kny
         fullRobotModelToInitialize.getOneDoFJointByName(atlasRobotModel.getJointMap().getLegJointName(robotSide, LegJointName.ANKLE_PITCH)).setQ(-0.3); //leg_aky
         fullRobotModelToInitialize.getOneDoFJointByName(atlasRobotModel.getJointMap().getLegJointName(robotSide, LegJointName.ANKLE_ROLL)).setQ(robotSide.negateIfRightSide(-0.1)); //leg_akx

         fullRobotModelToInitialize.getOneDoFJointByName(atlasRobotModel.getJointMap().getArmJointName(robotSide, ArmJointName.SHOULDER_YAW)).setQ(0.500); //arm_shy
         fullRobotModelToInitialize.getOneDoFJointByName(atlasRobotModel.getJointMap().getArmJointName(robotSide, ArmJointName.SHOULDER_ROLL)).setQ(robotSide.negateIfRightSide(-1.0)); //arm_shx
         fullRobotModelToInitialize.getOneDoFJointByName(atlasRobotModel.getJointMap().getArmJointName(robotSide, ArmJointName.ELBOW_PITCH)).setQ(2.00); //arm_ely
         fullRobotModelToInitialize.getOneDoFJointByName(atlasRobotModel.getJointMap().getArmJointName(robotSide, ArmJointName.ELBOW_ROLL)).setQ(robotSide.negateIfRightSide(0.6)); //arm_elx
         fullRobotModelToInitialize.getOneDoFJointByName(atlasRobotModel.getJointMap().getArmJointName(robotSide, ArmJointName.WRIST_PITCH)).setQ(0.000); //arm_wry
         fullRobotModelToInitialize.getOneDoFJointByName(atlasRobotModel.getJointMap().getArmJointName(robotSide, ArmJointName.WRIST_ROLL)).setQ(robotSide.negateIfRightSide(0)); //arm_wrx
      }
   }

   public void initializeSDFRobotlJointAngles(SDFRobot scsRobot)
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         scsRobot.getOneDegreeOfFreedomJoint(atlasRobotModel.getJointMap().getLegJointName(robotSide, LegJointName.HIP_YAW)).setQ(0.0); //leg_hpz
         scsRobot.getOneDegreeOfFreedomJoint(atlasRobotModel.getJointMap().getLegJointName(robotSide, LegJointName.HIP_ROLL)).setQ(robotSide.negateIfRightSide(0.062)); //leg_hpx
         scsRobot.getOneDegreeOfFreedomJoint(atlasRobotModel.getJointMap().getLegJointName(robotSide, LegJointName.HIP_PITCH)).setQ(-0.233); //leg_hpy
         scsRobot.getOneDegreeOfFreedomJoint(atlasRobotModel.getJointMap().getLegJointName(robotSide, LegJointName.KNEE)).setQ(0.518); //leg_kny
         scsRobot.getOneDegreeOfFreedomJoint(atlasRobotModel.getJointMap().getLegJointName(robotSide, LegJointName.ANKLE_PITCH)).setQ(-0.276); //leg_aky
         scsRobot.getOneDegreeOfFreedomJoint(atlasRobotModel.getJointMap().getLegJointName(robotSide, LegJointName.ANKLE_ROLL)).setQ(robotSide.negateIfRightSide(-0.062)); //leg_akx
         
         scsRobot.getOneDegreeOfFreedomJoint(atlasRobotModel.getJointMap().getArmJointName(robotSide, ArmJointName.SHOULDER_YAW)).setQ(0.300); //arm_shy
         scsRobot.getOneDegreeOfFreedomJoint(atlasRobotModel.getJointMap().getArmJointName(robotSide, ArmJointName.SHOULDER_ROLL)).setQ(robotSide.negateIfRightSide(-1.30)); //arm_shx
         scsRobot.getOneDegreeOfFreedomJoint(atlasRobotModel.getJointMap().getArmJointName(robotSide, ArmJointName.ELBOW_PITCH)).setQ(2.00); //arm_ely
         scsRobot.getOneDegreeOfFreedomJoint(atlasRobotModel.getJointMap().getArmJointName(robotSide, ArmJointName.ELBOW_ROLL)).setQ(robotSide.negateIfRightSide(0.498)); //arm_elx
         scsRobot.getOneDegreeOfFreedomJoint(atlasRobotModel.getJointMap().getArmJointName(robotSide, ArmJointName.WRIST_PITCH)).setQ(0.000); //arm_wry
         scsRobot.getOneDegreeOfFreedomJoint(atlasRobotModel.getJointMap().getArmJointName(robotSide, ArmJointName.WRIST_ROLL)).setQ(robotSide.negateIfRightSide(-0.004)); //arm_wrx
      }
   }
}
