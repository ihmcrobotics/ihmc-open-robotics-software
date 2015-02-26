package us.ihmc.atlas.hikSim;

import org.junit.Ignore;
import org.junit.Test;

import us.ihmc.SdfLoader.FullRobotModelVisualizer;
import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.atlas.AtlasWholeBodyIK;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.bambooTools.SimulationTestingParameters;
import us.ihmc.utilities.ThreadTools;
import us.ihmc.utilities.code.agileTesting.BambooAnnotations.AverageDuration;
import us.ihmc.utilities.humanoidRobot.partNames.ArmJointName;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.wholeBodyController.WholeBodyControllerParameters;
import us.ihmc.wholeBodyController.WholeBodyTrajectoryTest;

public class AtlasWholeBodyTrajectoryTest extends WholeBodyTrajectoryTest
{
   static private final AtlasRobotModel atlasRobotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_DUAL_ROBOTIQ, AtlasRobotModel.AtlasTarget.SIM, false);
   static private SDFFullRobotModel actualRobotModel =  atlasRobotModel.createFullRobotModel();
   static private AtlasWholeBodyIK wbSolver = new AtlasWholeBodyIK( atlasRobotModel );
   
   static private SimulationConstructionSet scs;

   private final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromEnvironmentVariables();   
   private final boolean VISUALIZE_GUI = simulationTestingParameters.getCreateGUI();

   static FullRobotModelVisualizer modelVisualizer;
   
   public AtlasWholeBodyTrajectoryTest() throws InterruptedException
   {
      super(actualRobotModel, wbSolver);
      
      for (RobotSide robotSide : RobotSide.values)
      {
         actualRobotModel.getOneDoFJointByName(atlasRobotModel.getJointMap().getArmJointName(robotSide, ArmJointName.SHOULDER_YAW)).setQ(0.500); //arm_shy
         actualRobotModel.getOneDoFJointByName(atlasRobotModel.getJointMap().getArmJointName(robotSide, ArmJointName.SHOULDER_ROLL)).setQ(robotSide.negateIfRightSide(-1.0)); //arm_shx
         actualRobotModel.getOneDoFJointByName(atlasRobotModel.getJointMap().getArmJointName(robotSide, ArmJointName.ELBOW_PITCH)).setQ(2.00); //arm_ely
         actualRobotModel.getOneDoFJointByName(atlasRobotModel.getJointMap().getArmJointName(robotSide, ArmJointName.ELBOW_ROLL)).setQ(robotSide.negateIfRightSide(0.6)); //arm_elx
         actualRobotModel.getOneDoFJointByName(atlasRobotModel.getJointMap().getArmJointName(robotSide, ArmJointName.WRIST_PITCH)).setQ(0.000); //arm_wry
         actualRobotModel.getOneDoFJointByName(atlasRobotModel.getJointMap().getArmJointName(robotSide, ArmJointName.WRIST_ROLL)).setQ(robotSide.negateIfRightSide(0)); //arm_wrx
      }
      
      if( scs == null && VISUALIZE_GUI )
      {
         scs = new SimulationConstructionSet( atlasRobotModel.createSdfRobot(false) );
         modelVisualizer = new FullRobotModelVisualizer( scs, actualRobotModel,  0.01 );
         scs.startOnAThread(); 
         
         scs.maximizeMainWindow();
         modelVisualizer.update(0);
         
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
      return modelVisualizer;
   }
   
   @Override 
   public SimulationConstructionSet getSimulationConstructionSet()
   {
      return scs;
   }

   @Ignore
   @AverageDuration(duration = 0.0)
   @Test(timeout = 300000)
   public void test()
   {

   }
}
