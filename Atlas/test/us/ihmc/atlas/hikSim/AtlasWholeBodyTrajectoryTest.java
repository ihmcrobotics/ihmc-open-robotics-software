package us.ihmc.atlas.hikSim;

import org.junit.Test;

import us.ihmc.SdfLoader.FullRobotModelVisualizer;
import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.atlas.AtlasWholeBodyIK;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.bambooTools.BambooTools;
import us.ihmc.utilities.ThreadTools;
import us.ihmc.wholeBodyController.WholeBodyControllerParameters;
import us.ihmc.wholeBodyController.WholeBodyTrajectoryTest;

public class AtlasWholeBodyTrajectoryTest extends WholeBodyTrajectoryTest
{
   static private final AtlasRobotModel atlasRobotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_DUAL_ROBOTIQ, AtlasRobotModel.AtlasTarget.SIM, false);
   static private SDFFullRobotModel actualRobotModel =  atlasRobotModel.createFullRobotModel();
   static private AtlasWholeBodyIK wbSolver = new AtlasWholeBodyIK( atlasRobotModel );
   
   static private SimulationConstructionSet scs;
   static private boolean VISUALIZE_GUI = true && !BambooTools.isRunningOnBamboo();   
   static FullRobotModelVisualizer modelVisualizer;
   
   public AtlasWholeBodyTrajectoryTest() throws InterruptedException
   {
      super(actualRobotModel, wbSolver);
      
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
      return modelVisualizer;
   }
   
   @Override 
   public SimulationConstructionSet getSimulationConstructionSet()
   {
      return scs;
   }

   @Test(timeout = 300000)
   public void test()
   {
    
   }



}
