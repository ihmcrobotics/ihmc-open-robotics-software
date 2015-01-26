package us.ihmc.wholeBodyController;

import static org.junit.Assert.fail;

import java.io.IOException;
import java.util.ArrayList;

import javax.vecmath.Vector3d;

import org.junit.Test;

import us.ihmc.SdfLoader.FullRobotModelVisualizer;
import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.bambooTools.BambooTools;
import us.ihmc.utilities.Pair;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.utilities.trajectory.TrajectoryND;
import us.ihmc.utilities.trajectory.TrajectoryND.WaypointND;
import us.ihmc.wholeBodyController.WholeBodyIkSolver.ComputeOption;
import us.ihmc.wholeBodyController.WholeBodyIkSolver.ControlledDoF;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;

public abstract class WholeBodyTrajectoryTest
{
   private SDFFullRobotModel actualRobotModel;
   private SDFFullRobotModel desiredRobotModel;
   private WholeBodyIkSolver wbSolver;

   static private final  YoVariableRegistry registry = new YoVariableRegistry("WholeBodyIkSolverTestFactory_Registry"); 

   private final ArrayList<Pair<ReferenceFrame, ReferenceFrame>> handArrayList = new ArrayList<Pair<ReferenceFrame, ReferenceFrame>>();

   static private final boolean DEBUG = false && !BambooTools.isRunningOnBamboo();


   public abstract WholeBodyControllerParameters getRobotModel();
   public abstract FullRobotModelVisualizer getFullRobotModelVisualizer();
   public abstract SimulationConstructionSet getSimulationConstructionSet();


   public WholeBodyTrajectoryTest(SDFFullRobotModel actualRobotModel, WholeBodyIkSolver solver)
   {
      this.actualRobotModel = actualRobotModel;
      desiredRobotModel = getRobotModel().createFullRobotModel();
      this.wbSolver = solver; 

      Vector3d rootPosition = new Vector3d(0,0, 0.93);
      actualRobotModel.getRootJoint().setPosition( rootPosition );   
   }    
   
   @Test
   public void testTrajectory() throws Exception
   {
      wbSolver.setVerbose(false);
     
      wbSolver.setNumberOfControlledDoF(RobotSide.LEFT, ControlledDoF.DOF_3P);
      wbSolver.setNumberOfControlledDoF(RobotSide.RIGHT, ControlledDoF.DOF_3P);
      
      ReferenceFrame soleFrame = actualRobotModel.getSoleFrame(RobotSide.RIGHT);
      
      ReferenceFrame targetL = ReferenceFrame.constructBodyFrameWithUnchangingTranslationFromParent(
            "targetL", soleFrame, new Vector3d( 0.4, -0.15, 1.3 ) );
      
      ReferenceFrame targetR = ReferenceFrame.constructBodyFrameWithUnchangingTranslationFromParent(
            "targetR", soleFrame, new Vector3d( 0.4, 0.35, 1.0 ) );
      
      wbSolver.setHandTarget(actualRobotModel, RobotSide.LEFT,  targetL );
      wbSolver.setHandTarget(actualRobotModel, RobotSide.RIGHT,  targetR );
      
      int ret = wbSolver.compute(actualRobotModel, desiredRobotModel, ComputeOption.RESEED );
      ret = wbSolver.compute(actualRobotModel, desiredRobotModel, ComputeOption.USE_JOINTS_CACHE );
      
      if( ret >=0 )
      {
         TrajectoryND trajectory;
         
         trajectory = WholeBodyTrajectory.createJointSpaceTrajectory(wbSolver, 
               actualRobotModel.getOneDoFJoints(), 
               desiredRobotModel.getOneDoFJoints());
         
         Pair<Boolean, WaypointND> result = trajectory.getNextInterpolatedPoints(0.01);
         
         while( result.first().booleanValue() == false)
         {
            actualRobotModel.copyAllJointsButKeepOneFootFixed( result.second().position, RobotSide.RIGHT );
            result = trajectory.getNextInterpolatedPoints(0.01);
            Thread.sleep(10);
            getFullRobotModelVisualizer().update(0);
         }
         
      }
      else{
         fail("no solution found\n");
      }
      
      //-------------------------------------------------------------
      targetL = ReferenceFrame.constructBodyFrameWithUnchangingTranslationFromParent(
            "targetL", soleFrame, new Vector3d( 0.4, -0.15, 0.7 ) );
      
      wbSolver.setHandTarget(actualRobotModel, RobotSide.LEFT,  targetL );
      wbSolver.setHandTarget(actualRobotModel, RobotSide.RIGHT,  targetR );
      
      ret = wbSolver.compute(actualRobotModel, desiredRobotModel, ComputeOption.USE_JOINTS_CACHE );
      
      if( ret >=0 )
      {
         TrajectoryND trajectory;
         
         wbSolver.setVerbose(true);
         trajectory = WholeBodyTrajectory.createJointSpaceTrajectory(wbSolver, 
               actualRobotModel.getOneDoFJoints(), 
               desiredRobotModel.getOneDoFJoints());
         
         wbSolver.setVerbose(false);
         
         Pair<Boolean, WaypointND> result = trajectory.getNextInterpolatedPoints(0.01);
         
         while( result.first().booleanValue() == false)
         {
            actualRobotModel.copyAllJointsButKeepOneFootFixed( result.second().position, RobotSide.RIGHT );
            result = trajectory.getNextInterpolatedPoints(0.01);
            Thread.sleep(10);
            getFullRobotModelVisualizer().update(0);
         }
      }
      else{
         fail("no solution found\n");
      }
      
   }

}
