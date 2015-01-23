package us.ihmc.wholeBodyController;

import static org.junit.Assert.*;

import java.io.IOException;
import java.util.ArrayList;

import javax.vecmath.Vector3d;

import org.junit.Test;

import us.ihmc.SdfLoader.FullRobotModelVisualizer;
import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.bambooTools.BambooTools;
import us.ihmc.utilities.Pair;
import us.ihmc.utilities.math.geometry.FramePoint;
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
   private WholeBodyIkSolver hikSolver;

   static private final  YoVariableRegistry registry = new YoVariableRegistry("WholeBodyIkSolverTestFactory_Registry"); 

   private final ArrayList<Pair<ReferenceFrame, ReferenceFrame>> handArrayList = new ArrayList<Pair<ReferenceFrame, ReferenceFrame>>();

   static private final boolean DEBUG = false && !BambooTools.isRunningOnBamboo();


   public abstract WholeBodyControllerParameters getRobotModel();
   public abstract FullRobotModelVisualizer getFullRobotModelVisualizer();
   public abstract SimulationConstructionSet getSimulationConstructionSet();


   public WholeBodyTrajectoryTest(SDFFullRobotModel actualRobotModel)
   {
      this.actualRobotModel = actualRobotModel;
      desiredRobotModel = getRobotModel().createFullRobotModel();

      try{
         hikSolver = new WholeBodyIkSolver(getRobotModel(), actualRobotModel);
      }
      catch (IOException e) {
         e.printStackTrace();
      }   

      Vector3d rootPosition = new Vector3d(0,0, 0.93);
      actualRobotModel.getRootJoint().setPosition( rootPosition );   
   }    
   
   @Test
   public void testTrajectory() throws Exception
   {
      hikSolver.setVerbose(false);
     
      hikSolver.setNumberOfControlledDoF(RobotSide.LEFT, ControlledDoF.DOF_3P);
      hikSolver.setNumberOfControlledDoF(RobotSide.RIGHT, ControlledDoF.DOF_3P);
      
      ReferenceFrame soleFrame = actualRobotModel.getSoleFrame(RobotSide.RIGHT);
      
      ReferenceFrame targetL = ReferenceFrame.constructBodyFrameWithUnchangingTranslationFromParent(
            "targetL", soleFrame, new Vector3d( 0.4, -0.15, 1.3 ) );
      
      ReferenceFrame targetR = ReferenceFrame.constructBodyFrameWithUnchangingTranslationFromParent(
            "targetR", soleFrame, new Vector3d( 0.4, 0.35, 1.0 ) );
      
      hikSolver.setHandTarget(RobotSide.LEFT,  targetL );
      hikSolver.setHandTarget(RobotSide.RIGHT,  targetR );
      
      int ret = hikSolver.compute(desiredRobotModel, ComputeOption.RESEED );
      ret = hikSolver.compute(desiredRobotModel, ComputeOption.USE_JOINTS_CACHE );
      
      if( ret >=0 )
      {
         TrajectoryND trajectory;
         
         trajectory = WholeBodyTrajectory.createJointSpaceTrajectory(hikSolver, 
               actualRobotModel.getOneDoFJoints(), 
               desiredRobotModel.getOneDoFJoints());
         
         Pair<Boolean, WaypointND> result = trajectory.getNextInterpolatedPoints(0.01);
         
         while( result.first().booleanValue() == false)
         {
            actualRobotModel.copyAllJointsButMaintainOneFootFixed( result.second().position, RobotSide.RIGHT );
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
      
      hikSolver.setHandTarget(RobotSide.LEFT,  targetL );
      hikSolver.setHandTarget(RobotSide.RIGHT,  targetR );
      
      ret = hikSolver.compute(desiredRobotModel, ComputeOption.USE_JOINTS_CACHE );
      
      if( ret >=0 )
      {
         TrajectoryND trajectory;
         
         hikSolver.setVerbose(true);
         trajectory = WholeBodyTrajectory.createJointSpaceTrajectory(hikSolver, 
               actualRobotModel.getOneDoFJoints(), 
               desiredRobotModel.getOneDoFJoints());
         
         hikSolver.setVerbose(false);
         
         Pair<Boolean, WaypointND> result = trajectory.getNextInterpolatedPoints(0.01);
         
         while( result.first().booleanValue() == false)
         {
            actualRobotModel.copyAllJointsButMaintainOneFootFixed( result.second().position, RobotSide.RIGHT );
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
