package us.ihmc.atlas.hikSim;

import org.junit.Test;

import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.wholeBodyController.WholeBodyIkSolver;

public class AtlasWholeBodyIkSolverTest
{
   @Test
   public void testRightHandinPMode()
   {
      new AtlasWholeBodyIkSolverTestFactory(RobotSide.RIGHT, WholeBodyIkSolver.ControlledDoF.DOF_3P, WholeBodyIkSolver.ControlledDoF.DOF_NONE);
   }
   
   @Test
   public void testLeftHandinPMode(){
      new AtlasWholeBodyIkSolverTestFactory(RobotSide.LEFT, WholeBodyIkSolver.ControlledDoF.DOF_NONE, WholeBodyIkSolver.ControlledDoF.DOF_3P);
   }
   
}
