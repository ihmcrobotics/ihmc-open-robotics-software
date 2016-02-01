package us.ihmc.wholeBodyController;

import us.ihmc.SdfLoader.SDFFullHumanoidRobotModel;
import us.ihmc.simulationconstructionset.Robot;

public class WholeBodyTrajectoryTestHelper
{
   private WholeBodyControllerParameters wholeBodyControllerParameters;
   private Robot robot;
   private SDFFullHumanoidRobotModel sdfFullRobotModel;
   private WholeBodyIkSolver wholeBodyIkSolver;
   
   public WholeBodyTrajectoryTestHelper(WholeBodyControllerParameters wholeBodyControllerParameters, Robot robot, SDFFullHumanoidRobotModel sdfFullRobotModel, WholeBodyIkSolver wholeBodyIkSolver)
   {
      this.wholeBodyControllerParameters = wholeBodyControllerParameters;
      this.robot = robot;
      this.sdfFullRobotModel = sdfFullRobotModel;
      this.wholeBodyIkSolver = wholeBodyIkSolver;
   }
   
   public WholeBodyControllerParameters getRobotModel()
   {
      return wholeBodyControllerParameters;
   }
   
   public Robot getRobot()
   {
      return robot;
   }
   
   public SDFFullHumanoidRobotModel getActualRobotModel()
   {
      return sdfFullRobotModel;
   }
   
   public WholeBodyIkSolver getWholeBodyIkSolver()
   {
      return wholeBodyIkSolver;
   }
}
