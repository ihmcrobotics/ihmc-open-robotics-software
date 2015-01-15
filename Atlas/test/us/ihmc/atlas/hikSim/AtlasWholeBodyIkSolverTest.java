package us.ihmc.atlas.hikSim;

import java.util.ArrayList;

import org.junit.Test;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.utilities.humanoidRobot.partNames.ArmJointName;
import us.ihmc.utilities.humanoidRobot.partNames.LegJointName;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.utilities.robotSide.SideDependentList;
import us.ihmc.wholeBodyController.WholeBodyControllerParameters;
import us.ihmc.wholeBodyController.WholeBodyIkSolver;
import us.ihmc.wholeBodyController.WholeBodyIkSolver.ControlledDoF;
import us.ihmc.wholeBodyController.WholeBodyIkSolverTestFactory;

public class AtlasWholeBodyIkSolverTest extends WholeBodyIkSolverTestFactory
{
   private final ArrayList<ReferenceFrame> handTargetArray = new ArrayList<ReferenceFrame>();
   private final static AtlasRobotModel atlasRobotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_DUAL_ROBOTIQ, AtlasRobotModel.AtlasTarget.SIM, false);

   public AtlasWholeBodyIkSolverTest()
   {
      super( atlasRobotModel );
   }

   @Test
   public void testRightHandPose()
   {
      createHandTargetArray(RobotSide.RIGHT);
      
      for(ReferenceFrame frame : handTargetArray)
      {
         boolean solutionfound = this.testHandsPose( ControlledDoF.DOF_NONE, ControlledDoF.DOF_3P, null, frame );
      }
   }


   @Override
   public void initializeJointAngles(SDFFullRobotModel fullRobotModelToInitialize)
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         fullRobotModelToInitialize.getOneDoFJointByName(atlasRobotModel.getJointMap().getLegJointName(robotSide, LegJointName.HIP_YAW)).setQ(0.0); //leg_hpz
         fullRobotModelToInitialize.getOneDoFJointByName(atlasRobotModel.getJointMap().getLegJointName(robotSide, LegJointName.HIP_ROLL)).setQ(robotSide.negateIfRightSide(0.062)); //leg_hpx
         fullRobotModelToInitialize.getOneDoFJointByName(atlasRobotModel.getJointMap().getLegJointName(robotSide, LegJointName.HIP_PITCH)).setQ(-0.233); //leg_hpy
         fullRobotModelToInitialize.getOneDoFJointByName(atlasRobotModel.getJointMap().getLegJointName(robotSide, LegJointName.KNEE)).setQ(0.518); //leg_kny
         fullRobotModelToInitialize.getOneDoFJointByName(atlasRobotModel.getJointMap().getLegJointName(robotSide, LegJointName.ANKLE_PITCH)).setQ(-0.276); //leg_aky
         fullRobotModelToInitialize.getOneDoFJointByName(atlasRobotModel.getJointMap().getLegJointName(robotSide, LegJointName.ANKLE_ROLL)).setQ(robotSide.negateIfRightSide(-0.062)); //leg_akx

         fullRobotModelToInitialize.getOneDoFJointByName(atlasRobotModel.getJointMap().getArmJointName(robotSide, ArmJointName.SHOULDER_YAW)).setQ(0.300); //arm_shy
         fullRobotModelToInitialize.getOneDoFJointByName(atlasRobotModel.getJointMap().getArmJointName(robotSide, ArmJointName.SHOULDER_ROLL)).setQ(robotSide.negateIfRightSide(-1.30)); //arm_shx
         fullRobotModelToInitialize.getOneDoFJointByName(atlasRobotModel.getJointMap().getArmJointName(robotSide, ArmJointName.ELBOW_PITCH)).setQ(2.00); //arm_ely
         fullRobotModelToInitialize.getOneDoFJointByName(atlasRobotModel.getJointMap().getArmJointName(robotSide, ArmJointName.ELBOW_ROLL)).setQ(robotSide.negateIfRightSide(0.498)); //arm_elx
         fullRobotModelToInitialize.getOneDoFJointByName(atlasRobotModel.getJointMap().getArmJointName(robotSide, ArmJointName.WRIST_PITCH)).setQ(0.000); //arm_wry
         fullRobotModelToInitialize.getOneDoFJointByName(atlasRobotModel.getJointMap().getArmJointName(robotSide, ArmJointName.WRIST_ROLL)).setQ(robotSide.negateIfRightSide(-0.004)); //arm_wrx
      }
   }      

   private void createHandTargetArray(RobotSide robotSide)
   {
      //Basically creates a half cylinder of points for the robot to try to reach in its (Right/Left) arm workspace.
      double sign = (robotSide == RobotSide.RIGHT ? -1.0 : 1.0);
      double maxReachRadius = 0.5;
      double maxHeight = 1.0;
      double maxTheta = Math.PI * 3 / 4;
      int radiusIncrements = 2;
      int heightIncrements = 6;
      int thetaIncrements = 4;
      
     // int totalNumberofPoints = radiusIncrements * heightIncrements * thetaIncrements;
      
      for (int z_int = 1; z_int < (1 + heightIncrements); z_int++)
      {
         for (int theta_int = 0; theta_int < thetaIncrements; theta_int++)
         {
            for (int radius_int = 1; radius_int < (1+ radiusIncrements); radius_int++)
            {
               double height = (maxHeight * z_int) / heightIncrements;
               double reachRadius = (maxReachRadius * radius_int) / radiusIncrements;
               double theta = sign * maxTheta * theta_int / thetaIncrements;      
               
               FramePoint point = new FramePoint(ReferenceFrame.getWorldFrame(),
                     reachRadius * Math.cos(theta) + 0.1, // offset to take the shape of robot into account.
                     reachRadius * Math.sin(theta) + (sign*0.2),
                     height - maxHeight/2.0);
               
               FrameVector zaxis = new FrameVector(ReferenceFrame.getWorldFrame(), 0.0, 0.0, 1.0);
               ReferenceFrame desiredEndEffectorFrame = ReferenceFrame.constructReferenceFrameFromPointAndZAxis("desiredFrame", point, zaxis);
               
               handTargetArray.add(desiredEndEffectorFrame);
               System.out.println(getClass().getSimpleName() + ": point added to targetArrayList" + point.toString());
            }
         }
      }
   }

}
