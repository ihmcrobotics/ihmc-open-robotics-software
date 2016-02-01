package us.ihmc.atlas.hikSim;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.util.ArrayList;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import org.apache.commons.lang3.tuple.ImmutablePair;

import us.ihmc.SdfLoader.SDFFullHumanoidRobotModel;
import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.SdfLoader.partNames.ArmJointName;
import us.ihmc.SdfLoader.partNames.LegJointName;
import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.atlas.AtlasWholeBodyIK;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestClass;
import us.ihmc.tools.testing.TestPlanTarget;
import us.ihmc.wholeBodyController.WholeBodyHalfCylinderTargetParameters;
import us.ihmc.wholeBodyController.WholeBodyIkSolver;
import us.ihmc.wholeBodyController.WholeBodyIkSolverTest;
import us.ihmc.wholeBodyController.WholeBodyIkSolverTestHelper;

//import us.ihmc.simulationconstructionset.bambooTools.SimulationTestingParameters;

@DeployableTestClass(targets = {TestPlanTarget.Fast})
public class AtlasWholeBodyIkSolverTest extends WholeBodyIkSolverTest
{
   private final ArrayList<SideDependentList<RigidBodyTransform>>  handToFootArray = new ArrayList<SideDependentList<RigidBodyTransform>>() ;

   private  WholeBodyIkSolver wholeBodySolver;

   @Override
   public WholeBodyIkSolverTestHelper getWholeBodyIkSolverTestHelper()
   {
      AtlasRobotModel atlasRobotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_DUAL_ROBOTIQ, DRCRobotModel.RobotTarget.SCS, false);
      SDFFullHumanoidRobotModel actualRobotModel = atlasRobotModel.createFullRobotModel();
      wholeBodySolver = new AtlasWholeBodyIK(atlasRobotModel);

      initializeFullRobotModelJointAngles(actualRobotModel, atlasRobotModel);

      wholeBodySolver.getConfiguration().setMaxNumberOfAutomaticReseeds(15);    // "I am NOT Feeling Lucky" ( Davide Faconti)

      Robot robot = atlasRobotModel.createSdfRobot(false);

      WholeBodyHalfCylinderTargetParameters wholeBodyHalfCylinderTargetParameters = new WholeBodyHalfCylinderTargetParameters();
      wholeBodyHalfCylinderTargetParameters.setMaxReachRadius(0.8);
      wholeBodyHalfCylinderTargetParameters.setMaxHeight(1.5);
      wholeBodyHalfCylinderTargetParameters.setMaxTheta(Math.PI * 3.0 / 4.0);
      wholeBodyHalfCylinderTargetParameters.setRadiusIncrements(4);
      wholeBodyHalfCylinderTargetParameters.setHeightIncrements(4);
      wholeBodyHalfCylinderTargetParameters.setThetaIncrements(4);

      WholeBodyIkSolverTestHelper ret = new WholeBodyIkSolverTestHelper(atlasRobotModel, actualRobotModel, wholeBodySolver, robot,
            wholeBodyHalfCylinderTargetParameters);

      createHandTargetArrays();

      return ret;
   }

   private void createHandTargetArrays()
   {
      Quat4d  rot = new Quat4d();
      Vector3d pos = new Vector3d();
      RobotSide side;
      SideDependentList<RigidBodyTransform> handTargets = null;
      
      int counter = 0;
      
      try{
         File file = new File( "testResources/savedHandPoses.txt");
         BufferedReader br = new BufferedReader(new FileReader(file));
         
         for(String line; (line = br.readLine()) != null; ) {

            Matcher m = Pattern.compile("-?\\d+(\\.\\d+)?").matcher(line);
            
            while(m.find()) {
               double value = Double.parseDouble(m.group());
              // System.out.println("value=" + value);
               
               if( counter%14 < 7)  side = RobotSide.LEFT;
               else                 side = RobotSide.RIGHT;
               
               if( counter%14 == 0) handTargets = new SideDependentList<RigidBodyTransform>();
               
               switch( counter%7 )
               {
                  case 0: rot.setX( value ); break;
                  case 1: rot.setY( value ); break;
                  case 2: rot.setZ( value ); break;
                  case 3: rot.setW( value ); break;
                  case 4: pos.setX( value ); break;
                  case 5: pos.setY( value ); break;
                  case 6: {
                     pos.setZ( value ); 
                     
                     if( counter%14 < 7)  handTargets.set(RobotSide.LEFT,  new RigidBodyTransform( rot, pos ) );
                     else                 handTargets.set(RobotSide.RIGHT, new RigidBodyTransform( rot, pos ) );
                     
                     break;
                  }
               }
               if( counter%14 == 13 ) handToFootArray.add(handTargets);
               counter++;
           }
         }
     } 
     catch(Exception e) { e.printStackTrace(); } 
   }

   private void initializeFullRobotModelJointAngles(SDFFullRobotModel fullRobotModelToInitialize, AtlasRobotModel atlasRobotModel)
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         fullRobotModelToInitialize.getOneDoFJointByName(atlasRobotModel.getJointMap().getLegJointName(robotSide, LegJointName.HIP_YAW)).setQ(0.0);    // leg_hpz
         fullRobotModelToInitialize.getOneDoFJointByName(atlasRobotModel.getJointMap().getLegJointName(robotSide,
               LegJointName.HIP_ROLL)).setQ(robotSide.negateIfRightSide(0.1));    // leg_hpx
         fullRobotModelToInitialize.getOneDoFJointByName(atlasRobotModel.getJointMap().getLegJointName(robotSide, LegJointName.HIP_PITCH)).setQ(-0.5);    // leg_hpy
         fullRobotModelToInitialize.getOneDoFJointByName(atlasRobotModel.getJointMap().getLegJointName(robotSide, LegJointName.KNEE)).setQ(1.0);    // leg_kny
         fullRobotModelToInitialize.getOneDoFJointByName(atlasRobotModel.getJointMap().getLegJointName(robotSide, LegJointName.ANKLE_PITCH)).setQ(-0.5);    // leg_aky
         fullRobotModelToInitialize.getOneDoFJointByName(atlasRobotModel.getJointMap().getLegJointName(robotSide,
               LegJointName.ANKLE_ROLL)).setQ(robotSide.negateIfRightSide(-0.1));    // leg_akx
         fullRobotModelToInitialize.getOneDoFJointByName(atlasRobotModel.getJointMap().getArmJointName(robotSide, ArmJointName.SHOULDER_YAW)).setQ(0.500);    // arm_shy
         fullRobotModelToInitialize.getOneDoFJointByName(atlasRobotModel.getJointMap().getArmJointName(robotSide,
               ArmJointName.SHOULDER_ROLL)).setQ(robotSide.negateIfRightSide(-1.0));    // arm_shx
         fullRobotModelToInitialize.getOneDoFJointByName(atlasRobotModel.getJointMap().getArmJointName(robotSide, ArmJointName.ELBOW_PITCH)).setQ(2.00);    // arm_ely
         fullRobotModelToInitialize.getOneDoFJointByName(atlasRobotModel.getJointMap().getArmJointName(robotSide,
               ArmJointName.ELBOW_ROLL)).setQ(robotSide.negateIfRightSide(0.9));    // arm_elx
         fullRobotModelToInitialize.getOneDoFJointByName(atlasRobotModel.getJointMap().getArmJointName(robotSide, ArmJointName.FIRST_WRIST_PITCH)).setQ(0.000);    // arm_wry
         fullRobotModelToInitialize.getOneDoFJointByName(atlasRobotModel.getJointMap().getArmJointName(robotSide,
               ArmJointName.WRIST_ROLL)).setQ(robotSide.negateIfRightSide(0));    // arm_wrx
      }
   }

   @Override
   public ArrayList<ImmutablePair<FramePose, FramePose>> generatePointsForRegression(RobotSide robotSide, int numberOfPoints)
   {
      // this is intentionally null. We want to skip these tests.
      return null;
   }

   @Override
   public ArrayList<ImmutablePair<FramePose, FramePose>> generatePointsForRegression(int numberOfPoints)
   {
      // this is intentionally null. We want to skip these tests.
      return null;
   }

   @Override
   public ArrayList<SideDependentList<RigidBodyTransform>> getHandToRootArray()
   {
      return handToFootArray;
   }



}
