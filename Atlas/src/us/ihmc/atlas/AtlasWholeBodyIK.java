package us.ihmc.atlas;

import java.io.File;
import java.io.IOException;
import java.io.InputStream;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.nio.file.StandardCopyOption;
import java.util.HashSet;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.utilities.hierarchicalKinematics.HierarchicalTask_BodyPose;
import us.ihmc.utilities.hierarchicalKinematics.RobotModel;
import us.ihmc.utilities.math.Vector64F;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.wholeBodyController.WholeBodyControllerParameters;
import us.ihmc.wholeBodyController.WholeBodyIkSolver;

public class AtlasWholeBodyIK extends WholeBodyIkSolver
{ 

   @Override 
   public String getGripperPalmLinkName(RobotSide side)  {
      return (side == RobotSide.LEFT) ? "l_gripper_palm" :  "r_gripper_palm";
   }

   @Override 
   public String getGripperAttachmentLinkName(RobotSide side)  {
      return (side == RobotSide.LEFT) ? "l_gripper_attachment" :  "r_gripper_attachment";
   }


   @Override 
   public String getFootLinkName(RobotSide side) {
      return (side == RobotSide.LEFT) ? "l_foot" : "r_foot";
   }

   @Override 
   public int getNumberDoFperArm(){
      return 6;
   }
   @Override 
   public int getNumberDoFperLeg(){
      return 6;
   }

   @Override 
   public RobotSide getSideOfTheFootRoot(){
      return RobotSide.RIGHT;
   }

   @Override 
   public ReferenceFrame  getRootFrame( SDFFullRobotModel actualSdfModel)
   {
      // important: this is AfterJoint, not beforeJoint, because the direction on the URDF model is reversed when
      // compared with the SDF model.
      return actualSdfModel.getOneDoFJointByName("r_leg_akx").getFrameAfterJoint();
   }

   static private String modelLocationPathString = null;

   @Override 
   public String getURDFConfigurationFileName() throws IOException
   {
      if( modelLocationPathString == null)
      {
         String urdf_filename = "models/atlas_v4_robotiq.wb.urdf";

         /// load the file. But first copy it on $HOME/.ihmc
         InputStream resource = getClass().getClassLoader().getResourceAsStream(urdf_filename);
         Files.createDirectories(Paths.get(System.getProperty("user.home"), ".ihmc", "models"));
         String[] string_parts = urdf_filename.split("/");
         Path modelLocationPath = Paths.get(System.getProperty("user.home"), ".ihmc", "models", string_parts[string_parts.length - 1]);
         modelLocationPathString = modelLocationPath.toAbsolutePath().toString();

         if (resource != null)
         {
            Files.copy(resource, modelLocationPath, StandardCopyOption.REPLACE_EXISTING);
         }
         else
         {
            throw new IOException("urdf not found in resources/" + urdf_filename);
         }
      }
      return modelLocationPathString;
   }

   public AtlasWholeBodyIK(WholeBodyControllerParameters robot_model) 
   {
      super(robot_model);
   }

   @Override
   protected HashSet<String> configureCollisionAvoidance(String urdfFileName ) throws Exception
   {
      HashSet<String> listOfEnabledColliders = new  HashSet<String>();
      hierarchicalSolver.collisionAvoidance.loadURDF(new File( urdfFileName ));

      listOfEnabledColliders.add("pelvis");
      listOfEnabledColliders.add("utorso");

      listOfEnabledColliders.add("r_larm");
      listOfEnabledColliders.add("r_uleg");
      listOfEnabledColliders.add("l_larm");
      listOfEnabledColliders.add("l_uleg");

      hierarchicalSolver.collisionAvoidance.addBodyPair("r_larm", "l_larm");

      hierarchicalSolver.collisionAvoidance.addBodyPair("r_larm", "r_uleg" );
      hierarchicalSolver.collisionAvoidance.addBodyPair("r_larm", "l_uleg" );
      hierarchicalSolver.collisionAvoidance.addBodyPair("r_larm", "pelvis");
      hierarchicalSolver.collisionAvoidance.addBodyPair("r_larm", "utorso" );

      hierarchicalSolver.collisionAvoidance.addBodyPair("r_larm", "r_uleg" );
      hierarchicalSolver.collisionAvoidance.addBodyPair("l_larm", "l_uleg" );
      hierarchicalSolver.collisionAvoidance.addBodyPair("l_larm", "pelvis");
      hierarchicalSolver.collisionAvoidance.addBodyPair("l_larm", "utorso" );

      return listOfEnabledColliders;
   }

   @Override
   protected void configureTasks()
   {
      RobotSide LEFT = RobotSide.LEFT;
      RobotSide RIGHT = RobotSide.RIGHT;

      HierarchicalTask_BodyPose task_ee_pose_R = taskEndEffectorPose.get(RIGHT);
      HierarchicalTask_BodyPose task_ee_pose_L = taskEndEffectorPose.get(LEFT);


      int rleg_akx = jointNamesToIndex.get("r_leg_akx");
      int rleg_aky = jointNamesToIndex.get("r_leg_aky");
      int rleg_kny = jointNamesToIndex.get("r_leg_kny");
      int rleg_hpy = jointNamesToIndex.get("r_leg_hpy");
      int rleg_hpx = jointNamesToIndex.get("r_leg_hpx");
      int rleg_hpz = jointNamesToIndex.get("r_leg_hpz");

      int lleg_akx = jointNamesToIndex.get("l_leg_akx");
      int lleg_aky = jointNamesToIndex.get("l_leg_aky");
      int lleg_kny = jointNamesToIndex.get("l_leg_kny");
      int lleg_hpy = jointNamesToIndex.get("l_leg_hpy");
      int lleg_hpx = jointNamesToIndex.get("l_leg_hpx");
      int lleg_hpz = jointNamesToIndex.get("l_leg_hpz");

      int back_bkz = jointNamesToIndex.get("back_bkz");
      int back_bky = jointNamesToIndex.get("back_bky");
      int back_bkx = jointNamesToIndex.get("back_bkx");

      int larm_elx = jointNamesToIndex.get("l_arm_elx");
      int rarm_elx = jointNamesToIndex.get("r_arm_elx");

      task_ee_pose_L.setMaximumError(0.3);
      task_ee_pose_R.setMaximumError(0.3);

      taskJointsPose.setMaximumError(0.6);

      this.setNumberOfControlledDoF(RIGHT, ControlledDoF.DOF_3P);
      this.setNumberOfControlledDoF(LEFT, ControlledDoF.DOF_NONE);

      //--------------------------------------------------
      // Important: you can't activate a joint in higher priorities and disable it later
      // use this vector for any setWeightJointSpace
      Vector64F joint_weights = new Vector64F( getNumberOfJoints() );
      joint_weights.zero();

      // use only the joints of legs to achieve this task. Ignore torso and arms
      for (int i = 0; i < getNumberDoFperLeg(); i++)
      {
         joint_weights.set(legJointIds.get(LEFT)[i], 1);
         joint_weights.set(legJointIds.get(RIGHT)[i], 1);
      }

      taskLegPose.setWeightsJointSpace(joint_weights);

      //TODO
      Vector64F target_left_foot = new Vector64F(7, 0, 2 * 0.11, 0, 0, 0, 0, 1);
      taskLegPose.setTarget(target_left_foot, 0.001);

      //--------------------------------------------------
      // Don't move the arm to keep the balance. But torso need to be activated
      joint_weights.set(back_bkz, 1);
      joint_weights.set(back_bky, 1);
      joint_weights.set(back_bkx, 1);

      taskComPosition.setWeightsJointSpace(joint_weights);

      // this value shall be overwritten later
      Vector64F target_com = new Vector64F(3, 0, 0, 0);    
      taskComPosition.setTarget(target_com, 0.01);
      //--------------------------------------------------
      // control position and rotation (6 DoF) of the end effector

      for (int i = 0; i < getNumberDoFperArm(); i++)
      {
         int index = armJointIds.get(RobotSide.LEFT)[i];
         joint_weights.set(index, 1);
      }
      task_ee_pose_L.setWeightsJointSpace(joint_weights);


      for (int i = 0; i < getNumberDoFperArm(); i++)
      {
         int index = armJointIds.get(RobotSide.LEFT)[i];
         joint_weights.set(index, 0);
         index = armJointIds.get(RobotSide.RIGHT)[i];
         joint_weights.set(index, 1);
      }
      task_ee_pose_R.setWeightsJointSpace(joint_weights);

      //--------------------------------------------------
      // control position and rotation (6 DoF) of the end effector
      Vector64F weights_jointpose = new Vector64F( getNumberOfJoints() );
      weights_jointpose.zero();
      coupledJointWeights.zero();

      // these are the joints to be knees
      weights_jointpose.set(rleg_kny, 0.4);
      weights_jointpose.set(lleg_kny, 0.4);

      // this is used to keep the pelvis straight
      // what you want is:
      //  q(r_leg_hpy) = - q(leg_aky) - q(leg_kny);
      weights_jointpose.set(rleg_hpy, 2);
      coupledJointWeights.set(rleg_hpy, rleg_aky, -1);
      coupledJointWeights.set(rleg_hpy, rleg_kny, -1);

      // back
      weights_jointpose.set(back_bkz, 0.5);
      weights_jointpose.set(back_bky, 1.5);
      weights_jointpose.set(back_bkx, 1.5);

      // try keep hpx mirrored 
      weights_jointpose.set(rleg_hpx, 1);
      weights_jointpose.set(lleg_hpx, 1);
      coupledJointWeights.set(rleg_hpx, lleg_hpx, -1);
      coupledJointWeights.set(lleg_hpx, rleg_hpx, -1);

      // keep this to zero if you can
      weights_jointpose.set(rleg_hpz, 0.2);
      weights_jointpose.set(lleg_hpz, 0.2);

      //elbows
    /*  weights_jointpose.set(larm_elx, 0.1);
      weights_jointpose.set(rarm_elx, 0.1);
      preferedJointPose.set(larm_elx, 1.0);
      preferedJointPose.set(rarm_elx, -1.0);
*/
      RobotModel model = this.getHierarchicalSolver().getRobotModel(); 
      for ( int index = 0; index< weights_jointpose.getNumElements(); index++ )
      {
         if( Math.abs( weights_jointpose.get(index)) < 0.0001 )
         {
           //  weights_jointpose.set(index, 0.01 );
          //  preferedJointPose.set( index,  0.5*(model.q_min(index)+ model.q_max(index)) );
         }
      }

      taskJointsPose.setWeightsTaskSpace(weights_jointpose);
      taskJointsPose.setWeightsJointSpace(joint_weights);
      taskJointsPose.setCoupledJointWeights(coupledJointWeights);

      taskJointsPose.setTarget(preferedJointPose, 3);
   }

   @Override
   public void reseedCachedModel()
   {
      Vector64F randQ = getHierarchicalSolver().getRandomQ();

      for (int i = 0; i < getNumberOfJoints(); i++)
      {
         boolean partOfQuietArm = false;

         for(RobotSide robotSide: RobotSide.values())
         {
            if( keepArmQuiet.get(robotSide).isTrue())
            {
               for(int j=0; j< getNumberDoFperArm(); j++){
                  if ( armJointIds.get(robotSide)[j] == i){
                     partOfQuietArm = true;
                     break;
                  }
               }
            }
         }           

         if( !partOfQuietArm ) {
            cachedAnglesQ.set(i, randQ.get(i));
         }
      }
      
      cachedAnglesQ.set(jointNamesToIndex.get("l_leg_akx"), 0.0);
      cachedAnglesQ.set(jointNamesToIndex.get("r_leg_akx"), 0.0);
      
      cachedAnglesQ.set(jointNamesToIndex.get("l_leg_aky"), -1.1);
      cachedAnglesQ.set(jointNamesToIndex.get("r_leg_aky"), -1.1);
      
      cachedAnglesQ.set(jointNamesToIndex.get("l_leg_kny"), 2.2);
      cachedAnglesQ.set(jointNamesToIndex.get("r_leg_kny"), 2.2);
      
      cachedAnglesQ.set(jointNamesToIndex.get("l_leg_hpy"), -1.1);
      cachedAnglesQ.set(jointNamesToIndex.get("r_leg_hpy"), -1.1);
      
      cachedAnglesQ.set(jointNamesToIndex.get("l_leg_hpx"), 0.0);
      cachedAnglesQ.set(jointNamesToIndex.get("r_leg_hpx"), 0.0);

      cachedAnglesQ.set(jointNamesToIndex.get("l_leg_hpz"), 0.0);
      cachedAnglesQ.set(jointNamesToIndex.get("r_leg_hpz"), 0.0);
      
      if( keepArmQuiet.get(RobotSide.LEFT).isFalse())
         cachedAnglesQ.set(jointNamesToIndex.get("l_arm_elx"), 1.0);
      if( keepArmQuiet.get(RobotSide.RIGHT).isFalse())
         cachedAnglesQ.set(jointNamesToIndex.get("r_arm_elx"), -1.0);
      
   }
   
}
