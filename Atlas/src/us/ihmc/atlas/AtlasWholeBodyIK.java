package us.ihmc.atlas;

import java.io.File;
import java.io.IOException;
import java.io.InputStream;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.nio.file.StandardCopyOption;
import java.util.HashSet;

import javax.xml.parsers.ParserConfigurationException;

import org.xml.sax.SAXException;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.utilities.hierarchicalKinematics.HierarchicalTask_BodyOrientation;
import us.ihmc.utilities.hierarchicalKinematics.HierarchicalTask_BodyPosition;
import us.ihmc.utilities.math.Vector64F;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.wholeBodyController.WholeBodyControllerParameters;
import us.ihmc.wholeBodyController.WholeBodyIkSolver;

public class AtlasWholeBodyIK extends WholeBodyIkSolver
{ 

   @Override 
   public String getEndEffectorLinkName(RobotSide side)  {
      if( side == RobotSide.LEFT) 
         return "l_ee_link";
      else
         return "r_ee_link";
   }

   @Override 
   public String getFootLinkName(RobotSide side) {
      if( side == RobotSide.LEFT) 
         return "l_foot";
      else
         return "r_foot";
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

   static private String modelLocationPathString;

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
      
      HierarchicalTask_BodyPosition task_ee_pose_R_pos = task_end_effector_translations.get(RIGHT);
      HierarchicalTask_BodyPosition task_ee_pose_L_pos = task_end_effector_translations.get(LEFT);
      HierarchicalTask_BodyOrientation task_ee_pose_R_rot = task_end_effector_rotations.get(RIGHT);
      HierarchicalTask_BodyOrientation task_ee_pose_L_rot = task_end_effector_rotations.get(LEFT);

      int rleg_akx = jointNamesMap.get("r_leg_akx");
      int rleg_aky = jointNamesMap.get("r_leg_aky");
      int rleg_kny = jointNamesMap.get("r_leg_kny");
      int rleg_hpy = jointNamesMap.get("r_leg_hpy");
      int rleg_hpx = jointNamesMap.get("r_leg_hpx");
      int rleg_hpz = jointNamesMap.get("r_leg_hpz");

      int lleg_akx = jointNamesMap.get("l_leg_akx");
      int lleg_aky = jointNamesMap.get("l_leg_aky");
      int lleg_kny = jointNamesMap.get("l_leg_kny");
      int lleg_hpy = jointNamesMap.get("l_leg_hpy");
      int lleg_hpx = jointNamesMap.get("l_leg_hpx");
      int lleg_hpz = jointNamesMap.get("l_leg_hpz");

      int back_bkz = jointNamesMap.get("back_bkz");
      int back_bky = jointNamesMap.get("back_bky");
      int back_bkx = jointNamesMap.get("back_bkx");

      int larm_elx = jointNamesMap.get("l_arm_elx");
      int rarm_elx = jointNamesMap.get("r_arm_elx");

      task_ee_pose_L_pos.setMaximumError(0.2);
      task_ee_pose_L_rot.setMaximumError(0.4);

      task_ee_pose_R_pos.setMaximumError(0.2);
      task_ee_pose_R_rot.setMaximumError(0.4);
      task_joints_pose.setMaximumError(0.6);

      task_ee_pose_R_pos.setEnabled(false);
      task_ee_pose_R_rot.setEnabled(false);
      task_ee_pose_L_pos.setEnabled(false);
      task_ee_pose_L_rot.setEnabled(false);

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

      task_leg_pose.setWeightsJointSpace(joint_weights);

      //TODO
      Vector64F target_left_foot = new Vector64F(7, 0, 2 * 0.11, 0, 0, 0, 0, 1);
      task_leg_pose.setTarget(target_left_foot, 0.001);

      //--------------------------------------------------
      // control only the X and Y directions of the COM
      //TODO this might not work when the footroot is not flat on the ground
      task_com_position.setWeightsTaskSpace(new Vector64F(3, 1, 1, 0));

      // Don't move the arm to keep the balance. But torso need to be activated
      joint_weights.set(back_bkz, 1);
      joint_weights.set(back_bky, 1);
      joint_weights.set(back_bkx, 1);

      task_com_position.setWeightsJointSpace(joint_weights);

      // TODO make this the center of the feet
      Vector64F target_com = new Vector64F(3, 0, 0.11, 0);    
      task_com_position.setTarget(target_com, 0.01);
      //--------------------------------------------------
      // control position and rotation (6 DoF) of the end effector
      Vector64F weights_ee_pos = new Vector64F(3, 1, 1, 1);
      Vector64F weights_ee_rot = new Vector64F(3, 1, 1, 1);

      task_ee_pose_L_pos.setWeightsTaskSpace(weights_ee_pos);
      task_ee_pose_L_rot.setWeightsTaskSpace(weights_ee_rot);
      task_ee_pose_R_pos.setWeightsTaskSpace(weights_ee_pos);
      task_ee_pose_R_rot.setWeightsTaskSpace(weights_ee_rot);

      for (int i = 0; i < getNumberDoFperArm(); i++)
      {
         int index = armJointIds.get(RobotSide.LEFT)[i];
         joint_weights.set(index, 1);
      }
      task_ee_pose_L_pos.setWeightsJointSpace(joint_weights);
      task_ee_pose_L_rot.setWeightsJointSpace(joint_weights);

      for (int i = 0; i < getNumberDoFperArm(); i++)
      {
         int index = armJointIds.get(RobotSide.LEFT)[i];
         joint_weights.set(index, 0);
         index = armJointIds.get(RobotSide.RIGHT)[i];
         joint_weights.set(index, 1);
      }
      task_ee_pose_R_pos.setWeightsJointSpace(joint_weights);
      task_ee_pose_R_rot.setWeightsJointSpace(joint_weights);

      //--------------------------------------------------
      // control position and rotation (6 DoF) of the end effector
      Vector64F weights_jointpose = new Vector64F( getNumberOfJoints() );
      weights_jointpose.zero();
      coupledJointWeights.zero();

      // these are the joints to be knees
      weights_jointpose.set(rleg_kny, 0.5);
      weights_jointpose.set(lleg_kny, 0.5);

      // this is used to keep the pelvis straight
      // what you want is:
      //  q(r_leg_hpy) = - q(leg_aky) - q(leg_kny);
      weights_jointpose.set(rleg_hpy, 2);
      coupledJointWeights.set(rleg_hpy, rleg_aky, -1);
      coupledJointWeights.set(rleg_hpy, rleg_kny, -1);

      // back
      weights_jointpose.set(back_bkz, 1);
      weights_jointpose.set(back_bky, 1);
      weights_jointpose.set(back_bkx, 1);

      // try keep hpx mirrored 
      weights_jointpose.set(rleg_hpx, 1);
      weights_jointpose.set(lleg_hpx, 1);
      coupledJointWeights.set(rleg_hpx, lleg_hpx, -1);
      coupledJointWeights.set(lleg_hpx, rleg_hpx, -1);

      // keep this to zero if you can
      weights_jointpose.set(rleg_hpz, 0.2);
      weights_jointpose.set(lleg_hpz, 0.2);

      //elbows
      weights_jointpose.set(larm_elx, 1.0);
      weights_jointpose.set(rarm_elx, 1.0);

      preferedJointPose.set(larm_elx, 1.0);
      preferedJointPose.set(rarm_elx, -1.0);

      task_joints_pose.setWeightsTaskSpace(weights_jointpose);
      task_joints_pose.setWeightsJointSpace(joint_weights);
      task_joints_pose.setCoupledJointWeights(coupledJointWeights);

      task_joints_pose.setTarget(preferedJointPose, 35);

   }

}
