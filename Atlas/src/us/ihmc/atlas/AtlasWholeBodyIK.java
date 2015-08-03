package us.ihmc.atlas;

import java.io.File;
import java.io.IOException;
import java.io.InputStream;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.nio.file.StandardCopyOption;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Map.Entry;

import us.ihmc.robotics.hierarchicalKinematics.RobotModel;
import us.ihmc.robotics.dataStructures.Vector64F;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.wholeBodyController.WholeBodyControllerParameters;
import us.ihmc.wholeBodyController.WholeBodyIkSolver;

public class AtlasWholeBodyIK extends WholeBodyIkSolver
{ 
   @Override
   public String getGripperPalmLinkName(RobotSide side)  {
      return (side == RobotSide.LEFT) ? "l_gripper_palm" :  "r_gripper_palm";
   }

   @Override
   public String getFootLinkName(RobotSide side) {
      return (side == RobotSide.LEFT) ? "l_foot" : "r_foot";
   }
   
   @Override
   public String getKneeJointName(RobotSide side) {
      return (side == RobotSide.LEFT) ? "l_leg_kny" : "r_leg_kny";
   }
   
   @Override
   public String getJointNextToFoot(RobotSide side) {
      return  (side == RobotSide.LEFT) ? "l_leg_akx" : "r_leg_akx";
   }
   
   @Override 
   public int getNumberDoFperArm(){
      return 7;
   }
   @Override 
   public int getNumberDoFperLeg(){
      return 6;
   }

   @Override 
   public RobotSide getSideOfTheFootRoot(){
      return RobotSide.RIGHT;
   }

   public int[] getWaistJointId()
   {
      int back_bkz = jointNamesToIndex.get("back_bkz");
      int back_bky = jointNamesToIndex.get("back_bky");
      int back_bkx = jointNamesToIndex.get("back_bkx");
      
      return new int[]{ back_bkz, back_bky, back_bkx };
   }

  /* @Override 
   public ReferenceFrame  getRootFrame( SDFFullRobotModel actualSdfModel)
   {
      // important: this is AfterJoint, not beforeJoint, because the direction on the URDF model is reversed when
      // compared with the SDF model.
      return actualSdfModel.getOneDoFJointByName("r_leg_akx").getFrameAfterJoint();
   }*/

   private String modelLocationPathString = null;

   @Override 
   public String getURDFConfigurationFileName() throws IOException
   {
      if (modelLocationPathString == null)
      {
         String urdf_filename = "models/atlas_v5.wb.urdf";

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
      
      int l_wrist2 = jointNamesToIndex.get("l_arm_wry2");
      int r_wrist2 = jointNamesToIndex.get("r_arm_wry2");
      
      int l_shoulderx = jointNamesToIndex.get("l_arm_shx");
      int r_shoulderx = jointNamesToIndex.get("r_arm_shx");

      String[] jointsToMinimize = new String[]{
            "l_arm_shx", "r_arm_shx",
           // "l_arm_elx", "r_arm_elx",
            "l_arm_wrx", "r_arm_wrx"  };

      taskEndEffectorPosition.get(RIGHT).setClampingValueForTaskSpaceError(0.2);
      taskEndEffectorPosition.get(LEFT).setClampingValueForTaskSpaceError(0.2);
      
      taskEndEffectorRotation.get(RIGHT).setClampingValueForTaskSpaceError(0.4);
      taskEndEffectorRotation.get(LEFT).setClampingValueForTaskSpaceError(0.4);

      taskJointsPose.setClampingValueForTaskSpaceError(0.6);

      this.getConfiguration().setNumberOfControlledDoF(RIGHT, ControlledDoF.DOF_3P);
      this.getConfiguration().setNumberOfControlledDoF(LEFT, ControlledDoF.DOF_3P);

      //--------------------------------------------------
      // Important: you can't activate a joint in higher priorities and disable it later
      // use this vector for any setWeightJointSpace
      Vector64F joint_weights = new Vector64F( getNumberOfJoints() );
      joint_weights.zero();

      // use only the joints of one leg
      for (int legJointIndex: legJointIds.get( this.getSideOfTheFootRoot() ))
      {
         joint_weights.set(legJointIndex, 1);
      }
 
      taskPelvisPose.setWeightsJointSpace(joint_weights);

      //--------------------------------------------------
      for (int legJointIndex: legJointIds.get( getSideOfTheFootRoot().getOppositeSide() ))
      {
         joint_weights.set(legJointIndex, 1);
      }
      
      taskLegPose.setWeightsJointSpace(joint_weights);
      
      Vector64F target_left_foot = new Vector64F(7, 0, 2 * 0.11, 0, 0, 0, 0, 1);
      taskLegPose.setTarget(target_left_foot);
               
      //--------------------------------------------------
      // Don't move the arm to keep the balance. But torso need to be activated
      joint_weights.set(back_bkz, 1);
      joint_weights.set(back_bky, 1);
      joint_weights.set(back_bkx, 1);

      taskCoM.setWeightsJointSpace(joint_weights);

      // this value shall be overwritten later
      Vector64F target_com = new Vector64F(3, 0, 0, 0);    
      taskCoM.setErrorTolerance( 0.01);
      taskCoM.setTarget(target_com);
      //--------------------------------------------------
      // control position and rotation (6 DoF) of the end effector

      for (int armJointIndex: armJointIds.get(RobotSide.LEFT)) 
         joint_weights.set(armJointIndex, 1 );

      for (int armJointIndex: armJointIds.get(RobotSide.RIGHT))
         joint_weights.set(armJointIndex, 0 );
       
      taskEndEffectorPosition.get(LEFT).setWeightsJointSpace(joint_weights);
      taskEndEffectorRotation.get(LEFT).setWeightsJointSpace(joint_weights);
      
    //-------------------
      for (int armJointIndex: armJointIds.get(RobotSide.LEFT)) 
         joint_weights.set(armJointIndex, 0 );

      for (int armJointIndex: armJointIds.get(RobotSide.RIGHT))
         joint_weights.set(armJointIndex, 1 );
      
      taskEndEffectorPosition.get(RIGHT).setWeightsJointSpace(joint_weights);
      taskEndEffectorRotation.get(RIGHT).setWeightsJointSpace(joint_weights);

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
      weights_jointpose.set(rleg_hpy, 0.5);
      coupledJointWeights.set(rleg_hpy, rleg_aky, -1);
      coupledJointWeights.set(rleg_hpy, rleg_kny, -1);

      // back
      weights_jointpose.set(back_bkz, 0.1);
      weights_jointpose.set(back_bky, 0.2);
      weights_jointpose.set(back_bkx, 0.8);

      // try keep hpx mirrored 
      weights_jointpose.set(rleg_hpx, 1);
      weights_jointpose.set(lleg_hpx, 1);
      coupledJointWeights.set(rleg_hpx, lleg_hpx, -1);
      coupledJointWeights.set(lleg_hpx, rleg_hpx, -1);

      // keep this to zero if you can
      weights_jointpose.set(rleg_hpz, 0.05);
      weights_jointpose.set(lleg_hpz, 0.05);

      // try to minimize the amount of motion of these joints (wrist pitches)
  /*    for(String jointName: jointsToMinimize)
      {
         int jointId = jointNamesToIndex.get(jointName);
         weights_jointpose.set(jointId, 0.1);
         coupledJointWeights.set(jointId, jointId, 1);
      }*/
      
   // try to reduce wild motion of wrist2
      weights_jointpose.set(l_wrist2, 0.02);
      weights_jointpose.set(r_wrist2, 0.02);
      weights_jointpose.set(l_shoulderx, 0.02);
      weights_jointpose.set(r_shoulderx, 0.02);
      
      //-------------------------------------------------
      RobotModel model = this.getHierarchicalSolver().getRobotModel(); 
      
     for (Entry<String,Integer> entry: jointNamesToIndex.entrySet())
      {
         String jointName = entry.getKey();
         int index = entry.getValue();
         if( Math.abs( weights_jointpose.get(index)) < 0.0001 )
         {
           // weights_jointpose.set(index, 0.01);
          //  parameters.setPreferedJointAngle( jointName,  0.5*(model.q_min(index)+ model.q_max(index)) );
         }
         else{
            parameters.setPreferedJointAngle( jointName,  0.0 );
         }
      }
      
      taskJointsPose.setWeightsTaskSpace(weights_jointpose);
      taskJointsPose.setWeightsJointSpace(joint_weights);
      taskJointsPose.setCoupledJointWeights(coupledJointWeights);

      taskJointsPose.setErrorTolerance( 0.9 );
   }
   
   private HashMap<String,Double> suggestedAnglesForReseed = new HashMap<String,Double>();
   
   @Override
   public HashMap<String,Double> getSuggestedAnglesForReseed()
   {
      suggestedAnglesForReseed.put("l_leg_akx", 0.0);
      suggestedAnglesForReseed.put("r_leg_akx", 0.0);
      
      suggestedAnglesForReseed.put("l_leg_aky", -0.8);
      suggestedAnglesForReseed.put("r_leg_aky", -0.8);
      
      suggestedAnglesForReseed.put("l_leg_kny", 1.6);
      suggestedAnglesForReseed.put("r_leg_kny", 1.6);
      
      suggestedAnglesForReseed.put("l_leg_hpy", -0.8);
      suggestedAnglesForReseed.put("r_leg_hpy", -0.8);
      
      suggestedAnglesForReseed.put("l_leg_hpx", 0.0);
      suggestedAnglesForReseed.put("r_leg_hpx", 0.0);

      suggestedAnglesForReseed.put("l_leg_hpz", 0.0);
      suggestedAnglesForReseed.put("r_leg_hpz", 0.0);
      
      suggestedAnglesForReseed.put("l_arm_elx", 1.0);
      suggestedAnglesForReseed.put("r_arm_elx", -1.0);

      return suggestedAnglesForReseed;
   }
}
