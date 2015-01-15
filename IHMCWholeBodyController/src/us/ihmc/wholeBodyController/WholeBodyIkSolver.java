package us.ihmc.wholeBodyController;

import java.io.File;
import java.io.IOException;
import java.io.InputStream;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.nio.file.StandardCopyOption;
import java.util.HashMap;
import java.util.HashSet;

import javax.vecmath.Matrix3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;
import javax.xml.parsers.ParserConfigurationException;

import org.apache.commons.lang.mutable.MutableBoolean;
import org.ejml.data.DenseMatrix64F;
import org.xml.sax.SAXException;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.utilities.hierarchicalKinematics.ForwardKinematicSolver;
import us.ihmc.utilities.hierarchicalKinematics.HierarchicalKinematicSolver;
import us.ihmc.utilities.hierarchicalKinematics.HierarchicalTask_BodyOrientation;
import us.ihmc.utilities.hierarchicalKinematics.HierarchicalTask_BodyPose;
import us.ihmc.utilities.hierarchicalKinematics.HierarchicalTask_BodyPosition;
import us.ihmc.utilities.hierarchicalKinematics.HierarchicalTask_COM;
import us.ihmc.utilities.hierarchicalKinematics.HierarchicalTask_JointsPose;
import us.ihmc.utilities.hierarchicalKinematics.RobotModel;
import us.ihmc.utilities.hierarchicalKinematics.VectorXd;
import us.ihmc.utilities.humanoidRobot.frames.ReferenceFrames;
import us.ihmc.utilities.math.Vector64F;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.math.geometry.RigidBodyTransform;
import us.ihmc.utilities.nativelibraries.NativeLibraryLoader;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.utilities.robotSide.SideDependentList;
import us.ihmc.utilities.screwTheory.InverseDynamicsJointStateCopier;
import us.ihmc.utilities.screwTheory.OneDoFJoint;
import us.ihmc.utilities.screwTheory.SixDoFJoint;
import us.ihmc.wholeBodyController.WholeBodyControllerParameters;

public class WholeBodyIkSolver
{

   ///--------------- Constants -----------------------------------------
   static public final double IGNORE = -66666.666;

   static public final SideDependentList<String> endEffectorLinkNames = new SideDependentList<String>();
   {
      endEffectorLinkNames.put(LEFT, "l_ee_link");
      endEffectorLinkNames.put(RIGHT, "r_ee_link");
   }

   static public final SideDependentList<String> footLinkNames = new SideDependentList<String>();
   {
      footLinkNames.put(LEFT, "l_foot");
      footLinkNames.put(RIGHT, "r_foot");
   }

   static public enum ControlledDoF { DOF_NONE, DOF_3P, DOF_3P2R, DOF_3P3R }
   
   static public enum ComputeOption { RESEED, USE_ACTUAL_MODEL_JOINTS, USE_JOINTS_CACHE };

   static public final int numDoFperArm = 6;
   static public final int numDoFperLeg = 6;
   static public final RobotSide footRootSide = RobotSide.RIGHT;

   // I just like shorter names :/
   final private static RobotSide RIGHT = RobotSide.RIGHT;
   final private static RobotSide LEFT = RobotSide.LEFT;

   ///--------------- Reference Frames and Transfoms --------------------------
   private final ReferenceFrames workingFrames;
   private final ReferenceFrames actualFrames;

   private final ReferenceFrame  workingPelvisFrame;
   private final ReferenceFrame  workingRootFrame;

   private final SideDependentList<ReferenceFrame> actualSoleFrames = new SideDependentList<ReferenceFrame>();
   private final SideDependentList<ReferenceFrame> workingSoleFrames = new SideDependentList<ReferenceFrame>();

   //---------- robot models in different contexts and different formats ----------
   private final RobotModel        urdfModel;
   private final SDFFullRobotModel actual_sdf_model;
   private final SDFFullRobotModel working_sdf_model;
   private final HierarchicalKinematicSolver wb_solver;

   //---------------  tasks assigned to this solver --------------------------------
   public HierarchicalTask_COM        task_com_position;
   public HierarchicalTask_JointsPose task_joints_pose;
   public final SideDependentList<HierarchicalTask_BodyOrientation> task_end_effector_rotations = new SideDependentList<HierarchicalTask_BodyOrientation>();
   public final SideDependentList<HierarchicalTask_BodyPosition> task_end_effector_translations = new SideDependentList<HierarchicalTask_BodyPosition>();
   public final HierarchicalTask_BodyPose task_leg_pose;

   // ------------- Parameters that change the overall behavior ----------------------

   private final SideDependentList<MutableBoolean> keepArmQuiet = new SideDependentList<MutableBoolean>();
   private final SideDependentList<MutableBoolean> disabledtHandRotationY = new SideDependentList<MutableBoolean>();
   private boolean lockLegs = false;
   private boolean suggestKneeAngle = true;

   // --------------------- Miscellaneous -------------------------------------
   private final HashMap<String, Integer> joint_names_map = new HashMap<String, Integer>();
   private final OneDoFJoint[] working_joints_in_urdf_order;
   private final SideDependentList<int[]> armJointIds = new SideDependentList<int[]>();
   private final SideDependentList<int[]> legJointIds = new SideDependentList<int[]>();

   private final DenseMatrix64F couple_j_weights;
   private final Vector64F      prefered_joint_pose;

   private final InverseDynamicsJointStateCopier actualModelJointCopier;
   private final InverseDynamicsJointStateCopier desiredModelJointCopier;

   private final SixDoFJoint  workingRootJoint;
   private final Vector64F q_init;

   final private int numOfJoints;
   final private HashSet<String> listOfVisibleAndEnableColliders = new HashSet<String>();

   //---------------------------------------------------------------------------------

   public void activateAutomaticPreferredHipHeight(boolean enable)
   {
      suggestKneeAngle = enable;
   }
   
   public void setVerbose(boolean verbose)
   {
      wb_solver.setVerbose(verbose);
   }

   public RobotModel getUrdfRobotModel()
   {
      return urdfModel;
   }
   
   /* Note. The after/before direction is reversed between the SDF and HIK models for the right leg.
    * This method uses the direction specified by the WB (reversed).
    */
   public int getIdOfBodyAfterJoint(String jointname)
   {
      int jointId = urdfModel.getJointIndexByName(jointname);
      return  getIdOfBodyAfterJoint(jointId);
   }
   
   /* Note. The after/before direction is reversed between the SDF and HIK models for the right leg.
    * This method uses the direction specified by the WB (reversed).
    */
   public int getIdOfBodyAfterJoint(int jointId)
   {
      return  urdfModel.getChildBodyOfJoint(jointId);
   }
   
   public RigidBodyTransform getLocalBodyTransform(String bodyname )
   {
      int bodyid = getUrdfRobotModel().getBodyId( bodyname );
      return getLocalBodyTransform(bodyid);
   }

   public RigidBodyTransform getLocalBodyTransform(int bodyid )
   {
      RigidBodyTransform out = new RigidBodyTransform();
      VectorXd body_rot = new VectorXd(4);
      VectorXd body_pos = new VectorXd(3);
      getHierarchicalSolver().getForwardSolver().getBodyPose( bodyid, body_rot, body_pos);
      
      Quat4d quat = new Quat4d( body_rot.get(0), body_rot.get(1), body_rot.get(2), body_rot.get(3) );
      quat.inverse();

      out.setRotation( quat );
      out.setTranslation( new Vector3d( body_pos.get(0), body_pos.get(1), body_pos.get(2) ) ); 
      return out;
   }
   
   public ReferenceFrame getRootFrame()
   {
      return workingRootFrame;
   }
   
   public ReferenceFrame getBodyReferenceFrame(String bodyname)
   {
      RigidBodyTransform localTransform = getLocalBodyTransform( bodyname );
      ReferenceFrame bodyFrame =  ReferenceFrame.constructFrameWithUnchangingTransformToParent("", workingRootFrame, localTransform);
      System.out.println( localTransform );
      return bodyFrame;
   }
   
   public ReferenceFrame getBodyReferenceFrame(int bodyId)
   {
      RigidBodyTransform localTransform = getLocalBodyTransform( bodyId );
      ReferenceFrame bodyFrame =  ReferenceFrame.constructFrameWithUnchangingTransformToParent("", workingRootFrame, localTransform);
      return bodyFrame;
   }   

   public HierarchicalKinematicSolver getHierarchicalSolver()
   {
      return wb_solver;
   }

   public int getNumberOfJoints()
   {
      return urdfModel.getNrOfJoints();
   }
   

   public WholeBodyIkSolver(WholeBodyControllerParameters robot_model, SDFFullRobotModel actualRobotModel) throws IOException
   {
      // load external library
      NativeLibraryLoader.loadLibrary("us.ihmc.utilities.hierarchicalKinematics", "hik_java");
      NativeLibraryLoader.loadLibrary("us.ihmc.convexOptimization", "qpOASESSwig_rel");

      actual_sdf_model   = actualRobotModel;
      working_sdf_model  = robot_model.createFullRobotModel();
      workingFrames = new ReferenceFrames(working_sdf_model);
      actualFrames  = new ReferenceFrames(actual_sdf_model);

      workingPelvisFrame = workingFrames.getPelvisFrame();
      workingRootJoint = working_sdf_model.getRootJoint();


      actualModelJointCopier = new InverseDynamicsJointStateCopier(actual_sdf_model.getElevator(), working_sdf_model.getElevator());
      desiredModelJointCopier = new InverseDynamicsJointStateCopier(working_sdf_model.getElevator(), working_sdf_model.getElevator());

      //----------- STEP A: find, copy and load the urdf file --------------------------------

      String urdf_filename = "models/atlas_v4_robotiq.wb.urdf";
      
      /// load the file. But first copy it on $HOME/.ihmc
      InputStream resource = getClass().getClassLoader().getResourceAsStream(urdf_filename);
      Files.createDirectories(Paths.get(System.getProperty("user.home"), ".ihmc", "models"));
      String[] string_parts = urdf_filename.split("/");
      Path modelLocationPath = Paths.get(System.getProperty("user.home"), ".ihmc", "models", string_parts[string_parts.length - 1]);
      String modelLocationPathString = modelLocationPath.toAbsolutePath().toString();

      if (resource != null)
      {
         Files.copy(resource, modelLocationPath, StandardCopyOption.REPLACE_EXISTING);
      }
      else
      {
         throw new IOException("urdf not found in resources/" + urdf_filename);
      }

      // create a RobotModel using a special URDF
      urdfModel = new RobotModel();
      urdfModel.loadURDF(modelLocationPathString, false);

      numOfJoints = urdfModel.getNrOfJoints();

      working_joints_in_urdf_order = new OneDoFJoint[numOfJoints];

      wb_solver = new HierarchicalKinematicSolver(urdfModel, 40, 0.0001);
      wb_solver.setVerbose(false);

      q_init = new Vector64F(numOfJoints);

      for (RobotSide robotSide : RobotSide.values())
      {
         actualSoleFrames.set(robotSide, actualFrames.getSoleFrame(robotSide));
         workingSoleFrames.set(robotSide, workingFrames.getSoleFrame(robotSide));
         armJointIds.set(robotSide, new int[numDoFperArm]);
         legJointIds.set(robotSide, new int[numDoFperLeg]);
         disabledtHandRotationY.set(robotSide, new MutableBoolean(false));
         keepArmQuiet.set(robotSide, new MutableBoolean(true));
      }
      // this is After and not before because the direction in URDF is inverted
      workingRootFrame = actual_sdf_model.getOneDoFJointByName("r_leg_akx").getFrameAfterJoint();

      for (int i = 0; i < numOfJoints; i++)
      {
         String joint_name_in_urdf = urdfModel.getActiveJointName(i);
         joint_names_map.put(joint_name_in_urdf, i);
         OneDoFJoint joint = working_sdf_model.getOneDoFJointByName(joint_name_in_urdf);
         if (joint != null)
         {
            working_joints_in_urdf_order[i] = joint;
         }
         else
         {
            System.err.println("WholeBodyIkSolver: Joint " + joint_name_in_urdf + " in urdf not found in sdf");
         }
      }

      ///----- Step B: allocate all the tasks and add them to the solver --------------

      ForwardKinematicSolver fk = wb_solver.getForwardSolver();

      task_com_position = new HierarchicalTask_COM(fk);

      for (RobotSide robotSide : RobotSide.values())
      {
         int currentHandId = urdfModel.getBodyId(endEffectorLinkNames.get(robotSide));

         task_end_effector_translations.set(robotSide, new HierarchicalTask_BodyPosition(fk, currentHandId));
         task_end_effector_rotations.set(robotSide, new HierarchicalTask_BodyOrientation(fk, currentHandId));
         urdfModel.getParentJoints(currentHandId, numDoFperArm, armJointIds.get(robotSide));
      }     

      int leftFootId = urdfModel.getBodyId(footLinkNames.get(LEFT));
      task_leg_pose = new HierarchicalTask_BodyPose(fk, leftFootId);

      urdfModel.getParentJoints(leftFootId, numDoFperLeg, legJointIds.get(LEFT) );

      for (int i = 0; i < numDoFperLeg; i++)
      {
         legJointIds.get(RIGHT)[i] = i;
      }

      task_joints_pose = new HierarchicalTask_JointsPose(fk);

      // the order is important!! first to be added have higher priority
      wb_solver.addTask(task_leg_pose);
      wb_solver.addTask(task_com_position);
      wb_solver.addTask(task_end_effector_translations.get(RIGHT));
      wb_solver.addTask(task_end_effector_rotations.get(RIGHT));
      wb_solver.addTask(task_end_effector_translations.get(LEFT));
      wb_solver.addTask(task_end_effector_rotations.get(LEFT));
      wb_solver.addTask(task_joints_pose);

      prefered_joint_pose = new Vector64F(numOfJoints);
      prefered_joint_pose.zero();
      couple_j_weights = new DenseMatrix64F(numOfJoints, numOfJoints);

      //code moved there for esthetic reasons :|
      configureTasks();

      try
      {
         wb_solver.collisionAvoidance.loadURDF(new File( modelLocationPathString ));
         configureCollisionAvoidance();
      }
      catch (ParserConfigurationException | SAXException e)
      {
         e.printStackTrace();
      }
   }

   public boolean isBodyIncludedInCollision(String bodyName)
   {
      return listOfVisibleAndEnableColliders.contains(bodyName);
   }

   private void configureCollisionAvoidance()
   {
      listOfVisibleAndEnableColliders.add("pelvis");
      listOfVisibleAndEnableColliders.add("utorso");
      
      listOfVisibleAndEnableColliders.add("r_larm");
      listOfVisibleAndEnableColliders.add("r_uleg");
      listOfVisibleAndEnableColliders.add("l_larm");
      listOfVisibleAndEnableColliders.add("l_uleg");
      
      wb_solver.collisionAvoidance.addBodyPair("r_larm", "l_larm");
      
      wb_solver.collisionAvoidance.addBodyPair("r_larm", "r_uleg" );
      wb_solver.collisionAvoidance.addBodyPair("r_larm", "l_uleg" );
      wb_solver.collisionAvoidance.addBodyPair("r_larm", "pelvis");
      wb_solver.collisionAvoidance.addBodyPair("r_larm", "utorso" );
     
      wb_solver.collisionAvoidance.addBodyPair("r_larm", "r_uleg" );
      wb_solver.collisionAvoidance.addBodyPair("l_larm", "l_uleg" );
      wb_solver.collisionAvoidance.addBodyPair("l_larm", "pelvis");
      wb_solver.collisionAvoidance.addBodyPair("l_larm", "utorso" );
   }

   private void configureTasks()
   {
      HierarchicalTask_BodyPosition task_ee_pose_R_pos = task_end_effector_translations.get(RIGHT);
      HierarchicalTask_BodyPosition task_ee_pose_L_pos = task_end_effector_translations.get(LEFT);
      HierarchicalTask_BodyOrientation task_ee_pose_R_rot = task_end_effector_rotations.get(RIGHT);
      HierarchicalTask_BodyOrientation task_ee_pose_L_rot = task_end_effector_rotations.get(LEFT);

      int rleg_akx = joint_names_map.get("r_leg_akx");
      int rleg_aky = joint_names_map.get("r_leg_aky");
      int rleg_kny = joint_names_map.get("r_leg_kny");
      int rleg_hpy = joint_names_map.get("r_leg_hpy");
      int rleg_hpx = joint_names_map.get("r_leg_hpx");
      int rleg_hpz = joint_names_map.get("r_leg_hpz");

      int lleg_akx = joint_names_map.get("l_leg_akx");
      int lleg_aky = joint_names_map.get("l_leg_aky");
      int lleg_kny = joint_names_map.get("l_leg_kny");
      int lleg_hpy = joint_names_map.get("l_leg_hpy");
      int lleg_hpx = joint_names_map.get("l_leg_hpx");
      int lleg_hpz = joint_names_map.get("l_leg_hpz");

      int back_bkz = joint_names_map.get("back_bkz");
      int back_bky = joint_names_map.get("back_bky");
      int back_bkx = joint_names_map.get("back_bkx");

      int larm_elx = joint_names_map.get("l_arm_elx");
      int rarm_elx = joint_names_map.get("r_arm_elx");

      task_ee_pose_L_pos.setMaximumError(0.2);
      task_ee_pose_L_rot.setMaximumError(0.4);

      task_ee_pose_R_pos.setMaximumError(0.2);
      task_ee_pose_R_rot.setMaximumError(0.4);
      task_joints_pose.setMaximumError(1.0);

      task_ee_pose_R_pos.setEnabled(false);
      task_ee_pose_R_rot.setEnabled(false);
      task_ee_pose_L_pos.setEnabled(false);
      task_ee_pose_L_rot.setEnabled(false);

      //--------------------------------------------------
      // Important: you can't activate a joint in higher priorities and disable it later
      // use this vector for any setWeightJointSpace
      Vector64F joint_weights = new Vector64F(numOfJoints);
      joint_weights.zero();

      // use only the joints of legs to achieve this task. Ignore torso and arms
      for (int i = 0; i < numDoFperLeg; i++)
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
      //task_com_position.setTargetBracket( new  Vector64F( 3,  0.01, 0.03,0 ) );

      // Don't move the arm to keep the balance. But torso need to be activated
      joint_weights.set(back_bkz, 1);
      joint_weights.set(back_bky, 1);
      joint_weights.set(back_bkx, 1);

      task_com_position.setWeightsJointSpace(joint_weights);

      // TODO make this the center of the feet
      Vector64F target_com = new Vector64F(3, 0, 0.11, 0);
      task_com_position.setTarget(target_com, 0.005);
      //--------------------------------------------------
      // control position and rotation (6 DoF) of the end effector
      Vector64F weights_ee_pos = new Vector64F(3, 1, 1, 1);
      Vector64F weights_ee_rot = new Vector64F(3, 1, 1, 1);

      task_ee_pose_L_pos.setWeightsTaskSpace(weights_ee_pos);
      task_ee_pose_L_rot.setWeightsTaskSpace(weights_ee_rot);
      task_ee_pose_R_pos.setWeightsTaskSpace(weights_ee_pos);
      task_ee_pose_R_rot.setWeightsTaskSpace(weights_ee_rot);

      for (int i = 0; i < numDoFperArm; i++)
      {
         int index = armJointIds.get(RobotSide.LEFT)[i];
         joint_weights.set(index, 1);
      }
      task_ee_pose_L_pos.setWeightsJointSpace(joint_weights);
      task_ee_pose_L_rot.setWeightsJointSpace(joint_weights);

      for (int i = 0; i < numDoFperArm; i++)
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
      Vector64F weights_jointpose = new Vector64F(numOfJoints);
      weights_jointpose.zero();
      couple_j_weights.zero();

      // these are the joints to be knees
      weights_jointpose.set(rleg_kny, 0.5);
      weights_jointpose.set(lleg_kny, 0.5);

      // this is used to keep the pelvis straight
      // what you want is:
      //  q(r_leg_hpy) = - q(leg_aky) - q(leg_kny);
      weights_jointpose.set(rleg_hpy, 2);
      couple_j_weights.set(rleg_hpy, rleg_aky, -1);
      couple_j_weights.set(rleg_hpy, rleg_kny, -1);

      // back
      weights_jointpose.set(back_bkz, 1);
      weights_jointpose.set(back_bky, 1);
      weights_jointpose.set(back_bkx, 1);

      // try keep hpx mirrored 
      weights_jointpose.set(rleg_hpx, 1);
      weights_jointpose.set(lleg_hpx, 1);
      couple_j_weights.set(rleg_hpx, lleg_hpx, -1);
      couple_j_weights.set(lleg_hpx, rleg_hpx, -1);

      // keep this to zero if you can
      weights_jointpose.set(rleg_hpz, 0.2);
      weights_jointpose.set(lleg_hpz, 0.2);

      //elbows
      weights_jointpose.set(larm_elx, 1.0);
      weights_jointpose.set(rarm_elx, 1.0);

      prefered_joint_pose.set(larm_elx, 1.0);
      prefered_joint_pose.set(rarm_elx, -1.0);

      task_joints_pose.setWeightsTaskSpace(weights_jointpose);
      task_joints_pose.setWeightsJointSpace(joint_weights);
      task_joints_pose.setCoupledJointWeights(couple_j_weights);

      task_joints_pose.setTarget(prefered_joint_pose, 35);

   }

   private void setPreferedKneeAngle()
   {
      if( suggestKneeAngle == false)
      {
         return;
      }
      HierarchicalTask_BodyPosition taskL = task_end_effector_translations.get(LEFT);
      HierarchicalTask_BodyPosition taskR = task_end_effector_translations.get(RIGHT);
        
      double zL = taskL.getTarget().get(2);
      double zR = taskR.getTarget().get(2);
         
      if(taskL.isEnabled() && !taskR.isEnabled() ) {
         zR = zL;
      }
      else  if( !taskL.isEnabled() && taskR.isEnabled() ) {
         zL = zR;
      }
      else if( !taskL.isEnabled() && !taskR.isEnabled() ) {
         zL = 1;
         zR = 1;
      }
      
      double preferedKneeAngle = 2.4 - (zL +  zR)*0.5;
      if( preferedKneeAngle < 0.4) preferedKneeAngle = 0.4;
      
      prefered_joint_pose.set(joint_names_map.get("l_leg_kny"), preferedKneeAngle);
      prefered_joint_pose.set(joint_names_map.get("r_leg_kny"), preferedKneeAngle);
      task_joints_pose.setTarget(prefered_joint_pose, 3);
   }

   public void disableHandRotationMainAxis(RobotSide side, boolean disable)
   {
      disabledtHandRotationY.get(side).setValue(disable);
   }

   public void setHandTarget(RobotSide end_effector_side, ReferenceFrame end_effector_pose)
   {
      final HierarchicalTask_BodyPosition task_ee_trans = task_end_effector_translations.get(end_effector_side);
      final HierarchicalTask_BodyOrientation task_ee_rot = task_end_effector_rotations.get(end_effector_side);

      if (task_ee_trans.isEnabled() || task_ee_rot.isEnabled())
      {
         RigidBodyTransform ee_transform = new RigidBodyTransform();
         end_effector_pose.getTransformToDesiredFrame(ee_transform, workingRootFrame);

         keepArmQuiet.get(end_effector_side).setValue( false );

         RobotSide steadySide = end_effector_side.getOppositeSide();

         wb_solver.prioritizeTasks(task_end_effector_translations.get(steadySide), task_ee_trans);
         wb_solver.prioritizeTasks(task_end_effector_rotations.get(steadySide), task_ee_rot);

         Vector64F target_ee = new Vector64F(7);
         Quat4d quat = new Quat4d();
         Vector3d pos = new Vector3d();

         ee_transform.get(quat, pos);
         target_ee.set(0, pos.x);
         target_ee.set(1, pos.y);
         target_ee.set(2, pos.z);

         target_ee.set(3, quat.x);
         target_ee.set(4, quat.y);
         target_ee.set(5, quat.z);
         target_ee.set(6, quat.w);

         task_ee_trans.setTarget(target_ee, 0.001);
         task_ee_rot.setTarget(target_ee, 0.001);

         if (task_ee_rot.isEnabled())
         {
            if ( disabledtHandRotationY.get(end_effector_side).isTrue())
            {
               Matrix3d rot = new Matrix3d();
               rot.set(quat);
               task_ee_rot.disableAxisInTaskSpace(new Vector3d(rot.m01, rot.m11, rot.m21));
            }
            else{
               task_ee_rot.setWeightsTaskSpace(new Vector64F(3, 1, 1, 1));
            }
         }
      }
   }

   public void setPreferedJointPose(String joint_name, double q)
   {
      int index = joint_names_map.get(joint_name);
      prefered_joint_pose.set(index, q);
      task_joints_pose.setTarget(prefered_joint_pose, 3);
   }

   private void checkIfArmShallStayQuiet(Vector64F q_init)
   {
      DenseMatrix64F weights_joint = task_joints_pose.getWeightMatrixTaskSpace();

      for( RobotSide side: RobotSide.values())
      {
         for (int J = 0; J < armJointIds.get(side).length; J++)
         {
            int jointId = armJointIds.get(side)[J];
            if ( keepArmQuiet.get(side).isTrue() )
            {
               if (J <= 3)
                  weights_joint.set(jointId, 0.4);
               else
                  weights_joint.set(jointId, 0.3);
               prefered_joint_pose.set(jointId, q_init.get(jointId));
            }
            else
            {
               weights_joint.set(jointId, 0.1);
               prefered_joint_pose.set(jointId, q_init.get(jointId));
            }
         }
      }
      task_joints_pose.setWeightsTaskSpace(weights_joint);
      task_joints_pose.setTarget(prefered_joint_pose, 3);
   }

   private void checkIfLegsNeedToBeLocked()
   {
      task_com_position.setEnabled(!lockLegs);
      task_leg_pose.setEnabled(!lockLegs);
      for (RobotSide side: RobotSide.values)
      {
         Vector64F weightsRot    =  task_end_effector_rotations.get(side).getWeightsJointSpace();
         Vector64F weightsPos    =  task_end_effector_translations.get(side).getWeightsJointSpace();
         Vector64F weightsJoints =  task_joints_pose.getWeightsJointSpace();
         Vector64F weightsCollision = getHierarchicalSolver().collisionAvoidance.getJointWeights();

         double val = lockLegs ? 0 : 1;

         for (int i = 0; i < numDoFperLeg; i++)
         {
            weightsRot.set(legJointIds.get(side)[i], val);
            weightsPos.set(legJointIds.get(side)[i], val);
            weightsJoints.set(legJointIds.get(side)[i], val);
            weightsCollision.set(legJointIds.get(side)[i], val);
         }
      }
   }
   
   //ComputeOption { RESEED, USE_WORKING_MODEL_JOINTS, USE_JOINTS_CACHE };
   
   public int compute(SDFFullRobotModel robotModelToPack)
   {
      return compute( robotModelToPack, ComputeOption.USE_JOINTS_CACHE);
   }
   
   public int compute(SDFFullRobotModel robotModelToPack, ComputeOption opt)
   {
      switch(opt)
      {
      case RESEED:
         Vector64F randQ = getHierarchicalSolver().getRandomQ();
         for (int i = 0; i < numOfJoints; i++)
         {
            q_init.set(i, randQ.get(i));
         }
         
         q_init.set(joint_names_map.get("l_leg_aky"), -0.7);
         q_init.set(joint_names_map.get("r_leg_aky"), -0.7);
         q_init.set(joint_names_map.get("l_leg_kny"), 1.4);
         q_init.set(joint_names_map.get("r_leg_kny"), 1.4);
         q_init.set(joint_names_map.get("l_leg_hpy"), -0.7);
         q_init.set(joint_names_map.get("r_leg_hpy"), -0.7);
         q_init.set(joint_names_map.get("l_arm_elx"), 0.7);
         q_init.set(joint_names_map.get("r_arm_elx"), -0.7);
         
         break;
      case USE_ACTUAL_MODEL_JOINTS:
         for (int i = 0; i < numOfJoints; i++)
         {
            OneDoFJoint joint = actual_sdf_model.getOneDoFJointByName(urdfModel.getActiveJointName(i));
            q_init.set(i, joint.getQ());
         }
         break;
         
      case USE_JOINTS_CACHE:
         //do nothing, the cache is used by default.
      }
      
      int ret = computeImpl(robotModelToPack);
      
      if(opt != ComputeOption.RESEED)
      {
         keepArmQuiet.get(LEFT).setValue(true);
         keepArmQuiet.get(RIGHT).setValue(true);
      }
      return ret;
   }

   
   private int computeImpl(SDFFullRobotModel robotModelToPack)
   {
      updateDesiredSDFFullRobotModelToActual();
      workingFrames.updateFrames();

      Vector64F q_out = new Vector64F(numOfJoints);

      checkIfLegsNeedToBeLocked();
      setPreferedKneeAngle();
      adjustOtherFoot();

      checkIfArmShallStayQuiet(q_init);

      int ret = -1;
      try
      {
         ret = wb_solver.solve(q_init, q_out);
      }
      catch (Exception e)
      {
         // TODO Auto-generated catch block
         e.printStackTrace();
      }
      if (ret < 0)
      {
         System.out.println("failed\n");
      }
      else{
         // use the last succesfull only.
         q_init.set(q_out);
      }

      if (ret != -2)
      {
         for (int i = 0; i < working_joints_in_urdf_order.length; i++)
         {
            if (working_joints_in_urdf_order[i] != null)
            {
               working_joints_in_urdf_order[i].setQ(q_out.get(i));
            }
         }

         moveDesiredFeetToActualFeet(footRootSide);
         if(robotModelToPack != null){
            updatePackModelToDesired(robotModelToPack);
         }
      }

      return ret;
   }
   
   public double getDesiredJointAngle(String jointName)
   {
      int index = joint_names_map.get(jointName);
      return working_joints_in_urdf_order[index].getQ();
   }

   private void adjustOtherFoot()
   {
      RigidBodyTransform foot_other_transform = new RigidBodyTransform();
      // note before and after... it is not a typo.
      ReferenceFrame leftAnkle = actual_sdf_model.getOneDoFJointByName("l_leg_akx").getFrameAfterJoint();
      //TODO. there might be something wrong here
      leftAnkle.getTransformToDesiredFrame(foot_other_transform, workingRootFrame);

      Quat4d quat = new Quat4d();
      Vector3d pos = new Vector3d();

      foot_other_transform.get(quat, pos);
      task_leg_pose.setTarget(quat, pos, 0.0005);
   }
   
   public ReferenceFrame getDesiredAfterJointFrame(String jointName, ReferenceFrame parentFrame)
   {
      workingFrames.updateFrames();
      
      ReferenceFrame workingJointFrame = working_sdf_model.getOneDoFJointByName(jointName).getFrameAfterJoint();
      ReferenceFrame workingRootFrame  = getRootFrame();
      
      RigidBodyTransform rootToJoint  = workingJointFrame.getTransformToDesiredFrame( workingRootFrame );
      RigidBodyTransform parentToRoot = workingRootFrame.getTransformToDesiredFrame( parentFrame );
      
      RigidBodyTransform parentToBody = new RigidBodyTransform();
      parentToBody.multiply(parentToRoot, rootToJoint);
      
      return ReferenceFrame.constructBodyZUpFrameWithUnchangingTransformToParent(jointName, parentFrame, parentToBody);
   }
   
   public ReferenceFrame getDesiredPelvisFrame(ReferenceFrame parentFrame)
   {
      return getDesiredBodyFrame("pelvis", parentFrame);
   }
   
   public ReferenceFrame getDesiredHandFrame(RobotSide handSide, ReferenceFrame parentFrame)
   {
      return getDesiredBodyFrame( this.endEffectorLinkNames.get(handSide), parentFrame);
   }
   
   public ReferenceFrame getDesiredBodyFrame( String name, ReferenceFrame parentFrame)
   {
      workingFrames.updateFrames();
      
      RigidBodyTransform rootToBody = getLocalBodyTransform(name);
      RigidBodyTransform parentToRoot = new RigidBodyTransform();

      ReferenceFrame workingRootFrame = getRootFrame();
      
      workingRootFrame.getTransformToDesiredFrame(parentToRoot, parentFrame);
      
      RigidBodyTransform parentToBody = new RigidBodyTransform();
      parentToBody.multiply(parentToRoot, rootToBody);
      
      return ReferenceFrame.constructBodyZUpFrameWithUnchangingTransformToParent(name, parentFrame, parentToBody);
   }

   private void moveDesiredFeetToActualFeet(RobotSide footToTrust)
   {
      workingFrames.updateFrames();
      
      ReferenceFrame actualSoleFrame = actualSoleFrames.get(footToTrust);
      ReferenceFrame workingSoleFrame = workingSoleFrames.get(footToTrust);

      RigidBodyTransform soleWorldToActual = new RigidBodyTransform();
      RigidBodyTransform soleToPelvis      = new RigidBodyTransform();
      RigidBodyTransform pelvisTransform   = new RigidBodyTransform();
      
      workingPelvisFrame.getTransformToDesiredFrame(soleToPelvis,  workingSoleFrame);
      actualSoleFrame.getTransformToDesiredFrame(soleWorldToActual, ReferenceFrame.getWorldFrame() );
      pelvisTransform.multiply( soleWorldToActual, soleToPelvis );
      
      workingRootJoint.setPositionAndRotation(pelvisTransform);
      workingFrames.updateFrames();
   }

   private void updateDesiredSDFFullRobotModelToActual()
   {
      synchronized (actual_sdf_model)
      {
         actualModelJointCopier.copy();
      }
   }

   private void updatePackModelToDesired(SDFFullRobotModel sdfFullRobotModelToPack)
   {
      desiredModelJointCopier.setRigidBodies(working_sdf_model.getElevator(), sdfFullRobotModelToPack.getElevator());
      desiredModelJointCopier.copy();
   }

   public void setNumberOfControlledDoF(RobotSide side, ControlledDoF dof)
   {
      switch( dof )
      {
      case DOF_NONE:
         task_end_effector_translations.get(side).setEnabled(false);
         task_end_effector_rotations.get(side).setEnabled(false);
         break;
      case DOF_3P:
         task_end_effector_translations.get(side).setEnabled(true);
         task_end_effector_rotations.get(side).setEnabled(false);
         break;
      case DOF_3P2R:
         task_end_effector_translations.get(side).setEnabled(true);
         task_end_effector_rotations.get(side).setEnabled(true);
         disableHandRotationMainAxis(side, true);
         break;
      case DOF_3P3R:
         task_end_effector_translations.get(side).setEnabled(true);
         task_end_effector_rotations.get(side).setEnabled(true);
         disableHandRotationMainAxis(side, false);
         break;   
      }

   }

   public boolean isLowerBodyLocked()
   {
      return lockLegs;
   }

   public void lockLowerBody(boolean lockLowerBody)
   {
      this.lockLegs = lockLowerBody;
   }


}
