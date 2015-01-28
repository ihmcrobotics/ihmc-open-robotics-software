package us.ihmc.wholeBodyController;

import java.io.IOException;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;

import javax.vecmath.Matrix3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import org.apache.commons.lang.mutable.MutableBoolean;
import org.ejml.data.DenseMatrix64F;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.utilities.LogTools;
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
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.PoseReferenceFrame;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.math.geometry.RigidBodyTransform;
import us.ihmc.utilities.nativelibraries.NativeLibraryLoader;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.utilities.robotSide.SideDependentList;
import us.ihmc.utilities.screwTheory.InverseDynamicsJointStateCopier;
import us.ihmc.utilities.screwTheory.OneDoFJoint;

abstract public class WholeBodyIkSolver
{
   static{
      NativeLibraryLoader.loadLibrary("us.ihmc.utilities.hierarchicalKinematics", "hik_java");
      NativeLibraryLoader.loadLibrary("us.ihmc.convexOptimization", "qpOASESSwig_rel");
   }

   ///--------------- Constants -----------------------------------------
   static public final double IGNORE = -66666.666;


   static public enum ControlledDoF { DOF_NONE, DOF_3P, DOF_3P2R, DOF_3P3R }

   static public enum ComputeOption { RESEED, USE_ACTUAL_MODEL_JOINTS, USE_JOINTS_CACHE };

   // I just like shorter names :/
   final private static RobotSide RIGHT = RobotSide.RIGHT;
   final private static RobotSide LEFT  = RobotSide.LEFT;

   ///--------------- Reference Frames and Transfoms --------------------------
   private final ReferenceFrames workingFrames;


   //  private final SideDependentList<ReferenceFrame> actualSoleFrames = new SideDependentList<ReferenceFrame>();
   private final SideDependentList<ReferenceFrame> workingSoleFrames = new SideDependentList<ReferenceFrame>();

   //---------- robot models in different contexts and different formats ----------
   private final RobotModel        urdfModel;
   private final SDFFullRobotModel workingSdfModel;
   protected final HierarchicalKinematicSolver hierarchicalSolver;

   //---------------  tasks assigned to this solver --------------------------------
   public HierarchicalTask_COM        taskComPosition;
   public HierarchicalTask_JointsPose taskJointsPose;
   public final SideDependentList<HierarchicalTask_BodyPose> taskEndEffectorPose = new SideDependentList<HierarchicalTask_BodyPose>();
   public final HierarchicalTask_BodyPose taskLegPose;

   // ------------- Parameters that change the overall behavior ----------------------

   private final SideDependentList<MutableBoolean> keepArmQuiet = new SideDependentList<MutableBoolean>();
   private final SideDependentList<ControlledDoF>  controlledDoF = new SideDependentList<ControlledDoF>();
   private boolean lockLegs = false;
   private boolean suggestKneeAngle = true;

   // --------------------- Miscellaneous -------------------------------------
   protected final HashMap<String, Integer> jointNamesToIndex = new HashMap<String, Integer>();
   protected final OneDoFJoint[] workingJointsInUrdfOrder;
   protected final SideDependentList<int[]> armJointIds = new SideDependentList<int[]>();
   protected final SideDependentList<int[]> legJointIds = new SideDependentList<int[]>();

   protected final DenseMatrix64F coupledJointWeights;
   protected final Vector64F      preferedJointPose;

   private final Vector64F q_init;

   final private int numOfJoints;
   private HashSet<String> listOfVisibleAndEnableColliders;

   //------- Abstract method that provide robot-specific information and configuration  -------------

   abstract public String getEndEffectorLinkName(RobotSide side);
   abstract public String getFootLinkName(RobotSide side);

   abstract public int getNumberDoFperArm();
   abstract public int getNumberDoFperLeg();
   abstract public RobotSide getSideOfTheFootRoot();
   abstract public String getURDFConfigurationFileName()  throws IOException ;
   abstract protected HashSet<String> configureCollisionAvoidance(String urdfFileName )  throws Exception;
   abstract protected void configureTasks();

   abstract public ReferenceFrame  getRootFrame( SDFFullRobotModel model);

   //---------------------------------------------------------------------------------

   public void activateAutomaticPreferredHipHeight(boolean enable)
   {
      suggestKneeAngle = enable;
   }

   public ReferenceFrame getRootFrame(){
      return getRootFrame(workingSdfModel);
   }

   public boolean hasJoint(String jointName)
   {
      return (jointNamesToIndex.get(jointName) != null);
   }

   public void setVerbose(boolean verbose)
   {
      hierarchicalSolver.setVerbose(verbose);
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


   public ReferenceFrame getBodyReferenceFrame(String bodyname)
   {
      RigidBodyTransform localTransform = getLocalBodyTransform( bodyname );
      ReferenceFrame bodyFrame =  ReferenceFrame.constructFrameWithUnchangingTransformToParent("", 
            getRootFrame(workingSdfModel), localTransform);
      System.out.println( localTransform );
      return bodyFrame;
   }

   public ReferenceFrame getBodyReferenceFrame(int bodyId)
   {
      RigidBodyTransform localTransform = getLocalBodyTransform( bodyId );
      ReferenceFrame bodyFrame =  ReferenceFrame.constructFrameWithUnchangingTransformToParent("", 
            getRootFrame(workingSdfModel), localTransform);
      return bodyFrame;
   }   


   public HierarchicalKinematicSolver getHierarchicalSolver()
   {
      return hierarchicalSolver;
   }

   public int getNumberOfJoints()
   {
      return urdfModel.getNrOfJoints();
   }

   static 
   {
      NativeLibraryLoader.loadLibrary("us.ihmc.utilities.hierarchicalKinematics", "hik_java");
      NativeLibraryLoader.loadLibrary("us.ihmc.convexOptimization", "qpOASESSwig_rel");
   }

   public WholeBodyIkSolver( WholeBodyControllerParameters robotControllerParameters ) 
   {
      // load external library
      workingSdfModel  = robotControllerParameters.createFullRobotModel();
      workingFrames = new ReferenceFrames(workingSdfModel);

      //----------- STEP A: find, copy and load the urdf file -------------------------------- 

      // create a RobotModel using a special URDF
      urdfModel = new RobotModel();

      try{
         urdfModel.loadURDF( getURDFConfigurationFileName() , false);
      }
      catch(Exception e)
      {
         e.printStackTrace();
      }

      numOfJoints = urdfModel.getNrOfJoints();

      workingJointsInUrdfOrder = new OneDoFJoint[numOfJoints];

      double EpsilonInRadiansForVerySmallAngles = 0.0001;
      int NumberOfIterations = 40;

      hierarchicalSolver = new HierarchicalKinematicSolver(urdfModel, NumberOfIterations, EpsilonInRadiansForVerySmallAngles);
      hierarchicalSolver.setVerbose(false);

      q_init = new Vector64F(numOfJoints);

      for (RobotSide robotSide : RobotSide.values())
      {
         workingSoleFrames.set(robotSide, workingFrames.getSoleFrame(robotSide));
         armJointIds.set(robotSide, new int[ getNumberDoFperArm() ]);
         legJointIds.set(robotSide, new int[ getNumberDoFperLeg() ]);
         controlledDoF.set(robotSide, ControlledDoF.DOF_NONE );
         keepArmQuiet.set(robotSide, new MutableBoolean(true));
      }

      for (int i = 0; i < numOfJoints; i++)
      {
         String joint_name_in_urdf = urdfModel.getActiveJointName(i);
         jointNamesToIndex.put(joint_name_in_urdf, i);
         OneDoFJoint joint = workingSdfModel.getOneDoFJointByName(joint_name_in_urdf);
         if (joint != null)
         {
            workingJointsInUrdfOrder[i] = joint;
         }
         else
         {
            System.err.println("WholeBodyIkSolver: Joint " + joint_name_in_urdf + " in urdf not found in sdf");
         }
      }

      ///----- Step B: allocate all the tasks and add them to the solver --------------

      ForwardKinematicSolver fk = hierarchicalSolver.getForwardSolver();

      taskComPosition = new HierarchicalTask_COM(fk);

      for (RobotSide robotSide : RobotSide.values())
      {
         int currentHandId = urdfModel.getBodyId(getEndEffectorLinkName(robotSide));

         taskEndEffectorPose.set(robotSide, new HierarchicalTask_BodyPose(fk, currentHandId));
         urdfModel.getParentJoints(currentHandId, getNumberDoFperArm() , armJointIds.get(robotSide));
      }     

      RobotSide otherFootSide = getSideOfTheFootRoot().getOppositeSide(); 
      int leftFootId = urdfModel.getBodyId(  getFootLinkName(otherFootSide ) );
      taskLegPose = new HierarchicalTask_BodyPose(fk, leftFootId);

      urdfModel.getParentJoints(leftFootId, getNumberDoFperLeg(), legJointIds.get( otherFootSide ) );

      for (int i = 0; i < getNumberDoFperLeg(); i++)
      {
         legJointIds.get( getSideOfTheFootRoot() )[i] = i;
      }

      taskJointsPose = new HierarchicalTask_JointsPose(fk);

      // the order is important!! first to be added have higher priority
      hierarchicalSolver.addTask(taskLegPose);
      hierarchicalSolver.addTask(taskComPosition);
      hierarchicalSolver.addTask(taskEndEffectorPose.get(RIGHT));
      hierarchicalSolver.addTask(taskEndEffectorPose.get(LEFT));
      hierarchicalSolver.addTask(taskJointsPose);

      preferedJointPose = new Vector64F(numOfJoints);
      preferedJointPose.zero();
      coupledJointWeights = new DenseMatrix64F(numOfJoints, numOfJoints);

      //code moved there for esthetic reasons :|
      configureTasks();
      try{
         listOfVisibleAndEnableColliders = configureCollisionAvoidance( getURDFConfigurationFileName() );
      }
      catch(Exception e)
      {
         e.printStackTrace();
      }
   }

   public boolean isBodyIncludedInCollision(String bodyName)
   {
      return listOfVisibleAndEnableColliders.contains(bodyName);
   }



   // TODO: this is robot specific ! Move it to AtlasWholeBodyIk
   private void setPreferedKneeAngle()
   {
      if( suggestKneeAngle == false)
      {
         return;
      }
      HierarchicalTask_BodyPose taskL = taskEndEffectorPose.get(LEFT);
      HierarchicalTask_BodyPose taskR = taskEndEffectorPose.get(RIGHT);

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

      preferedJointPose.set(jointNamesToIndex.get("l_leg_kny"), preferedKneeAngle);
      preferedJointPose.set(jointNamesToIndex.get("r_leg_kny"), preferedKneeAngle);
      taskJointsPose.setTarget(preferedJointPose, 3);
   }

   /* public void disableHandRotationMainAxis(RobotSide side, boolean disable)
   {
      disabledtHandRotationY.get(side).setValue(disable);
   }*/

   public void setHandTarget(SDFFullRobotModel actualRobotModel, RobotSide end_effector_side, ReferenceFrame end_effector_pose)
   {
      movePelvisToHaveOverlappingFeet( actualRobotModel, workingSdfModel );

      final HierarchicalTask_BodyPose handTask = taskEndEffectorPose.get(end_effector_side);

      RigidBodyTransform ee_transform;
      ee_transform = end_effector_pose.getTransformToDesiredFrame( getRootFrame( actualRobotModel ));

      keepArmQuiet.get(end_effector_side).setValue( false );

      RobotSide steadySide = end_effector_side.getOppositeSide();

      hierarchicalSolver.prioritizeTasks(taskEndEffectorPose.get(steadySide), handTask);

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

      handTask.setTarget(target_ee, 0.001); 
   }

   private void enforceControlledDoF( ComputeOption opt  )
   {
      double rotationWeight = 0.5;

      for ( RobotSide side: RobotSide.values)
      {
         HierarchicalTask_BodyPose handTask = taskEndEffectorPose.get(side);

         switch( controlledDoF.get(side) )
         {
         case DOF_NONE:
            handTask.setEnabled(false);
            break;
            
         case DOF_3P:
            handTask.setEnabled(true);

            if( opt != ComputeOption.RESEED)
               handTask.setWeightsTaskSpace( new Vector64F(6, 1, 1, 1, 0.05, 0.05, 0.05) );
            else
               handTask.setWeightsTaskSpace( new Vector64F(6, 1, 1, 1, 0.0, 0.0, 0.0) );

            handTask.setWeightError( new Vector64F(6, 1, 1, 1,  0,0,0) );
            break;

         case DOF_3P2R:
            handTask.setEnabled(true);
            handTask.setWeightError( new Vector64F(6, 1,1,1,  1,1,1 ) );
            
            Vector64F target = handTask.getTarget();
            Matrix3d rot = new Matrix3d();
            rot.set( new Quat4d( target.get(3),  target.get(4), target.get(5), target.get(6) ) );
            handTask.disableAxisInTaskSpace(rotationWeight, new Vector3d(rot.m01, rot.m11, rot.m21)); 
            
            break;
         case DOF_3P3R:
            handTask.setEnabled(true);
            handTask.setWeightError( new Vector64F(6, 1,1,1,  1,1,1 ) );
            handTask.setWeightsTaskSpace( new Vector64F(6, 1, 1, 1, rotationWeight, rotationWeight, rotationWeight) ); 
            break;   
         }
      }
   }

   public void setHandTarget(SDFFullRobotModel actualRobotModel, RobotSide end_effector_side, FramePose end_effector_pose)
   {
      PoseReferenceFrame endEffectorPoseReferenceFrame = new PoseReferenceFrame("endEffectorPoseReferenceFrame", end_effector_pose);
      setHandTarget(actualRobotModel, end_effector_side, endEffectorPoseReferenceFrame);
   }

   public void setPreferedJointPose(String joint_name, double q)
   {
      int index = jointNamesToIndex.get(joint_name);
      preferedJointPose.set(index, q);
      taskJointsPose.setTarget(preferedJointPose, 3);
   }

   private void checkIfArmShallStayQuiet(Vector64F q_init)
   {
      DenseMatrix64F weights_joint = taskJointsPose.getWeightMatrixTaskSpace();

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
               preferedJointPose.set(jointId, q_init.get(jointId));
            }
            else
            {
               weights_joint.set(jointId, 0.1);
               preferedJointPose.set(jointId, q_init.get(jointId));
            }
         }
      }
      taskJointsPose.setWeightsTaskSpace(weights_joint);
      taskJointsPose.setTarget(preferedJointPose, 3);
   }

   private void checkIfLegsNeedToBeLocked()
   {
      taskComPosition.setEnabled(!lockLegs);
      taskLegPose.setEnabled(!lockLegs);

      for (RobotSide side: RobotSide.values)
      {
         Vector64F weightsEE        =  taskEndEffectorPose.get(side).getWeightsJointSpace();
         Vector64F weightsJoints    =  taskJointsPose.getWeightsJointSpace();
         Vector64F weightsCollision = getHierarchicalSolver().collisionAvoidance.getJointWeights();

         double val = lockLegs ? 0 : 1;

         for (int i = 0; i < getNumberDoFperLeg(); i++)
         {
            weightsEE.set(legJointIds.get(side)[i], val);
            weightsJoints.set(legJointIds.get(side)[i], val);
            weightsCollision.set(legJointIds.get(side)[i], val);
         }
      }
   }


   public int compute(SDFFullRobotModel actualSdfModel,  Map<String, Double> anglesToUseAsInitialState,  Map<String, Double> outputAngles) throws Exception
   {
      for (int i = 0; i < numOfJoints; i++)
      { 
         String activeJointName = urdfModel.getActiveJointName(i);
         Double newValue =  anglesToUseAsInitialState.get( activeJointName );

         if( newValue == null) {
            q_init.set(i, newValue.doubleValue());
         }
      }

      int ret = compute(actualSdfModel, null,  ComputeOption.USE_JOINTS_CACHE);

      for (int i = 0; i < numOfJoints; i++)
      {
         double new_value = q_init.get(i);
         String jointName =  urdfModel.getActiveJointName(i);

         outputAngles.put(jointName, new Double(new_value) );
      }
      return ret;
   }

   public int compute(SDFFullRobotModel actualSdfModel, SDFFullRobotModel desiredRobotModelToPack, ComputeOption opt) throws Exception
   {
      if( desiredRobotModelToPack == actualSdfModel)
      {
         throw new Exception(" the output of compute should not be a reference to actual_sdf_model");
      }

      switch(opt)
      {
      case RESEED:
         Vector64F randQ = getHierarchicalSolver().getRandomQ();

         for (int i = 0; i < numOfJoints; i++)
         {
            boolean partOfQuietArm = false;

            for(RobotSide robotSide: RobotSide.values())
            {
               for(int j=0; j< getNumberDoFperArm(); j++){
                  if ( armJointIds.get(robotSide)[j] == i){
                     partOfQuietArm = true;
                     break;
                  }
               }
            }           

            if( !partOfQuietArm ) {
               q_init.set(i, randQ.get(i));
            }
         }

         // TODO: this is robot specific ! Move it to AtlasWholeBodyIk
         q_init.set(jointNamesToIndex.get("l_leg_aky"), -0.7);
         q_init.set(jointNamesToIndex.get("r_leg_aky"), -0.7);
         q_init.set(jointNamesToIndex.get("l_leg_kny"), 1.4);
         q_init.set(jointNamesToIndex.get("r_leg_kny"), 1.4);
         q_init.set(jointNamesToIndex.get("l_leg_hpy"), -0.7);
         q_init.set(jointNamesToIndex.get("r_leg_hpy"), -0.7);

         if( keepArmQuiet.get(LEFT).isFalse())
            q_init.set(jointNamesToIndex.get("l_arm_elx"), 0.7);
         if( keepArmQuiet.get(RIGHT).isFalse())
            q_init.set(jointNamesToIndex.get("r_arm_elx"), -0.7);

         break;
      case USE_ACTUAL_MODEL_JOINTS:
         actualSdfModel.updateFrames();
         for (int i = 0; i < numOfJoints; i++)
         {
            OneDoFJoint joint = actualSdfModel.getOneDoFJointByName(urdfModel.getActiveJointName(i));
            q_init.set(i, joint.getQ());
         }
         break;

      case USE_JOINTS_CACHE:
         //do nothing, the cache is used by default.
      }

      enforceControlledDoF( opt );

      int ret = computeImpl(actualSdfModel, desiredRobotModelToPack);

      if(opt != ComputeOption.RESEED)
      {
         keepArmQuiet.get(LEFT).setValue(true);
         keepArmQuiet.get(RIGHT).setValue(true);
      }
      return ret;
   }

   private void adjustDesiredCOM(SDFFullRobotModel actualSdfModel )
   {
      ReferenceFrame leftSole = actualSdfModel.getSoleFrame(LEFT);
      ReferenceFrame rightSole = actualSdfModel.getSoleFrame(RIGHT);
      RigidBodyTransform rightToLeftFoot = leftSole.getTransformToDesiredFrame( rightSole );
      Vector3d diff = new Vector3d();
      rightToLeftFoot.getTranslation(diff); 

      taskComPosition.setTarget( new Vector64F(3, diff.x/2, diff.y/2, 0), 0.01 );
   }

   private void movePelvisToHaveOverlappingFeet(SDFFullRobotModel referenceModel, SDFFullRobotModel followerModel)
   {
      referenceModel.updateFrames();
      followerModel.updateFrames();

      ReferenceFrame referenceSoleFrame = referenceModel.getSoleFrame( getSideOfTheFootRoot() );
      ReferenceFrame followerSoleFrame  = followerModel.getSoleFrame( getSideOfTheFootRoot() );

      RigidBodyTransform soleWorldToFollow = new RigidBodyTransform();
      RigidBodyTransform soleToPelvis      = new RigidBodyTransform();
      RigidBodyTransform pelvisTransform   = new RigidBodyTransform();

      ReferenceFrame followerPelvisFrame = followerModel.getRootJoint().getFrameAfterJoint();

      followerPelvisFrame.getTransformToDesiredFrame(soleToPelvis,  followerSoleFrame);
      referenceSoleFrame.getTransformToDesiredFrame(soleWorldToFollow, ReferenceFrame.getWorldFrame());
      pelvisTransform.multiply( soleWorldToFollow, soleToPelvis);

      followerModel.getRootJoint().setPositionAndRotation(pelvisTransform);
      followerModel.updateFrames();
   }


   synchronized private int computeImpl(SDFFullRobotModel actualSdfModel, SDFFullRobotModel desiredRobotModelToPack)
   {    
      movePelvisToHaveOverlappingFeet( actualSdfModel, workingSdfModel );

      int ret = -1;
      Vector64F q_out = new Vector64F(numOfJoints);

      try{
         adjustDesiredCOM(actualSdfModel);
         checkIfLegsNeedToBeLocked();
         setPreferedKneeAngle();
         adjustOtherFoot(actualSdfModel);

         checkIfArmShallStayQuiet(q_init);
         ret = hierarchicalSolver.solve(q_init, q_out);
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

      if (ret != -2)
      {
         q_init.set(q_out);

         HashMap<String, Double> listOfNewJointAngles = new HashMap<String, Double>();

         for (int i = 0; i < workingJointsInUrdfOrder.length; i++)
         {
            if (workingJointsInUrdfOrder[i] != null)
            {
               String jointName =  workingJointsInUrdfOrder[i].getName();
               double Q = q_out.get(i);
               workingJointsInUrdfOrder[i].setQ( Q );
               listOfNewJointAngles.put( jointName, Q );
            }
         }

         movePelvisToHaveOverlappingFeet( actualSdfModel, workingSdfModel );

         if(desiredRobotModelToPack != null)
         {
            InverseDynamicsJointStateCopier desiredModelJointCopier = new InverseDynamicsJointStateCopier(
                  workingSdfModel.getElevator(), 
                  desiredRobotModelToPack.getElevator());

            desiredModelJointCopier.copy();
            desiredRobotModelToPack.updateFrames();
         }
      }

      if(desiredRobotModelToPack != null)
      {
         movePelvisToHaveOverlappingFeet( actualSdfModel, desiredRobotModelToPack );
      }

      return ret;
   }

   public double getDesiredJointAngle(String jointName)
   {
      int index = jointNamesToIndex.get(jointName);
      return workingJointsInUrdfOrder[index].getQ();
   }

   private void adjustOtherFoot(SDFFullRobotModel actualSdfModel )
   {
      RigidBodyTransform foot_other_transform = new RigidBodyTransform();
      // note before and after... it is not a typo.
      ReferenceFrame leftAnkle = actualSdfModel.getOneDoFJointByName("l_leg_akx").getFrameAfterJoint();
      //TODO. there might be something wrong here
      leftAnkle.getTransformToDesiredFrame(foot_other_transform, getRootFrame(actualSdfModel) );

      Quat4d quat = new Quat4d();
      Vector3d pos = new Vector3d();

      foot_other_transform.get(quat, pos);
      taskLegPose.setTarget(quat, pos, 0.0005);
   }


   public ReferenceFrame getDesiredAfterJointFrame(String jointName, ReferenceFrame parentFrame)
   {
      workingFrames.updateFrames();

      ReferenceFrame workingJointFrame = workingSdfModel.getOneDoFJointByName(jointName).getFrameAfterJoint();
      ReferenceFrame workingRootFrame  = getRootFrame();

      RigidBodyTransform rootToJoint  = workingJointFrame.getTransformToDesiredFrame( workingRootFrame );
      RigidBodyTransform parentToRoot = workingRootFrame.getTransformToDesiredFrame( parentFrame );

      RigidBodyTransform parentToBody = new RigidBodyTransform();
      parentToBody.multiply(parentToRoot, rootToJoint);

      return ReferenceFrame.constructBodyFrameWithUnchangingTransformToParent(jointName, parentFrame, parentToBody);
   }

   public void updateWorkingModel( SDFFullRobotModel sdfModel)
   {
      workingSdfModel.copyAllJointsButKeepOneFootFixed(sdfModel.getOneDoFJoints(), getSideOfTheFootRoot());
      movePelvisToHaveOverlappingFeet( sdfModel, workingSdfModel );
   }

   public ReferenceFrame getDesiredHandFrame(RobotSide handSide, ReferenceFrame parentFrame)
   {
      return getDesiredBodyFrame(getEndEffectorLinkName(handSide), parentFrame);
   }

   public ReferenceFrame getDesiredBodyFrame(String name, ReferenceFrame parentFrame)
   {
      workingFrames.updateFrames();

      RigidBodyTransform rootToBody   = getLocalBodyTransform(name);
      RigidBodyTransform parentToRoot =  getRootFrame().getTransformToDesiredFrame(parentFrame);

      RigidBodyTransform parentToBody = new RigidBodyTransform();
      parentToBody.multiply(parentToRoot, rootToBody);

      return ReferenceFrame.constructBodyFrameWithUnchangingTransformToParent(name, parentFrame, parentToBody);
   }


   public ControlledDoF getNumberOfControlledDoF(RobotSide side)
   {
      return controlledDoF.get(side);
   }

   public void setNumberOfControlledDoF(RobotSide side, ControlledDoF dof)
   {
      controlledDoF.set(side, dof );
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
