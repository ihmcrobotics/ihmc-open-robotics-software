package us.ihmc.wholeBodyController;

import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Random;
import java.util.Map.Entry;

import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import org.apache.commons.lang3.SystemUtils;
import org.apache.commons.lang3.mutable.MutableBoolean;
import org.ejml.data.DenseMatrix64F;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.robotics.hierarchicalKinematics.ForwardKinematicSolver;
import us.ihmc.robotics.hierarchicalKinematics.HierarchicalKinematicSolver;
import us.ihmc.robotics.hierarchicalKinematics.HierarchicalTask;
import us.ihmc.robotics.hierarchicalKinematics.HierarchicalTask_BodyPose;
import us.ihmc.robotics.hierarchicalKinematics.HierarchicalTask_COM;
import us.ihmc.robotics.hierarchicalKinematics.HierarchicalTask_JointsPose;
import us.ihmc.robotics.hierarchicalKinematics.RobotModel;
import us.ihmc.robotics.humanoidRobot.frames.ReferenceFrames;
import us.ihmc.utilities.io.printing.PrintTools;
import us.ihmc.robotics.dataStructures.Vector64F;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.PoseReferenceFrame;
import us.ihmc.robotics.geometry.ReferenceFrame;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.utilities.nativelibraries.NativeLibraryLoader;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.utilities.robotSide.SideDependentList;
import us.ihmc.utilities.screwTheory.CenterOfMassCalculator;
import us.ihmc.utilities.screwTheory.InverseDynamicsJointStateCopier;
import us.ihmc.utilities.screwTheory.OneDoFJoint;


/**
 * This class is a front-end to the HierarchicalIKSolver which assume a humanoid shape.
 * <p>
 * It adds some customization and initialization and an higher level API on top of it.
 * A specific robot should implement several abstract methods, in particular configureTasks().
 * 
 * IMPORTANT: we expect the URDF to have a robot where one of the foot is the root of the kinematic chain.
 * Unfortunately this means that every time you have a new robot (once every few months), you might need to
 * spend something like 1 hour to create such URDF file manually.
 * 
 */
abstract public class WholeBodyIkSolver
{
   static public enum ComputeOption { 

      /**  create a random joint pose and search again a solution.*/
      RESEED,         
      /** Use the actual position of the robot's joints as initial guess. */
      USE_ACTUAL_MODEL_JOINTS,   
      /** Use the last computed position of the joints as initial guess. */
      USE_JOINTS_CACHE,         
      /** Call again the solver and try to converge to a more stable solution. */
      RELAX }


   static public enum ComputeResult { 

      /** The solver is disabled (i.e. using LockLevel.DISABLED). */
      FAILED_DISABLED,  
      /** Solution is completely wrong. */
      FAILED_INVALID,         
      /** calculated solution is not good enough, but you might want to double check the error to see if it is acceptable. */
      FAILED_NOT_CONVERGED,
      /** Solution converged. */
      SUCCEEDED             
   };

   /**
    * The Enum ControlledDoF.
    */
   static public enum ControlledDoF { 

      /** don't control this end effector. */
      DOF_NONE, 
      /** control only the position of the end effector. */
      DOF_3P,  
      /** control position and rotation (6 DoF) of the end effector. */
      DOF_3P3R,  
      /** Control 3 positions and only 2 rotations. */
      DOF_3P2R  
   }


   static public enum LockLevel{

      /** The disabled. */
      DISABLED,
      /** Use everything you can. */
      USE_WHOLE_BODY,   
      /** Lock only the pelvis orientation */
      LOCK_PELVIS_ORIENTATION,
      /** Disable the taskCoM and allow to control everything, including the pose of both feet and the pelvis. */
      CONTROL_MANUALLY,
      /** Similar to CONTROL_MANUALLY, but pelvis is locked. */
      CONTROL_MANUALLY_AND_LOCK_WAIST,
      /** None of the joints of the legs will move. It disables taskCoM. */
      LOCK_LEGS,        
      /** None of the joints of the legs will move; waist X and Y are locked too. It disables taskCoM. */
      LOCK_LEGS_AND_WAIST_X_Y,  
      /** None of the joints of the legs and the waist will move. It disables taskCoM. */
      LOCK_LEGS_AND_WAIST       
   };


      /**
       * The Class WholeBodyConfiguration contains all the parameters that affect the behavior of the solver.
       * They are grouped together to make it easier to save and restore the internal state of the solver.
       */
      public class WholeBodyConfiguration
      {

         private LockLevel lockLevel = LockLevel.LOCK_PELVIS_ORIENTATION;
         private final SideDependentList<ControlledDoF> controlledDoF;
         private int maxNumberOfAutomaticReseeds = 0;

         private final HashMap<String,Double> jointPreferedAngle;

         public WholeBodyConfiguration()
         {
            controlledDoF = new SideDependentList<ControlledDoF>();
            jointPreferedAngle = new HashMap<String,Double> ();
         }

         public WholeBodyConfiguration(WholeBodyConfiguration other )
         {
            lockLevel = other.lockLevel;
            controlledDoF = new SideDependentList<ControlledDoF>( other.controlledDoF);
            maxNumberOfAutomaticReseeds = other.maxNumberOfAutomaticReseeds;
            jointPreferedAngle = (HashMap<String, Double>) other.jointPreferedAngle.clone();
         }

         public LockLevel getLockLevel() {
            return lockLevel;
         }


         public int getMaxNumberOfAutomaticReseeds() {
            return maxNumberOfAutomaticReseeds;
         }

         public ControlledDoF getNumberOfControlledDoF(RobotSide side)
         {
            return controlledDoF.get(side);
         }

         public HashMap<String,Double> getPreferedJointAngle()
         {
            return jointPreferedAngle;
         }


         public void setAllPreferedJointAngle(HashMap<String,Double> jointPreferedAngle)
         {
            for(Map.Entry<String,Double> entry : jointPreferedAngle.entrySet())
            {
               setPreferedJointAngle( entry.getKey(),  entry.getValue());
            }
         }

         public void setLockLevel(LockLevel lockLevel) {
            this.lockLevel = lockLevel;
         }

         /**
          * "Automatic reseed" means that if the hierarchical solver can't find a solution, 
          * a new set of random joint angles will be used and the solver will be called again.
          * PROS: it helps to find a solution.
          * CONS: such solution will be discontinuous (i.e. might have large motion at the joint level).
          *
          * @param maxNumberOfAutomaticReseeds the new max number of automatic reseeds.
          */
         public void setMaxNumberOfAutomaticReseeds(int maxNumberOfAutomaticReseeds)
         {
            if( maxNumberOfAutomaticReseeds <0) maxNumberOfAutomaticReseeds = 0;
            this.maxNumberOfAutomaticReseeds = maxNumberOfAutomaticReseeds;
         }

         /**
          * Sets the number of controlled degrees of freedom of an end effector. See ControlledDoF.
          *
          * @param side the side of the end effector.
          * @param controlled_dof the parameter.
          */
         public void setNumberOfControlledDoF(RobotSide side, ControlledDoF controlled_dof)
         {
            controlledDoF.set(side, controlled_dof);
         }


         /**
          * Sets the preferred joint angle.
          *
          * @param joint_name the joint_name
          * @param preferredQ the preferred q
          */
         public void setPreferedJointAngle(String joint_name, double preferredQ)
         {
            jointPreferedAngle.put(joint_name, preferredQ);
         }
      }
      // Load (only once) the external libraries.
      static
      {
         try
         {
            NativeLibraryLoader.loadLibrary("us.ihmc.robotics.hierarchicalKinematics", "hik_java");
            NativeLibraryLoader.loadLibrary("us.ihmc.convexOptimization", "qpOASESSwig_rel");
         }
         catch (UnsatisfiedLinkError e)
         {
            if (SystemUtils.IS_OS_WINDOWS)
               PrintTools.error("Install Visual C++ Redistributable Packages for Visual Studio 2015!!");
         }
      }

      final private static RobotSide RIGHT = RobotSide.RIGHT;
      final private static RobotSide LEFT  = RobotSide.LEFT;


      private final ReferenceFrames workingFrames;
      private final SideDependentList<ReferenceFrame> workingSoleFrames = new SideDependentList<ReferenceFrame>();

      private final RobotModel        urdfModel;

      /** This model is used to store the result of the solver and/or as initial state for the next update. */
      private final SDFFullRobotModel cachedModel;

      protected final HierarchicalKinematicSolver hierarchicalSolver;

      /** Task that controls the Center of Mass. */
      public HierarchicalTask_COM        taskCoM;

      /** This task is the last in the hierarchy and controls the posture of the robot. */
      public HierarchicalTask_JointsPose taskJointsPose;



      /**  Task use to control mostly the pelvis. Note that when taskComPosition is active, this task 
       * should control only rotation when taskCoM is enabled.*/

      public final HierarchicalTask_BodyPose taskPelvisPose;

      /** Task that controls the foot position of the leg opposite to the root foot. */
      public final HierarchicalTask_BodyPose taskLegPose;

      // NOTE: HierarchicalTask_BodyPose can be used to control 6 degrees of freedom.
      // Nevertheless we split this 6 DoF task in two tasks, each of them controlling 3 DoF.
      // This allows us to prioritize the position task over the rotation one.
      
      /** The task that controls the end effector position. */
      public final SideDependentList<HierarchicalTask_BodyPose> taskEndEffectorPosition = new SideDependentList<HierarchicalTask_BodyPose>();

      /** The task that controls the end effector rotation. */
      public final SideDependentList<HierarchicalTask_BodyPose> taskEndEffectorRotation = new SideDependentList<HierarchicalTask_BodyPose>();

      protected final SideDependentList<MutableBoolean> keepArmQuiet = new SideDependentList<MutableBoolean>();
      private boolean suggestKneeAngle = true;

      protected final WholeBodyConfiguration parameters = new WholeBodyConfiguration();

      protected final HashMap<String, Integer> jointNamesToIndex = new HashMap<String, Integer>();

      /** This array is ordered to make an easy mapping from the order used by HierarchicalKinematicSolver */
      protected final OneDoFJoint[] workingJointsInUrdfOrder;

      protected final SideDependentList<HashSet<Integer>> armJointIds = new SideDependentList<HashSet<Integer>>();
      protected final SideDependentList<HashSet<Integer>> legJointIds = new SideDependentList<HashSet<Integer>>();
      protected final int[] waistJointId;

      protected final DenseMatrix64F coupledJointWeights;

      /** This is used to store the initial position that is sent to the Hierarchical solver. */
      protected final Vector64F cachedAnglesQ;

      final private int numOfJoints;
      private HashSet<String> listOfVisibleAndEnabledColliders;

      protected final SideDependentList<FramePose> handTarget = new SideDependentList<FramePose>();
      protected final SideDependentList<FramePose> feetTarget = new SideDependentList<FramePose>();
      protected final FramePose pelvisTarget = new FramePose();


      protected final ArrayList<ActionListener> subscribers = new ArrayList<ActionListener>();

      /** The random generator. */
      // NOTE we use a fixed seed to make unit tests more repeatable
      final Random randomGenerator = new Random(666);
      //---------------------------------------------------------------------------
      /**
       * Configure collision avoidance should be implemented by the user.
       * It must add the pair of bodies to be checked using the method  
       * hierarchicalSolver.collisionAvoidance.addBodyPair().
       *
       * @param urdfFileName the urdf file name.
       * @return a list of names of all the bodies that are involved in the collision detection.
       */
      abstract protected HashSet<String> configureCollisionAvoidance(String urdfFileName )  throws Exception;

      /**
       * This is one of the most important "robot-specific" abstract methods to be implemented.
       * Its responsibility is to:
       * 
       * -) Set the weights (in task and joint space) of each of the {@link HierarchicalTask}(s).
       * -) configure the parameters of the solver (see class {@link WholeBodyConfiguration}).
       */
      abstract protected void configureTasks();
      
      /**
       * Gets the foot link name.
       *
       * @param side the side
       * @return the foot link name
       */
      abstract public String getFootLinkName(RobotSide side);
      
      /**
       * Gets the gripper palm link name. "Palm" refers to the center of the gripper, as opposite to "wrist".
       *
       * @param side the side
       * @return the gripper palm link name.
       */
      abstract public String getGripperPalmLinkName(RobotSide side);
      
      /**
       * Gets the urdf robot model.
       */
      public RobotModel getUrdfRobotModel()
      {
         return urdfModel;
      }

      /**
       * Gets the side of the foot root. 
       */
      abstract public RobotSide getSideOfTheFootRoot();

      /**
       * Gets the number Degrees of Freedom per arm.
       */
      abstract public int getNumberDoFperArm();

      /**
       * Gets the number Degrees of Freedom per leg.
       */
      abstract public int getNumberDoFperLeg();
      
      /**
       * You can provide default angles for the reseed process.
       * Note that you don't need to provide a suggestion for each angle.
       * 
       * @return non exhaustive list of suggested angles.
       */
      abstract public HashMap<String,Double> getSuggestedAnglesForReseed();

      /**
       * Gets the URDF configuration file name.
       */
      abstract public String getURDFConfigurationFileName()  throws IOException ;

      /**
       * Gets the waist joint id.
       */
      abstract public int[] getWaistJointId();
      
      
      abstract public String getJointNextToFoot(RobotSide side);
      
      //---------------------------------------------------------------------------
      
      public WholeBodyIkSolver( WholeBodyControllerParameters robotControllerParameters ) 
      {
         // load external library
         cachedModel  = robotControllerParameters.createFullRobotModel();
         workingFrames = new ReferenceFrames(cachedModel);

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

         double EpsilonInRadiansForVerySmallAngles = 0.001;
         int NumberOfIterations = 60;

         hierarchicalSolver = new HierarchicalKinematicSolver(urdfModel, NumberOfIterations, EpsilonInRadiansForVerySmallAngles);
         hierarchicalSolver.setVerbosityLevel(0);

         cachedAnglesQ = new Vector64F(numOfJoints);

         for (RobotSide robotSide : RobotSide.values())
         {
            workingSoleFrames.set(robotSide, workingFrames.getSoleFrame(robotSide));
            armJointIds.set(robotSide, new HashSet<Integer>());
            legJointIds.set(robotSide, new HashSet<Integer>());
            parameters.setNumberOfControlledDoF(robotSide, ControlledDoF.DOF_NONE );
            keepArmQuiet.set(robotSide, new MutableBoolean(true));
         }

         for (int i = 0; i < numOfJoints; i++)
         {
            String joint_name_in_urdf = urdfModel.getActiveJointName(i);
            jointNamesToIndex.put(joint_name_in_urdf, i);
            OneDoFJoint joint = cachedModel.getOneDoFJointByName(joint_name_in_urdf);
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

         taskCoM = new HierarchicalTask_COM("COM Position",fk);
         taskPelvisPose = new HierarchicalTask_BodyPose("PelvisPose",fk, urdfModel, "pelvis");

         for (RobotSide robotSide : RobotSide.values())
         {
            String gripperPalm = getGripperPalmLinkName(robotSide);
            int currentHandId = urdfModel.getBodyId(gripperPalm);

            String sideName = (robotSide == RobotSide.LEFT) ? "Left" : "Right";

            taskEndEffectorPosition.set(robotSide, 
                  new HierarchicalTask_BodyPose(sideName + " Hand Position", fk, urdfModel, gripperPalm));
            taskEndEffectorRotation.set(robotSide, 
                  new HierarchicalTask_BodyPose(sideName + " Hand Rotation", fk, urdfModel, gripperPalm));

            feetTarget.set(robotSide, null );
            handTarget.set(robotSide, null );

            int [] indexes = new  int[ getNumberDoFperArm()];
            urdfModel.getParentJoints(currentHandId, getNumberDoFperArm() , indexes);
            for (int i: indexes)
            {
               armJointIds.get(robotSide).add( i );
            }
         }     

         {
            RobotSide otherFootSide = getSideOfTheFootRoot().getOppositeSide(); 
            int leftFootId = urdfModel.getBodyId(  getFootLinkName(otherFootSide ) );
            taskLegPose = new HierarchicalTask_BodyPose("Left Leg Pose",fk,urdfModel,  getFootLinkName(otherFootSide ));

            int [] indexes = new  int[ getNumberDoFperArm()];
            urdfModel.getParentJoints(leftFootId, getNumberDoFperLeg(), indexes );
            for (int i: indexes)
            {
               legJointIds.get(otherFootSide).add( i );
            }
         }

         for (int i = 0; i < getNumberDoFperLeg(); i++)
         {
            legJointIds.get( getSideOfTheFootRoot()).add(i);
         }

         taskJointsPose = new HierarchicalTask_JointsPose("Whole body Posture", fk);


         // the order is important!! first to be added have higher priority
         hierarchicalSolver.addTask(taskPelvisPose);
         hierarchicalSolver.addTask(taskLegPose); 
         hierarchicalSolver.addTask(taskCoM);

         hierarchicalSolver.addTask(taskEndEffectorPosition.get(RIGHT));
         hierarchicalSolver.addTask(taskEndEffectorPosition.get(LEFT));

         hierarchicalSolver.addTask(taskEndEffectorRotation.get(RIGHT));
         hierarchicalSolver.addTask(taskEndEffectorRotation.get(LEFT));

         hierarchicalSolver.addTask(taskJointsPose);

         coupledJointWeights = new DenseMatrix64F(numOfJoints, numOfJoints);
         waistJointId = getWaistJointId();

         configureTasks();
         try{
            listOfVisibleAndEnabledColliders = configureCollisionAvoidance( getURDFConfigurationFileName() );
         }
         catch(Exception e)
         {
            e.printStackTrace();
         }
      }

      /**
       * Activate automatic preferred hip height.
       */
      public void activateAutomaticPreferredHipHeight(boolean enable)
      {
         suggestKneeAngle = enable;
      }

      /**
       * Adds the action listener.
       */
      public void addActionistener( ActionListener listener)
      {
         subscribers.add( listener );
      }

      public double calculateCenterOfMassError(SDFFullRobotModel actualSdfModel )
      {
         ReferenceFrame leftSole = actualSdfModel.getSoleFrame(LEFT);
         ReferenceFrame rightSole = actualSdfModel.getSoleFrame(RIGHT);

         Vector3d leftFoot = new Vector3d();
         Vector3d rightFoot = new Vector3d();

         leftSole.getTransformToWorldFrame().getTranslation(leftFoot);
         rightSole.getTransformToWorldFrame().getTranslation(rightFoot);

         Vector3d desiredCOM =  new Vector3d( 
               (leftFoot.getX() + rightFoot.getX())*0.5,
               (leftFoot.getY() + rightFoot.getY())*0.5, 
               0.0 );

         CenterOfMassCalculator comCalculator = new CenterOfMassCalculator(actualSdfModel.getElevator(), ReferenceFrame.getWorldFrame() );
         comCalculator.compute();
         FramePoint calculatedCOM = comCalculator.getCenterOfMass();

         double x1 = calculatedCOM.getPoint().getX();
         double y1 = calculatedCOM.getPoint().getY();

         double x2 = desiredCOM.getX();
         double y2 = desiredCOM.getY();

         double dist = Math.sqrt( (x1-x2)*(x1-x2) + (y1-y2)*(y1-y2)  );
         return dist;
      }

      /**
       * Helper function to check if a certain robot state is self-colliding or not.
       *
       * @param modelToCheck the model.
       * @return true, if there is a self collision.
       */
      public boolean checkCollisions(SDFFullRobotModel modelToCheck)
      {
         HashMap<String,RigidBodyTransform> bodyTransforms = new HashMap<String,RigidBodyTransform>();
         for( OneDoFJoint joint : modelToCheck.getOneDoFJoints())
         {
            String bodyName     = joint.getSuccessor().getName();
            RigidBodyTransform bodyTransform = joint.getFrameAfterJoint().getTransformToWorldFrame();

            bodyTransforms.put(bodyName, bodyTransform);
         }

         return getHierarchicalSolver().collisionAvoidance.hasCollision( bodyTransforms );
      }

      public WholeBodyConfiguration cloneConfiguration()
      {
         return new WholeBodyConfiguration(parameters);
      }

      public ComputeResult compute(SDFFullRobotModel actualSdfModel,  Map<String, Double> anglesToUseAsInitialState,  Map<String, Double> outputAngles) throws Exception
      {
         for (int i = 0; i < numOfJoints; i++)
         { 
            String activeJointName = urdfModel.getActiveJointName(i);
            Double newValue =  anglesToUseAsInitialState.get( activeJointName );

            if( newValue != null) {
               cachedAnglesQ.set(i, newValue.doubleValue());
            }
         }

         ComputeResult ret = compute(actualSdfModel, null,  ComputeOption.USE_JOINTS_CACHE);

         if( ret != ComputeResult.FAILED_INVALID)
         {
            for (int i = 0; i < numOfJoints; i++)
            {
               double new_value = cachedAnglesQ.get(i);
               String jointName =  urdfModel.getActiveJointName(i);
               outputAngles.put(jointName, new Double(new_value) );
            }
         }
         return ret;
      }

      public ComputeResult compute(SDFFullRobotModel actualSdfModel, SDFFullRobotModel desiredRobotModelToPack, ComputeOption opt) throws Exception
      {
         if( desiredRobotModelToPack == actualSdfModel)
         {
            throw new Exception(" the output of compute should not be a reference to actual_sdf_model");
         }  

         switch(opt)
         {
         case RESEED:

            reseedCachedModel();         

            break;
         case USE_ACTUAL_MODEL_JOINTS:
            actualSdfModel.updateFrames();
            for (int i = 0; i < numOfJoints; i++)
            {
               OneDoFJoint joint = actualSdfModel.getOneDoFJointByName(urdfModel.getActiveJointName(i));
               cachedAnglesQ.set(i, joint.getQ());
            }
            break;

         case USE_JOINTS_CACHE:
         case RELAX:

            break;
         }

         //
         adjustAllTargets( actualSdfModel );
         enforceControlledDoF( opt );

         //-------------------------------------------
         for(RobotSide side: RobotSide.values)
         {
            if( ( taskEndEffectorPosition.get(side).isErrorLessThanTolerance() || taskEndEffectorPosition.get(side).isEnabled() == false) && 
                  ( taskEndEffectorRotation.get(side).isErrorLessThanTolerance() || taskEndEffectorRotation.get(side).isEnabled() == false) )
            {
               keepArmQuiet.get( side ).setValue( true );
            }
            else{
               keepArmQuiet.get( side ).setValue( false );
            }
         }
         //-------------------------------------------

         ComputeResult ret = computeImpl(actualSdfModel, desiredRobotModelToPack, true );

         int reseedLeft = parameters.getMaxNumberOfAutomaticReseeds();


         Vector64F savedCacheQ = new Vector64F(cachedAnglesQ);

         while( reseedLeft > 0 && ret != ComputeResult.SUCCEEDED)
         { 
            savedCacheQ.set( cachedAnglesQ );
            reseedCachedModel();
            ret = computeImpl(actualSdfModel, desiredRobotModelToPack, true );           
            reseedLeft--;
            if( ret == ComputeResult.FAILED_INVALID)
            {
               cachedAnglesQ.set( savedCacheQ );
               //               System.out.println("RESTORE\n"+ cachedAnglesQ);
            }
         }

         return ret;
      }

      public SDFFullRobotModel getCachedModel()
      {
         return cachedModel;
      }

      //---------------------------------------------------------------------------------

      public WholeBodyConfiguration getConfiguration()
      {
         return parameters;
      }

      public ReferenceFrame getDesiredAfterJointFrame(String jointName, ReferenceFrame parentFrame)
      {
         workingFrames.updateFrames();

         ReferenceFrame workingJointFrame = cachedModel.getOneDoFJointByName(jointName).getFrameAfterJoint();
         ReferenceFrame workingRootFrame  = getRootFrame( cachedModel );

         RigidBodyTransform rootToJoint  = workingJointFrame.getTransformToDesiredFrame( workingRootFrame );
         RigidBodyTransform parentToRoot = workingRootFrame.getTransformToDesiredFrame( parentFrame );

         RigidBodyTransform parentToBody = new RigidBodyTransform();
         parentToBody.multiply(parentToRoot, rootToJoint);

         return ReferenceFrame.constructBodyFrameWithUnchangingTransformToParent(jointName, parentFrame, parentToBody);
      }

      public ReferenceFrame getDesiredBodyFrame(String name, ReferenceFrame parentFrame)
      {
         workingFrames.updateFrames();

         RigidBodyTransform rootToBody   = getLocalBodyTransform(name);
         RigidBodyTransform parentToRoot =  getRootFrame( cachedModel ).getTransformToDesiredFrame(parentFrame);

         RigidBodyTransform parentToBody = new RigidBodyTransform();
         parentToBody.multiply(parentToRoot, rootToBody);

         return ReferenceFrame.constructBodyFrameWithUnchangingTransformToParent(name, parentFrame, parentToBody);
      }

      public ReferenceFrame getDesiredGripperPalmFrame(RobotSide handSide, ReferenceFrame parentFrame)
      {
         return getDesiredBodyFrame(getGripperPalmLinkName(handSide), parentFrame);
      }

      public double getDesiredJointAngle(String jointName)
      {
         int index = jointNamesToIndex.get(jointName);
         return workingJointsInUrdfOrder[index].getQ();
      }



      public FramePose getFootTarget(RobotSide side)
      {
         return feetTarget.get(side);  
      }
      


      public HierarchicalKinematicSolver getHierarchicalSolver()
      {
         return hierarchicalSolver;
      }

      /**
       * Gets the id of body after joint. 
       * <p>
       * "After" means after the rotation of the joint, i.e. the coordinate frame of the child body.
       * <p>
       * NOTE: The after/before direction is reversed between the SDF and HIK models for the right leg.
       * This method uses the direction specified by the WB (reversed).
       *
       * @param jointId the joint id
       * @return the id of body after joint
       */
      public int getIdOfBodyAfterJoint(int jointId)
      {
         return  urdfModel.getChildBodyOfJoint(jointId);
      }

      /**
       * Gets the id of body after joint.
       * <p>
       * "After" means after the rotation of the joint, i.e. the coordinate frame of the child body.
       * <p>
       * NOTE: The after/before direction is reversed between the SDF and HIK models for the right leg.
       * This method uses the direction specified by the WB (reversed).
       *
       * @param jointname the joint name.
       * @return the id of body after joint.
       */
      public int getIdOfBodyAfterJoint(String jointname)
      {
         int jointId = urdfModel.getJointIndexByName(jointname);
         return  getIdOfBodyAfterJoint(jointId);
      }

      /**
       * Gets the body transform expressed in the coordinate system of the root body.
       *
       * @param bodyid the body id.
       * @return body transform expressed in the coordinate system of the root body.
       */
      public RigidBodyTransform getLocalBodyTransform(int bodyid )
      {
         RigidBodyTransform out = new RigidBodyTransform();
         getHierarchicalSolver().getForwardSolver().getBodyPose( bodyid, out);
         return out;
      }

      /**
       * Gets the body transform expressed in the coordinate system of the root body.
       *
       * @param bodyname the body name.
       * @return body transform expressed in the coordinate system of the root body.
       */
      public RigidBodyTransform getLocalBodyTransform(String bodyname )
      {
         int bodyid = getUrdfRobotModel().getBodyId( bodyname );
         return getLocalBodyTransform(bodyid);
      }


      public int getNumberOfJoints()
      {
         return urdfModel.getNrOfJoints();
      }

      public FramePose getPelvisTarget()
      {
         return pelvisTarget;
      }

      /**
       * Gets the root frame of a certain instance of SDFFullRobotModel.
       */
      public ReferenceFrame  getRootFrame( SDFFullRobotModel actualSdfModel)
      {
         if( feetTarget.get(getSideOfTheFootRoot()) != null && parameters.getLockLevel() == LockLevel.CONTROL_MANUALLY )
         {
            return new PoseReferenceFrame("rootFootFrame", feetTarget.get(getSideOfTheFootRoot()) );
         }
         else{
            String ankleJointName = getJointNextToFoot( getSideOfTheFootRoot() );
            return actualSdfModel.getOneDoFJointByName( ankleJointName ).getFrameAfterJoint();
         }
      }


      public ReferenceFrame getRootFrameOfWorkingModel(){
         return getRootFrame(cachedModel);
      }


      public ReferenceFrames getWorkingReferenceFrames()
      {
         return workingFrames;
      }

      /**
       * Checks for joint.
       *
       * @param jointName the joint name.
       * @return true, if it is present.
       */
      public boolean hasJoint(String jointName)
      {
         return (jointNamesToIndex.get(jointName) != null);
      }

      public boolean isBodyIncludedInCollision(String bodyName)
      {
         return listOfVisibleAndEnabledColliders.contains(bodyName);
      }

      public void setConfiguration(WholeBodyConfiguration other)
      {
         parameters.lockLevel = other.lockLevel;
         parameters.setNumberOfControlledDoF(LEFT, other.controlledDoF.get(LEFT) );
         parameters.setNumberOfControlledDoF(RIGHT, other.controlledDoF.get(RIGHT) );
         parameters.maxNumberOfAutomaticReseeds = other.maxNumberOfAutomaticReseeds;

         parameters.jointPreferedAngle.clear();
         for (Entry<String,Double> entry: other.jointPreferedAngle.entrySet() )
         {
            parameters.jointPreferedAngle.put( entry.getKey(), entry.getValue() );
         }
      }

      public void setFootTarget(RobotSide side, FramePose footPose)
      {
         feetTarget.set(side, footPose);  
      }


      /**
       * Sets the gripper palm target. 
       * <p>
       * We call "GripperPalm" the location where grasping shall be made.
       * Its position corresponds with the blue cylinder of the ModifiafleCylinderGrabber.
       *
       * @param endEffectorSide the end effector side.
       * @param endEffectorPose the end effector pose.
       */
      public void setGripperPalmTarget(RobotSide endEffectorSide, FramePose endEffectorPose)
      {
         handTarget.set(endEffectorSide, endEffectorPose);

         taskEndEffectorPosition.get(endEffectorSide).setBodyToControl( getGripperPalmLinkName(endEffectorSide));
         taskEndEffectorRotation.get(endEffectorSide).setBodyToControl( getGripperPalmLinkName(endEffectorSide));
      }

      public void setPelvisTarget(FramePose pelvisPose)
      {
         pelvisTarget.setPose( pelvisPose );
      }

      public void setPreferedJointAngle(String joint_name, double q)
      {
         getConfiguration().setPreferedJointAngle(joint_name, q);
      }

      /**
       * Sets the seed angles. They will be used only if you call the method compute(...)
       * using the parameter ComputeOption.USE_JOINTS_CACHE.
       *
       * @param seedAngles the seed angles
       */
      public void setSeedAngles(HashMap<String, Double> seedAngles)
      {
         for (Entry<String, Double> entry: seedAngles.entrySet()  )
         {
            Integer jointIndex = jointNamesToIndex.get( entry.getKey() );
            if( jointIndex != null )
            {
               double Q = entry.getValue();
               cachedAnglesQ.set(jointIndex,  Q );
            }
         }
      }


      /**
       * Sets the seed angles. They will be used only if you call the method compute(...)
       * using the parameter ComputeOption.USE_JOINTS_CACHE.
       *
       * @param seedAngles the new seed angles
       */
      public void setSeedAngles(OneDoFJoint[] seedAngles)
      {
         for (OneDoFJoint otherJoint: seedAngles  )
         {
            Integer jointIndex = jointNamesToIndex.get( otherJoint.getName() );
            if( jointIndex != null )
            {
               double Q = otherJoint.getQ();
               cachedAnglesQ.set(jointIndex,  Q );
            }
         }
      }

      /**
       * Verbosity levels:
       * 0: none
       * 1: show a resume.
       * 2: Lot of data...
       *
       * @param level the new verbosity level
       */
      public void setVerbosityLevel(int level)
      {
         hierarchicalSolver.setVerbosityLevel(level);
      }

      public void updateCachedModelModel( SDFFullRobotModel sdfModel)
      {
         cachedModel.copyAllJointsButKeepOneFootFixed(sdfModel.getOneDoFJoints(), getSideOfTheFootRoot());
         movePelvisToHaveOverlappingFeet( sdfModel, cachedModel );
      }

      private void adjustAllTargets(SDFFullRobotModel actualRobotModel)
      {
         Quat4d quat = new Quat4d();
         Vector3d pos = new Vector3d();
         RigidBodyTransform ee_transform;
         ReferenceFrame targetFrame;

         if( feetTarget.get(RobotSide.LEFT) == null || parameters.getLockLevel() != LockLevel.CONTROL_MANUALLY )
         {
            String ankleJointName = getJointNextToFoot( getSideOfTheFootRoot().getOppositeSide() );
            targetFrame = actualRobotModel.getOneDoFJointByName( ankleJointName ).getFrameAfterJoint() ;
         }
         else {
            RobotSide nonRootSide = getSideOfTheFootRoot().getOppositeSide();
            targetFrame = new PoseReferenceFrame("footReferenceFrame", feetTarget.get( nonRootSide ));
         }
         ee_transform = targetFrame.getTransformToDesiredFrame( getRootFrame( actualRobotModel ));
         ee_transform.get(quat, pos);
         taskLegPose.setTarget( quat, pos ); 

         if( pelvisTarget != null)
         {  
            targetFrame = new PoseReferenceFrame("pelvisReferenceFrame", pelvisTarget );
            ee_transform = targetFrame.getTransformToDesiredFrame( getRootFrame( actualRobotModel ));
            ee_transform.get(quat, pos);
            taskPelvisPose.setTarget( quat, pos ); 
         }

         for (RobotSide side: RobotSide.values)
         {
            if( handTarget.get(side) != null)
            {
               targetFrame = new PoseReferenceFrame("endEffectorPoseReferenceFrame", handTarget.get(side));
               ee_transform = targetFrame.getTransformToDesiredFrame( getRootFrame( actualRobotModel ));

               keepArmQuiet.get(side).setValue( false );

               ee_transform.get(quat, pos);

               taskEndEffectorPosition.get(side).setTarget( quat, pos ); 
               taskEndEffectorRotation.get(side).setTarget( quat, pos );
            }
         }
      }

      private void adjustDesiredCOM(SDFFullRobotModel actualSdfModel )
      {
         ReferenceFrame leftSole = actualSdfModel.getSoleFrame(LEFT);
         ReferenceFrame rightSole = actualSdfModel.getSoleFrame(RIGHT);
         RigidBodyTransform rightToLeftFoot = leftSole.getTransformToDesiredFrame( rightSole );
         Vector3d diff = new Vector3d();
         rightToLeftFoot.getTranslation(diff); 

         taskCoM.setTarget( new Vector64F(3, diff.x/2, diff.y/2, 0) );
      }

      /**
       * Adjust preferred joint angles.
       */
      private void adjustPreferedJointAngles()
      {
         Vector64F preferredAngles = new Vector64F( cachedAnglesQ.getNumElements() );
         preferredAngles.setZero();

         for (Entry<String, Double> entry: parameters.getPreferedJointAngle().entrySet()  )
         {
            int index = jointNamesToIndex.get( entry.getKey() );
            preferredAngles.set(index,  entry.getValue() );
         }
         taskJointsPose.setTarget(preferredAngles);
      }

      private void checkIfArmShallStayQuiet(SDFFullRobotModel actualSdfModel)
      {
         for( RobotSide side: RobotSide.values())
         {         
            boolean enable =  parameters.getNumberOfControlledDoF(side) != ControlledDoF.DOF_NONE;

            for (int jointId: armJointIds.get(side))
            {
               enableJointWeights(actualSdfModel, jointId, enable );
            }

            /*  if( taskEndEffectorPosition.get(side).isErrorLessThanTolerance() &&
                  taskEndEffectorRotation.get(side).isErrorLessThanTolerance() )
            {
               for (int jointId: armJointIds.get(side))
               {
                  taskJointsPose.getWeightMatrixTaskSpace().set(jointId, jointId,  0.1);
                  preferredJointPose.set( jointId, cachedAnglesQ.get(jointId) );
                  taskJointsPose.setTarget( preferredJointPose );
               }
            }
            else{
               for (int jointId: armJointIds.get(side))
               {
                  taskJointsPose.getWeightMatrixTaskSpace().set(jointId, jointId,  0.0);
               }
            }*/
         }
      }

      private void checkIfLegsAndWaistNeedToBeLocked(SDFFullRobotModel actualSdfModel)
      {
         boolean enableLeg   = true;
         boolean enableWaistZ  = true;
         boolean enableWaistXY = true;
         boolean lockPelvisRotation = false;
         boolean enableCOM = true;

         switch( parameters.getLockLevel() )
         {
         case USE_WHOLE_BODY: break;   

         case CONTROL_MANUALLY_AND_LOCK_WAIST:
            enableCOM = false;
            enableWaistZ = false;
            enableWaistXY = false;
            lockPelvisRotation = true;
            break;

         case CONTROL_MANUALLY:    
            enableCOM = false;
            lockPelvisRotation = true;
            break;

         case LOCK_PELVIS_ORIENTATION:
            lockPelvisRotation = true;
            break;

         case LOCK_LEGS:  
            enableLeg = false;
            enableCOM = false;
            break;

         case LOCK_LEGS_AND_WAIST_X_Y:
            enableLeg = false;
            enableCOM = false;
            enableWaistXY = false;
            break;

         case LOCK_LEGS_AND_WAIST:
            enableLeg = false;
            enableCOM = false;
            enableWaistZ = false;
            enableWaistXY = false;
            break;
            
         default:
            enableLeg = false;
            enableCOM = false;
            enableWaistZ = false;
            enableWaistXY = false;
            break;
         }

         taskPelvisPose.setEnabled( enableLeg && lockPelvisRotation );
         taskCoM.setEnabled( enableLeg && enableCOM  );
         taskLegPose.setEnabled( enableLeg );

         for (RobotSide side: RobotSide.values)
         {
            for (int jointId : legJointIds.get(side) )
            {
               enableJointWeights(actualSdfModel, jointId, enableLeg );
            }
         }  

         for (int i=0; i< waistJointId.length; i++)
         {
            int jointId = waistJointId[i];

            // the first element must be Z. sorry about the hidden assumption
            boolean enableWaist = ( i > 0 ) ? enableWaistXY : enableWaistZ;

            enableJointWeights(actualSdfModel, jointId, enableWaist );
         }
      }

      synchronized private ComputeResult computeImpl(SDFFullRobotModel actualSdfModel, SDFFullRobotModel desiredRobotModelToPack, boolean continueUntilPoseConverged)
      {    
         movePelvisToHaveOverlappingFeet( actualSdfModel, cachedModel );

         int ret = -1;
         Vector64F q_out = new Vector64F(numOfJoints);

         try{

            adjustDesiredCOM(actualSdfModel);
            controlPelvis( actualSdfModel );
            checkIfLegsAndWaistNeedToBeLocked(actualSdfModel);
            setPreferedKneeAngle();
            checkIfArmShallStayQuiet(actualSdfModel);
            adjustPreferedJointAngles(); 

            q_out.set(cachedAnglesQ);
            ret = hierarchicalSolver.solve(cachedAnglesQ, q_out,continueUntilPoseConverged);

         }
         catch (Exception e)
         {
            // TODO Auto-generated catch block
            e.printStackTrace();
         }

         ComputeResult result ;

         if( ret == -2) 
            result = ComputeResult.FAILED_INVALID;
         else if( ret == -1)
            result = ComputeResult.FAILED_NOT_CONVERGED;
         else 
            result = ComputeResult.SUCCEEDED;

         //------------------------------------------
         if (result != ComputeResult.FAILED_INVALID )
         {
            cachedAnglesQ.set(q_out);

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

            movePelvisToHaveOverlappingFeet( actualSdfModel, cachedModel );

            if(desiredRobotModelToPack != null)
            {
               InverseDynamicsJointStateCopier desiredModelJointCopier = new InverseDynamicsJointStateCopier(
                     cachedModel.getElevator(), 
                     desiredRobotModelToPack.getElevator());

               desiredModelJointCopier.copy();
               desiredRobotModelToPack.updateFrames();
            }
         }

         if(desiredRobotModelToPack != null)
         {
            movePelvisToHaveOverlappingFeet( actualSdfModel, desiredRobotModelToPack );
         }

         ActionEvent event = new ActionEvent(this, ret, result.toString() );
         for (int s=0; s< subscribers.size(); s++)
         {
            subscribers.get(s).actionPerformed( event );
         }

         return result;
      }

      private void controlPelvis(SDFFullRobotModel actualSdfModel)
      {
         Vector64F weightForRotationOnly; 

         if( parameters.getLockLevel() == LockLevel.LOCK_PELVIS_ORIENTATION)
         {
            weightForRotationOnly = new Vector64F(6, 0,0,0,  1,1,1 );

            ReferenceFrame rootFrame = this.getRootFrame(actualSdfModel);
            RigidBodyTransform pelvisPose = actualSdfModel.getRootJoint().getFrameAfterJoint().getTransformToDesiredFrame(rootFrame);
            Quat4d pelvisOrientation = new Quat4d();
            pelvisPose.getRotation(pelvisOrientation);
            taskPelvisPose.setTarget( pelvisOrientation, new Vector3d() );
         }
         else{
            weightForRotationOnly = new Vector64F(6, 1,1,1,  1,1,1 );
         }

         taskPelvisPose.setWeightsTaskSpace( weightForRotationOnly );
      }

      private void enableJointWeights(SDFFullRobotModel actualSdfModel, int index, boolean enable)
      {
         double val = enable ? 1 : 0; 

         taskJointsPose.getWeightsJointSpace().set(index, val);
         //  taskJointsPose.getWeightsError().set( index, val );

         getHierarchicalSolver().collisionAvoidance.getJointWeights().set(index, val);

         taskEndEffectorPosition.get(LEFT).getWeightsJointSpace().set(index, val);
         taskEndEffectorPosition.get(RIGHT).getWeightsJointSpace().set(index, val);

         taskEndEffectorRotation.get(LEFT).getWeightsJointSpace().set(index, val);
         taskEndEffectorRotation.get(RIGHT).getWeightsJointSpace().set(index, val);

         if( !enable ) {
            OneDoFJoint joint = actualSdfModel.getOneDoFJointByName(urdfModel.getActiveJointName(index));
            cachedAnglesQ.set(index, joint.getQ());
         }
      }

      private void enforceControlledDoF( ComputeOption opt ) throws Exception
      {
         double rotationWeight = 0.4;

         for ( RobotSide side: RobotSide.values)
         {

            switch( parameters.getNumberOfControlledDoF(side) )
            {
            case DOF_NONE:
               taskEndEffectorPosition.get(side).setEnabled(false);
               taskEndEffectorRotation.get(side).setEnabled(false);
               break;

            case DOF_3P:
               taskEndEffectorPosition.get(side).setEnabled(true);
               taskEndEffectorPosition.get(side).setWeightsTaskSpace( new Vector64F(6, 1, 1, 1,   0.0,  0.0, 0.0) );
     
               taskEndEffectorRotation.get(side).setEnabled(false);      
               taskEndEffectorRotation.get(side).setWeightsTaskSpace( new Vector64F(6, 0, 0, 0,   0, 0, 0) );          
               break;


            case DOF_3P3R:
               taskEndEffectorPosition.get(side).setEnabled(true);
               taskEndEffectorPosition.get(side).setWeightsTaskSpace( new Vector64F(6, 1, 1, 1,  0, 0, 0) );

               taskEndEffectorRotation.get(side).setEnabled(true);
               taskEndEffectorRotation.get(side).setWeightsTaskSpace( new Vector64F(6, 0, 0, 0,  0.5,  0.5,  0.5) );
               break;   

            case DOF_3P2R:

               taskEndEffectorPosition.get(side).setEnabled(true);
               taskEndEffectorPosition.get(side).setWeightsTaskSpace( new Vector64F(6, 1, 1, 1,  0, 0, 0) );           

               //-------------
               taskEndEffectorRotation.get(side).setEnabled(true);
               taskEndEffectorRotation.get(side).setWeightsTaskSpace( new Vector64F(6, 0, 0, 0,  1, 1, 1) );
 
               Vector3d axisToDisable = new Vector3d(0, 1, 0);
               taskEndEffectorRotation.get(side).disableAxisInTaskSpace(rotationWeight, axisToDisable );  

               break;
            }
         }
      }

      private Vector64F getRandomQ()
      {
         int N = getNumberOfJoints();
         final Vector64F q_out = new Vector64F(N);     
         RobotModel model = hierarchicalSolver.getRobotModel();

         for (int i = 0; i < N; i++) {
            double random_var = randomGenerator.nextDouble() * 2.0 - 1.0;
            double delta = 0.49 * (model.q_max(i) - model.q_min(i))
                  * random_var;

            q_out.set(i, 0.5 * (model.q_max(i) + model.q_min(i)) + delta);
         }
         return q_out;
      }

      private void movePelvisToHaveOverlappingFeet(SDFFullRobotModel referenceModel, SDFFullRobotModel followerModel)
      {
         referenceModel.updateFrames();
         followerModel.updateFrames();

         ReferenceFrame referenceSoleFrame =  getRootFrame( referenceModel ); 
         String ankleJointName = getJointNextToFoot( getSideOfTheFootRoot() );
         ReferenceFrame followerSoleFrame  =  followerModel.getOneDoFJointByName( ankleJointName ).getFrameAfterJoint();

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

      private void reseedCachedModel()
      {
         Vector64F newQ = getRandomQ();

         HashMap<String,Double> suggestedQ = getSuggestedAnglesForReseed();

         for( Map.Entry<String,Double> entry: suggestedQ.entrySet())
         {
            int index = jointNamesToIndex.get(entry.getKey());
            newQ.set( index,  entry.getValue() );
         }

         for (int i = 0; i < getNumberOfJoints(); i++)
         {
            boolean partOfQuietArm = false;
            boolean partOfQuietLeg = false;
            boolean partOfQuietWaist = false;

            for(RobotSide robotSide: RobotSide.values())
            {
               if(  armJointIds.get(robotSide).contains(i) )
               {
                  if( keepArmQuiet.get(robotSide).isTrue() || parameters.getNumberOfControlledDoF(robotSide) == ControlledDoF.DOF_NONE )
                  {
                     partOfQuietArm = true;
                  }
               }        
            }

            if( parameters.getLockLevel() == LockLevel.LOCK_LEGS_AND_WAIST &&  waistJointId[0] == i )  
            {
               partOfQuietWaist = true;
            }
            if( parameters.getLockLevel() != LockLevel.LOCK_LEGS_AND_WAIST || parameters.getLockLevel() != LockLevel.LOCK_LEGS_AND_WAIST_X_Y ) 
            {
               if( waistJointId[1] == i || waistJointId[2] == i )
                  partOfQuietWaist = true;
            }

            if( !( partOfQuietArm || partOfQuietLeg || partOfQuietWaist ) ) {
               cachedAnglesQ.set(i, newQ.get(i));
            } 
         }
      }

      // TODO: this is robot specific ! Move it to AtlasWholeBodyIk
      private void setPreferedKneeAngle()
      {
         if( suggestKneeAngle == false)
         {
            return;
         }
         HierarchicalTask_BodyPose taskL = taskEndEffectorPosition.get(LEFT);
         HierarchicalTask_BodyPose taskR = taskEndEffectorPosition.get(RIGHT);

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

         double preferredKneeAngle = 2.2 - Math.min(zL , zR);

         if( preferredKneeAngle < 0.6) preferredKneeAngle = 0.6;

         parameters.setPreferedJointAngle( getKneeJointName(RIGHT), preferredKneeAngle);
         parameters.setPreferedJointAngle( getKneeJointName(LEFT),  preferredKneeAngle);

      }

      abstract public String getKneeJointName(RobotSide side);

      

}
