package us.ihmc.wholeBodyController;

import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import java.util.ArrayList;
import java.util.Random;

import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import us.ihmc.SdfLoader.FullRobotModelVisualizer;
import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.graphics3DAdapter.graphics.appearances.AppearanceDefinition;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.utilities.Pair;
import us.ihmc.utilities.ThreadTools;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.math.geometry.RigidBodyTransform;
import us.ihmc.utilities.math.geometry.RotationFunctions;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.utilities.robotSide.SideDependentList;
import us.ihmc.wholeBodyController.WholeBodyIkSolver.ComputeOption;
import us.ihmc.wholeBodyController.WholeBodyIkSolver.ComputeResult;
import us.ihmc.wholeBodyController.WholeBodyIkSolver.ControlledDoF;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.graphics.BagOfBalls;
import us.ihmc.yoUtilities.graphics.YoGraphicsListRegistry;

public class WholeBodyIkSolverTestHelper
{
   private SDFFullRobotModel actualRobotModel;
   private SDFFullRobotModel desiredRobotModel;
   private WholeBodyIkSolver hikSolver;

   private final YoVariableRegistry registry = new YoVariableRegistry("WholeBodyIkSolverTestFactory_Registry");

//   private final ArrayList<Pair<FramePose, FramePose>> handArrayList = new ArrayList<Pair<FramePose, FramePose>>();
   private final double maxDistanceFromOriginInCartesianDirection = 5.0;
   private double randomRobotPositionX;
   private double randomRobotPositionY;
   private double randomRobotYaw;
   static private final double minAcceptableSuccessPercentage = 0.95;

   private final double ERROR_TOLERANCE = 0.01;
   private final boolean DEBUG = false;
   
   private SimulationConstructionSet scs;
   private FullRobotModelVisualizer  modelVisualizer;
   private BagOfBalls bagOfBalls;
   
   public void addGraphics(SimulationConstructionSet scs, FullRobotModelVisualizer  modelVisualizer)
   {
      this.scs = scs;
      this.modelVisualizer = modelVisualizer; 
      YoGraphicsListRegistry listRegistry = new YoGraphicsListRegistry();
      bagOfBalls = new BagOfBalls(50, 0.03, "targets", YoAppearance.Transparent(), registry, listRegistry );
      scs.addYoGraphicsListRegistry( listRegistry );
   }


   public WholeBodyIkSolverTestHelper(
         WholeBodyControllerParameters robotModel,
         SDFFullRobotModel actualRobotModel, 
         WholeBodyIkSolver solver)
   {
      this.actualRobotModel = actualRobotModel;
      this.desiredRobotModel = robotModel.createFullRobotModel();
      this.hikSolver = solver;

      Random random = new Random();
      randomRobotPositionX = maxDistanceFromOriginInCartesianDirection * random.nextDouble();
      randomRobotPositionY = maxDistanceFromOriginInCartesianDirection * random.nextDouble();
      randomRobotYaw = 2.0 * Math.PI * random.nextDouble();
      
    //  randomRobotPositionX = 0;
     // randomRobotPositionY = 0;
    //  randomRobotYaw = 0;
      
      Vector3d rootPosition = new Vector3d(randomRobotPositionX, randomRobotPositionY, 0.93);
      
      actualRobotModel.getRootJoint().setPosition(rootPosition);
      actualRobotModel.getRootJoint().setRotation(randomRobotYaw, 0.0, 0.0);

   }

   public void executeHandTargetTest(ControlledDoF dofToControlLeft, ControlledDoF dofToControlRight,
         ArrayList<Pair<FramePose, FramePose>> handTargetPairsArray, boolean thisTestIsStaticsBased)
   {
      hikSolver.setVerbosityLevel( scs == null ? 0:1 );
      hikSolver.getHierarchicalSolver().collisionAvoidance.setEnabled(false);

      double successCount = 0;
      ArrayList<FramePose> framesToPlotAsBlueSpheres = new ArrayList<FramePose>();

      SideDependentList<ControlledDoF> dofToControl = new SideDependentList<ControlledDoF>();
      dofToControl.set(RobotSide.LEFT, dofToControlLeft);
      dofToControl.set(RobotSide.RIGHT, dofToControlRight);
      
      for (Pair<FramePose, FramePose> framePair : handTargetPairsArray)
      {
         SideDependentList<FramePose> endEffectorTarget = new SideDependentList<FramePose>();
                  
         if ( framePair.first() != null)
         {
            endEffectorTarget.set(RobotSide.LEFT, moveDesiredFrameToRobotLocation( framePair.first() ) );
          //  endEffectorTargetLeft = endEffectorTarget.get(RobotSide.LEFT).getFramePointCopy();
         }
                     
         if ( framePair.second() != null)
         {
            endEffectorTarget.set(RobotSide.RIGHT, moveDesiredFrameToRobotLocation( framePair.second() ) );
            //endEffectorTargetLeft = endEffectorTarget.get(RobotSide.RIGHT).getFramePointCopy();
         }

         actualRobotModel.updateFrames();

         for (RobotSide robotSide : RobotSide.values())
         {
            hikSolver.setNumberOfControlledDoF(robotSide, dofToControl.get(robotSide));
            if (endEffectorTarget.get(robotSide) != null)
            {
               hikSolver.setGripperAttachmentTarget(actualRobotModel, robotSide, endEffectorTarget.get(robotSide));
            }
         }

         ComputeResult hikSolutionExists = ComputeResult.FAILED_INVALID;
         try
         {
            hikSolutionExists = hikSolver.compute(actualRobotModel, desiredRobotModel, ComputeOption.USE_ACTUAL_MODEL_JOINTS);
         }
         catch (Exception e)
         {
            e.printStackTrace();
         }

         System.out.println(getClass().getSimpleName() + ": ret(" + hikSolutionExists + ") hik solution "
         + (hikSolutionExists == ComputeResult.SUCCEEDED ? "found" : "not found"));


         actualRobotModel.copyAllJointsButKeepOneFootFixed( desiredRobotModel.getOneDoFJoints(), RobotSide.RIGHT );
         actualRobotModel.updateFrames();

         if (scs != null)
         {
            AppearanceDefinition appearance = YoAppearance.Green();
            if (hikSolutionExists != ComputeResult.SUCCEEDED)
            {
               appearance = YoAppearance.Red();
            }
            
            for (RobotSide side: RobotSide.values)
            {
               if( endEffectorTarget.get( side ) != null)
               {
                  bagOfBalls.setBallLoop(endEffectorTarget.get( side ).getFramePointCopy(), appearance );
               }
            }
            
            modelVisualizer.update(0);
            
            ThreadTools.sleep(100);
         }

         for (RobotSide side : RobotSide.values())
         {
            // you don't need to check if you are not trying to control any DoF
            if (dofToControl.get(side) != ControlledDoF.DOF_NONE && endEffectorTarget.get(side) != null)
            {
               ReferenceFrame calculatedHandFrame = hikSolver.getDesiredGripperAttachmentFrame(side, ReferenceFrame.getWorldFrame());        
               RigidBodyTransform calculatedAttachmnent = calculatedHandFrame.getTransformToWorldFrame();

               calculatedHandFrame = hikSolver.getDesiredGripperPalmFrame(side, ReferenceFrame.getWorldFrame());
               RigidBodyTransform calculatedPalm = calculatedHandFrame.getTransformToWorldFrame();
               
               RigidBodyTransform desiredAttachment = new RigidBodyTransform();
               endEffectorTarget.get(side).getRigidBodyTransform(desiredAttachment); 
               
               RigidBodyTransform actualAttachment = desiredRobotModel.getHandControlFrame(side).getTransformToWorldFrame();
            
               Vector3d errorVector = RigidBodyTransform.getTranslationDifference(desiredAttachment , actualAttachment);
                 
               if (DEBUG)
               {
                  System.out.println(" actual\n" + actualAttachment); 
                  System.out.println(" wanted\n" + desiredAttachment);        
                  System.out.println(" calculated Att\n" + calculatedAttachmnent);
                  System.out.println(" calculated Palm\n" + calculatedPalm);
  
                  System.out.println(" actual sole\n" + actualRobotModel.getSoleFrame(side).getTransformToWorldFrame() );

                  System.out.println("\n desired (local B)\n" + hikSolver.taskEndEffectorPose.get(side).getTarget());
                  System.out.println(" calculated (local)\n" + hikSolver.taskEndEffectorPose.get(side).getCurrent() + "\n\n");
               }

               if (hikSolutionExists == ComputeResult.SUCCEEDED)
               {
                  if (DEBUG)
                  {
                     if (errorVector.length() > ERROR_TOLERANCE)
                     {
                        System.out.println(getClass().getSimpleName() + ": [" + side + "] FAILURE - error vector is \n" + errorVector.toString()
                              + "\n length is " + errorVector.length() + " m");
                     }
                     else
                     {
                        System.out.println(getClass().getSimpleName() + ": [" + side + "] SUCCESS");
                     }
                  }
                  assertTrue(errorVector.length() < ERROR_TOLERANCE);
                  framesToPlotAsBlueSpheres.add(endEffectorTarget.get(side));
                  successCount++;
               }
               else
               {
                  if (!thisTestIsStaticsBased)
                  {
                     fail(getClass().getSimpleName() + ": hik solution not found");
                  }
               }

            }
         }
      }

      double successPercentage = successCount / handTargetPairsArray.size();
      if (thisTestIsStaticsBased)
      {
         if (successPercentage < minAcceptableSuccessPercentage)
         {
            fail(getClass().getSimpleName() + ": Only " + successPercentage * 100 + "% of test points were feasible according to the IK solver.");
         }
      }
   }

//   static private int count = 0;

  /* protected ArrayList<Pair<ReferenceFrame, ReferenceFrame>> createWallOfTargetPoints(RobotSide robotSide)
   {
      //Creates a wall of points in the front and center of the robot. The orientation of each target frame is as if the humanoid were performing the DRC wall task with the y-axis always pointing straight into the wall.
      double maxWallDepth = 0.2;
      double wallDistanceFromRobotCenter = 0.4;
      double maxWallWidth = 1.0;
      double maxWallHeight = 1.5;
      double minWallHeight = 0.9;
      int depthIncrements = 4;
      int halfOfWidthIncrements = 10;
      int heightIncrements = 10;
      int sign = (robotSide == RobotSide.RIGHT ? -1 : 1);
      for (int x = 0; x < depthIncrements; x++)
      {
         for (int y = -halfOfWidthIncrements; y < (halfOfWidthIncrements + 1); y++)
         {
            for (int z = 0; z < heightIncrements; z++)
            {
               Vector3d vector = new Vector3d(wallDistanceFromRobotCenter + x * maxWallDepth / depthIncrements, y * maxWallWidth / halfOfWidthIncrements,
                     minWallHeight + z * (maxWallHeight - minWallHeight) / heightIncrements);
               DenseMatrix64F matrix = new DenseMatrix64F(3, 3);
               matrix.set(3, 3, true, 0, -sign, 0, sign, 0, 0, 0, 0, 1);
               RigidBodyTransform transformToParent = new RigidBodyTransform(matrix, vector);
               ReferenceFrame desiredReferenceFrame = ReferenceFrame.constructFrameWithUnchangingTransformToParent("desiredFrame",
                     ReferenceFrame.getWorldFrame(), transformToParent);
               desiredReferenceFrame = moveDesiredFrameToRobotLocation(desiredReferenceFrame);
               Pair<ReferenceFrame, ReferenceFrame> handLeftFramesPair = new Pair<ReferenceFrame, ReferenceFrame>(desiredReferenceFrame, null);
               Pair<ReferenceFrame, ReferenceFrame> handRightFramesPair = new Pair<ReferenceFrame, ReferenceFrame>(null, desiredReferenceFrame);
               if (robotSide == RobotSide.RIGHT)
               {
                  handArrayList.add(handRightFramesPair);
               }
               else
               {
                  handArrayList.add(handLeftFramesPair);
               }
            }
         }
      }
      return handArrayList;
   }
*/
   

   private FramePose moveDesiredFrameToRobotLocation(final FramePose desiredPose)
   {
      RigidBodyTransform transformFromDesiredToWorld = new RigidBodyTransform();
      desiredPose.getRigidBodyTransform(transformFromDesiredToWorld);
      
      RigidBodyTransform transformFromRobotBaseToWorld = new RigidBodyTransform();
      transformFromRobotBaseToWorld.setTranslation(randomRobotPositionX, randomRobotPositionY, 0.0);
      
      Quat4d jointRotation = new Quat4d();
      RotationFunctions.setQuaternionBasedOnYawPitchRoll(jointRotation, randomRobotYaw, 0.0, 0.0);
      transformFromRobotBaseToWorld.setRotation(jointRotation);

      RigidBodyTransform transform = new RigidBodyTransform();
      transform.multiply( transformFromRobotBaseToWorld, transformFromDesiredToWorld);

      FramePose desiredToReturnFrame = new FramePose( ReferenceFrame.getWorldFrame(), transform);

      return desiredToReturnFrame;
   }
   
   /**
   public void testForwardKinematicsAndConsistencyOfURDF()
   {
      Random random = new Random();
      VectorXd Q = new VectorXd(hikSolver.getNumberOfJoints());

      // this map will tells us which joint are present both in hikSolver and fullRobotModel.
      HashMap<String, Integer> activeJoints = hikSolver.getHierarchicalSolver().createListOfActiveJoints();

      // set random position for each joint. Store this value both in hikSolver and fullRobotModel.
      for (Map.Entry<String, Integer> joint : activeJoints.entrySet())
      {
         String jointName = joint.getKey();
         int jointIndex = joint.getValue();
         double val = RandomTools.generateRandomDouble(random, -1.5, 1.5);
         actualRobotModel.getOneDoFJointByName(jointName).setQ(val);
         Q.set(jointIndex, val);
      }

      // update the forward kinematics of WB model
      hikSolver.getHierarchicalSolver().getForwardSolver().updateKinematics(Q);

      // move the pelvis to the same position to make the comparison in WorldFrame meaningful
      ReferenceFrame pelvisFrameAccordingToWB = hikSolver.getDesiredPelvisFrame(ReferenceFrame.getWorldFrame());
      actualRobotModel.getRootJoint().setPositionAndRotation(pelvisFrameAccordingToWB.getTransformToWorldFrame());

      // update the forward kinematics of the SDF model.
      actualRobotModel.updateFrames();

      RigidBodyTransform IDENTITY = new RigidBodyTransform();

      for (Map.Entry<String, Integer> joint : activeJoints.entrySet())
      {
         String jointName = joint.getKey();

         OneDoFJoint oneDoFJoint = actualRobotModel.getOneDoFJointByName(jointName);
         ReferenceFrame frame_sdf = oneDoFJoint.getFrameAfterJoint();

         //Sorry, this is not optimal. the after/before relationship of joints is 
         // reversed on the right leg used described in URDF.
         if (jointName.contains("r_leg_"))
         {
            frame_sdf = oneDoFJoint.getFrameBeforeJoint();
         }

         int bodyId = hikSolver.getIdOfBodyAfterJoint(jointName);

         ReferenceFrame frame_wb = hikSolver.getBodyReferenceFrame(bodyId);
         RigidBodyTransform difference = frame_sdf.getTransformToDesiredFrame(frame_wb);

         LogTools.debug(this, "-------- " + jointName + " ----------\n");

         if (difference.epsilonEquals(IDENTITY, 0.0001) == false || DEBUG)
         {
            System.out.println("SDF\n" + frame_sdf.getTransformToWorldFrame());
            System.out.println("WB\n" + frame_wb.getTransformToWorldFrame());
            System.out.println("DIFFERENCE\n" + difference);
         }

         assertTrue(difference.epsilonEquals(IDENTITY, 0.0001));
      }
   }
  */


 }

