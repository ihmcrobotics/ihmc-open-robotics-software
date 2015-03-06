package us.ihmc.wholeBodyController;

import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import java.util.ArrayList;
import java.util.Random;

import javax.vecmath.Matrix4d;
import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.SdfLoader.FullRobotModelVisualizer;
import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.graphics3DAdapter.graphics.appearances.AppearanceDefinition;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.utilities.Pair;
import us.ihmc.utilities.humanoidRobot.model.FullRobotModel;
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
   private final int NUMBER_OF_REGRESSION_POSES = 5;

   private SDFFullRobotModel actualRobotModel;
   private SDFFullRobotModel desiredRobotModel;
   
   private WholeBodyIkSolver hikSolver;
   private Robot robot;

   private WholeBodyHalfCylinderTargetParameters wholeBodyHalfCylinderTargetParameters;

   private final YoVariableRegistry registry = new YoVariableRegistry("WholeBodyIkSolverTestFactory_Registry");

   private final double maxDistanceFromOriginInCartesianDirection = 5.0;
   private double randomRobotPositionX;
   private double randomRobotPositionY;
   private double randomRobotYaw;
   static private final double minAcceptableSuccessPercentage = 0.95;

   private final double ERROR_TOLERANCE = 0.01;
   private final boolean DEBUG = false;

   private SimulationConstructionSet scs;
   private FullRobotModelVisualizer modelVisualizer;
   private BagOfBalls bagOfBalls;

   public WholeBodyIkSolverTestHelper(WholeBodyControllerParameters robotModel, SDFFullRobotModel actualRobotModel, WholeBodyIkSolver solver, Robot robot,
                                      WholeBodyHalfCylinderTargetParameters wholeBodyHalfCylinderTargetParameters)
   {
      this.actualRobotModel = actualRobotModel;
      this.desiredRobotModel = robotModel.createFullRobotModel();
      this.hikSolver = solver;
      this.robot = robot;
      this.wholeBodyHalfCylinderTargetParameters = wholeBodyHalfCylinderTargetParameters;
   }

   public void addGraphics(SimulationConstructionSet scs, FullRobotModelVisualizer modelVisualizer)
   {
      this.scs = scs;
      this.modelVisualizer = modelVisualizer;
      YoGraphicsListRegistry listRegistry = new YoGraphicsListRegistry();
      bagOfBalls = new BagOfBalls(50, 0.03, "targets", YoAppearance.Transparent(), registry, listRegistry);
      scs.addYoGraphicsListRegistry(listRegistry);
   }

   public void executeHandTargetTest(ControlledDoF dofToControlLeft, ControlledDoF dofToControlRight,
                                     ArrayList<Pair<FramePose, FramePose>> handTargetPairsArray, boolean thisTestIsStaticsBased)
   {
      executeHandTargetTest(dofToControlLeft, dofToControlRight, handTargetPairsArray, thisTestIsStaticsBased, true);
   }

   public void executeHandTargetTest(ControlledDoF dofToControlLeft, ControlledDoF dofToControlRight,
                                     ArrayList<Pair<FramePose, FramePose>> handTargetPairsArray, boolean thisTestIsStaticsBased,
                                     boolean useRandomRobotLocations)
   {
      Random random = new Random(100L);
      if (useRandomRobotLocations)
      {
         randomRobotPositionX = maxDistanceFromOriginInCartesianDirection * random.nextDouble();
         randomRobotPositionY = maxDistanceFromOriginInCartesianDirection * random.nextDouble();
         randomRobotYaw = 2.0 * Math.PI * random.nextDouble();
      }
      else
      {
         randomRobotPositionX = 0;
         randomRobotPositionY = 0;
         randomRobotYaw = 0;
      }

      // randomRobotPositionX = 0;
      // randomRobotPositionY = 0;
      // randomRobotYaw = 0;

      Vector3d rootPosition = new Vector3d(randomRobotPositionX, randomRobotPositionY, 0.93);

      actualRobotModel.getRootJoint().setPosition(rootPosition);
      actualRobotModel.getRootJoint().setRotation(randomRobotYaw, 0.0, 0.0);

      hikSolver.setVerbosityLevel((scs == null) ? 0 : 1);
      hikSolver.getHierarchicalSolver().collisionAvoidance.setEnabled(false);

      double successCount = 0;
      ArrayList<FramePose> framesToPlotAsBlueSpheres = new ArrayList<FramePose>();

      SideDependentList<ControlledDoF> dofToControl = new SideDependentList<ControlledDoF>();
      dofToControl.set(RobotSide.LEFT, dofToControlLeft);
      dofToControl.set(RobotSide.RIGHT, dofToControlRight);

      for (Pair<FramePose, FramePose> framePair : handTargetPairsArray)
      {
         SideDependentList<FramePose> endEffectorTarget = new SideDependentList<FramePose>();

         if (framePair.first() != null)
         {
            endEffectorTarget.set(RobotSide.LEFT, moveDesiredFrameToRobotLocation(framePair.first()));

            // endEffectorTargetLeft = endEffectorTarget.get(RobotSide.LEFT).getFramePointCopy();
         }

         if (framePair.second() != null)
         {
            endEffectorTarget.set(RobotSide.RIGHT, moveDesiredFrameToRobotLocation(framePair.second()));

            // endEffectorTargetLeft = endEffectorTarget.get(RobotSide.RIGHT).getFramePointCopy();
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

         if (DEBUG)
         {
            System.out.println(getClass().getSimpleName() + ": ret(" + hikSolutionExists + ") hik solution "
                               + ((hikSolutionExists == ComputeResult.SUCCEEDED) ? "found" : "not found"));
         }

         actualRobotModel.copyAllJointsButKeepOneFootFixed(desiredRobotModel.getOneDoFJoints(), RobotSide.RIGHT);
         actualRobotModel.updateFrames();

         if (scs != null)
         {
            AppearanceDefinition appearance = YoAppearance.Green();
            if (hikSolutionExists != ComputeResult.SUCCEEDED)
            {
               appearance = YoAppearance.Red();
            }

            for (RobotSide side : RobotSide.values)
            {
               if (endEffectorTarget.get(side) != null)
               {
                  bagOfBalls.setBallLoop(endEffectorTarget.get(side).getFramePointCopy(), appearance);
               }
            }

            modelVisualizer.update(0);

         }

         for (RobotSide side : RobotSide.values())
         {
            // you don't need to check if you are not trying to control any DoF
            if ((dofToControl.get(side) != ControlledDoF.DOF_NONE) && (endEffectorTarget.get(side) != null))
            {
               ReferenceFrame calculatedHandFrame = hikSolver.getDesiredGripperAttachmentFrame(side, ReferenceFrame.getWorldFrame());
               RigidBodyTransform calculatedAttachmnent = calculatedHandFrame.getTransformToWorldFrame();

               calculatedHandFrame = hikSolver.getDesiredGripperPalmFrame(side, ReferenceFrame.getWorldFrame());
               RigidBodyTransform calculatedPalm = calculatedHandFrame.getTransformToWorldFrame();

               RigidBodyTransform desiredAttachment = new RigidBodyTransform();
               endEffectorTarget.get(side).getRigidBodyTransform(desiredAttachment);

               RigidBodyTransform actualAttachment = desiredRobotModel.getHandControlFrame(side).getTransformToWorldFrame();

               Vector3d errorVector = RigidBodyTransform.getTranslationDifference(desiredAttachment, actualAttachment);

               if (DEBUG)
               {
                  System.out.println(" actual\n" + actualAttachment);
                  System.out.println(" wanted\n" + desiredAttachment);
                  System.out.println(" calculated Att\n" + calculatedAttachmnent);
                  System.out.println(" calculated Palm\n" + calculatedPalm);

                  System.out.println(" actual sole\n" + actualRobotModel.getSoleFrame(side).getTransformToWorldFrame());

                  System.out.println("\n desired (local B)\n" + hikSolver.taskEndEffectorPosition.get(side).getTarget());
                  System.out.println(" calculated (local)\n" + hikSolver.taskEndEffectorPosition.get(side).getCurrent() + "\n\n");
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

                     double error = hikSolver.calculateCenterOfMassError(desiredRobotModel);
                     double EPS = 0.03;    // 3 cm tolerance

                     if (error > EPS)
                     {
                        System.out.println(getClass().getSimpleName() + ": - ERROR: CoM doesnt match");
                     }

                     boolean isSelfColliding = hikSolver.checkCollisions(desiredRobotModel);

                     if (isSelfColliding)
                     {
                        System.out.println(getClass().getSimpleName() + ": - ERROR: Collisions detected");
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

   // static private int count = 0;

   public ArrayList<Pair<FramePose, FramePose>> createWallOfTargetPoints(RobotSide robotSide)
   {
      // Creates a wall of points in the front and center of the robot. The orientation of each target frame is as if the humanoid were performing the DRC wall task with the y-axis always pointing straight into the wall.
      double maxWallDepth = 0.2;
      double wallDistanceFromRobotCenter = 0.4;
      double maxWallWidth = 1.0;
      double maxWallHeight = 1.5;
      double minWallHeight = 0.9;
      int depthIncrements = 4;
      int halfOfWidthIncrements = 10;
      int heightIncrements = 10;
      int sign = ((robotSide == RobotSide.RIGHT) ? -1 : 1);
      ArrayList<Pair<FramePose, FramePose>> handArrayList = new ArrayList<Pair<FramePose, FramePose>>();
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
               FramePose desiredFramePose = new FramePose(desiredReferenceFrame);
               desiredFramePose = moveDesiredFrameToRobotLocation(desiredFramePose);
               Pair<FramePose, FramePose> handLeftFramesPair = new Pair<FramePose, FramePose>(desiredFramePose, null);
               Pair<FramePose, FramePose> handRightFramesPair = new Pair<FramePose, FramePose>(null, desiredFramePose);
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

   private FramePose moveDesiredFrameToRobotLocation(final FramePose desiredPose)
   {
      // Note RobotBase is directly on top of Old
      RigidBodyTransform transformFromDesiredToOldWorld = new RigidBodyTransform();
      desiredPose.getRigidBodyTransform(transformFromDesiredToOldWorld);

      RigidBodyTransform transformFromRobotBaseToNewWorld = new RigidBodyTransform();
      transformFromRobotBaseToNewWorld.setTranslation(randomRobotPositionX, randomRobotPositionY, 0.0);

      Quat4d robotRotation = new Quat4d();
      RotationFunctions.setQuaternionBasedOnYawPitchRoll(robotRotation, randomRobotYaw, 0.0, 0.0);
      transformFromRobotBaseToNewWorld.setRotation(robotRotation);

      RigidBodyTransform transform = new RigidBodyTransform();
      transform.multiply(transformFromRobotBaseToNewWorld, transformFromDesiredToOldWorld);

      FramePose desiredToReturnFrame = new FramePose(ReferenceFrame.getWorldFrame(), transform);

      return desiredToReturnFrame;
   }

   public FullRobotModel getActualFullRobotModel()
   {
      return actualRobotModel;
   }

   public Robot getRobot()
   {
      return robot;
   }

   public ArrayList<Pair<FramePose, FramePose>> createHalfCylinderOfTargetPoints(RobotSide robotSide)
   {
      double maxReachRadius = wholeBodyHalfCylinderTargetParameters.getMaxReachRadius();
      double maxHeight = wholeBodyHalfCylinderTargetParameters.getMaxHeight();
      double maxTheta = wholeBodyHalfCylinderTargetParameters.getMaxTheta();
      int radiusIncrements = wholeBodyHalfCylinderTargetParameters.getRadiusIncrements();
      int heightIncrements = wholeBodyHalfCylinderTargetParameters.getHeightIncrements();
      int thetaIncrements = wholeBodyHalfCylinderTargetParameters.getThetaIncrements();

      // Creates a half cylinder of points for the robot to try to reach in its (Right/Left) arm workspace.
      int sign = ((robotSide == RobotSide.RIGHT) ? -1 : 1);

      ArrayList<Pair<FramePose, FramePose>> handArrayList = new ArrayList<Pair<FramePose, FramePose>>();

      for (int z_int = heightIncrements; z_int > 0; z_int--)
      {
         for (int theta_int = 0; theta_int < 4; theta_int++)
         {
            for (int radius_int = 2; radius_int < (1 + radiusIncrements); radius_int++)
            {
               double height = maxHeight * z_int / heightIncrements;
               double reachRadius = maxReachRadius * radius_int / radiusIncrements;
               double theta = sign * maxTheta * theta_int / thetaIncrements;

               Point3d point = new Point3d(reachRadius * Math.cos(theta), reachRadius * Math.sin(theta), height);
               Quat4d noRotation = new Quat4d();

               FramePose desiredPose = new FramePose(ReferenceFrame.getWorldFrame(), point, noRotation);

               Pair<FramePose, FramePose> handLeftFramesPair = new Pair<FramePose, FramePose>(desiredPose, null);
               Pair<FramePose, FramePose> handRightFramesPair = new Pair<FramePose, FramePose>(null, desiredPose);

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


   public ArrayList<Pair<FramePose, FramePose>> createManualFramePosePairArrayListForOneHand(RobotSide robotSide,
           ArrayList<Matrix4d> handRigidBodyTransformData)
   {
      if (robotSide == RobotSide.LEFT)
         return createManualFramePosePairArrayList(handRigidBodyTransformData, null);
      else
         return createManualFramePosePairArrayList(null, handRigidBodyTransformData);
   }

   public ArrayList<Pair<FramePose, FramePose>> createManualFramePosePairArrayList(ArrayList<Matrix4d> leftHandRigidBodyTransformData,
           ArrayList<Matrix4d> rightHandRigidBodyTransformData)
   {
      ArrayList<Pair<FramePose, FramePose>> arrayListToReturn = new ArrayList<Pair<FramePose, FramePose>>();
      try
      {
         if ((leftHandRigidBodyTransformData != null) && (rightHandRigidBodyTransformData != null))
         {
            if (leftHandRigidBodyTransformData.size() != rightHandRigidBodyTransformData.size())
            {
               throw new Exception(getClass().getSimpleName() + ": left and right hand double arrays must be same length");
            }
         }
      }
      catch (Exception e)
      {
         e.printStackTrace();
      }

      int maxi = -1;
      if (leftHandRigidBodyTransformData != null)
      {
         maxi = leftHandRigidBodyTransformData.size();
      }
      else
      {
         maxi = rightHandRigidBodyTransformData.size();
      }

      FramePose desiredLeftHandFrame = null;
      FramePose desiredRightHandFrame = null;

      for (int i = 0; i < maxi; i++)
      {
         if (leftHandRigidBodyTransformData != null)
         {
            Matrix4d matrix = leftHandRigidBodyTransformData.get(i);
            RigidBodyTransform leftHandToWorldTransform = new RigidBodyTransform(matrix);
            desiredLeftHandFrame = new FramePose(ReferenceFrame.getWorldFrame(), leftHandToWorldTransform);

         }

         if (rightHandRigidBodyTransformData != null)
         {
            Matrix4d matrix = rightHandRigidBodyTransformData.get(i);
            RigidBodyTransform rightHandToWorldTransform = new RigidBodyTransform(matrix);
            desiredRightHandFrame = new FramePose(ReferenceFrame.getWorldFrame(), rightHandToWorldTransform);
         }

         Pair<FramePose, FramePose> pair = new Pair<FramePose, FramePose>(desiredLeftHandFrame, desiredRightHandFrame);
         arrayListToReturn.add(pair);
      }

      return arrayListToReturn;
   }


   public int getNumberOfRegressionPoses()
   {
      return NUMBER_OF_REGRESSION_POSES;
   }


   /**
    * public ArrayList<Pair<FramePose, FramePose>> generatePointsForRegression(int pointsDesired)
    * {
    *
    *  Vector3d rootPosition = new Vector3d(0.0, 0.0, 0.93);
    *  desiredRobotModel.getRootJoint().setPosition(rootPosition);
    *  desiredRobotModel.getRootJoint().setRotation(0.0, 0.0, 0.0);
    *
    *  SimulationConstructionSet scs = new SimulationConstructionSet(robotModel.createSdfRobot(false));
    *  FullRobotModelVisualizer localModelVisualizer = new FullRobotModelVisualizer(scs, desiredRobotModel, 0.01);
    *  if (VISUALIZE_RANDOMLY_GENERATED_POSES)
    *  {
    *     scs.startOnAThread();
    *     ThreadTools.sleep(2000);
    *  }
    *
    *  ArrayList<Pair<FramePose, FramePose>> handTargetArrayListToReturn = new ArrayList<Pair<FramePose, FramePose>>();
    *  Random random = new Random();
    *
    *  while (handTargetArrayListToReturn.size() < pointsDesired)
    *  {
    *     //         VectorXd Q = new VectorXd(hikSolver.getNumberOfJoints());
    *
    *     // this map will tells us which joint are present both in hikSolver and fullRobotModel.
    *     HashMap<String, Integer> activeJoints = hikSolver.getHierarchicalSolver().createListOfActiveJoints();
    *
    *     // set random position for each joint. Store this value both in fullRobotModel.
    *     for (Map.Entry<String, Integer> joint : activeJoints.entrySet())
    *     {
    *        String jointName = joint.getKey();
    *        //            int jointIndex = joint.getValue();
    *        double val = RandomTools.generateRandomDouble(random, -1.0, 1.0);
    *        desiredRobotModel.getOneDoFJointByName(jointName).setQ(val);
    *        //            Q.set(jointIndex, val);
    *     }
    *
    *     // update the forward kinematics of WB model
    *     //         hikSolver.getHierarchicalSolver().getForwardSolver().updateKinematics(Q);
    *
    *     // update the forward kinematics of the SDF model.
    *     desiredRobotModel.updateFrames();
    *     desiredRobotModel.copyAllJointsButKeepOneFootFixed(desiredRobotModel.getOneDoFJoints(), RobotSide.RIGHT);
    *     //         hikSolver.updateWorkingModel(actualRobotModel);
    *
    *     double error = hikSolver.calculateCenterOfMassError(desiredRobotModel);
    *     double EPS = 0.01; // 1 cm tolerance
    *
    *     boolean isSelfColliding = hikSolver.checkCollisions(desiredRobotModel);
    *
    *     //if CoM is close enough to stable and there are no self collisions then accept the proposed pose as feasible and add it to the list of targets to try with the IK solver.
    *     if (error < EPS & !isSelfColliding)
    *     {
    *        Vector3d vector = new Vector3d();
    *        Quat4d noRotation = new Quat4d();
    *
    *        ReferenceFrame leftHandFrameDesired = desiredRobotModel.getHandControlFrame(RobotSide.LEFT);
    *        ReferenceFrame rightHandFrameDesired = desiredRobotModel.getHandControlFrame(RobotSide.RIGHT);
    *
    *        leftHandFrameDesired.getTransformToWorldFrame().getTranslation(vector);
    *        Point3d point = new Point3d(vector);
    *        FramePose leftHandTarget = new FramePose(ReferenceFrame.getWorldFrame(), point, noRotation);
    *
    *        rightHandFrameDesired.getTransformToWorldFrame().getTranslation(vector);
    *        point = new Point3d(vector);
    *        FramePose rightHandTarget = new FramePose(ReferenceFrame.getWorldFrame(), point, noRotation);
    *
    *        Pair<FramePose, FramePose> pairToPack = new Pair<FramePose, FramePose>(leftHandTarget, rightHandTarget);
    *        handTargetArrayListToReturn.add(pairToPack);
    *        if (VISUALIZE_RANDOMLY_GENERATED_POSES)
    *        {
    *           localModelVisualizer.update(0);
    *           ThreadTools.sleep(100);
    *        }
    *     }
    *  }
    *  ThreadTools.sleep(1200000);
    *  return handTargetArrayListToReturn;
    * }
    *
    * /** public void testForwardKinematicsAndConsistencyOfURDF()
    * {
    *  Random random = new Random();
    *  VectorXd Q = new VectorXd(hikSolver.getNumberOfJoints());
    *
    *  // this map will tells us which joint are present both in hikSolver and fullRobotModel.
    *  HashMap<String, Integer> activeJoints = hikSolver.getHierarchicalSolver().createListOfActiveJoints();
    *
    *  // set random position for each joint. Store this value both in hikSolver and fullRobotModel.
    *  for (Map.Entry<String, Integer> joint : activeJoints.entrySet())
    *  {
    *     String jointName = joint.getKey();
    *     int jointIndex = joint.getValue();
    *     double val = RandomTools.generateRandomDouble(random, -1.5, 1.5);
    *     actualRobotModel.getOneDoFJointByName(jointName).setQ(val);
    *     Q.set(jointIndex, val);
    *  }
    *
    *  // update the forward kinematics of WB model
    *  hikSolver.getHierarchicalSolver().getForwardSolver().updateKinematics(Q);
    *
    *  // move the pelvis to the same position to make the comparison in WorldFrame meaningful
    *  ReferenceFrame pelvisFrameAccordingToWB = hikSolver.getDesiredPelvisFrame(ReferenceFrame.getWorldFrame());
    *  actualRobotModel.getRootJoint().setPositionAndRotation(pelvisFrameAccordingToWB.getTransformToWorldFrame());
    *
    *  // update the forward kinematics of the SDF model.
    *  actualRobotModel.updateFrames();
    *
    *  RigidBodyTransform IDENTITY = new RigidBodyTransform();
    *
    *  for (Map.Entry<String, Integer> joint : activeJoints.entrySet())
    *  {
    *     String jointName = joint.getKey();
    *
    *     OneDoFJoint oneDoFJoint = actualRobotModel.getOneDoFJointByName(jointName);
    *     ReferenceFrame frame_sdf = oneDoFJoint.getFrameAfterJoint();
    *
    *     //Sorry, this is not optimal. the after/before relationship of joints is
    *     // reversed on the right leg used described in URDF.
    *     if (jointName.contains("r_leg_"))
    *     {
    *        frame_sdf = oneDoFJoint.getFrameBeforeJoint();
    *     }
    *
    *     int bodyId = hikSolver.getIdOfBodyAfterJoint(jointName);
    *
    *     ReferenceFrame frame_wb = hikSolver.getBodyReferenceFrame(bodyId);
    *     RigidBodyTransform difference = frame_sdf.getTransformToDesiredFrame(frame_wb);
    *
    *     LogTools.debug(this, "-------- " + jointName + " ----------\n");
    *
    *     if (difference.epsilonEquals(IDENTITY, 0.0001) == false || DEBUG)
    *     {
    *        System.out.println("SDF\n" + frame_sdf.getTransformToWorldFrame());
    *        System.out.println("WB\n" + frame_wb.getTransformToWorldFrame());
    *        System.out.println("DIFFERENCE\n" + difference);
    *     }
    *
    *     assertTrue(difference.epsilonEquals(IDENTITY, 0.0001));
    *  }
    * }
    */

   /**
    * Insert this code at around line 210 in WholeBodyIkDevelopmentPanel to collect manual points. Launch the AtlasObstacleCourseDemo in debug mode and hit pause every
    * time you have a pose that you want to copy paste from the console.
    */

   /**
    * boolean willNeedsToCollectSomeManualPointsForIkTests = true;
    * if(willNeedsToCollectSomeManualPointsForIkTests){
    *
    * ReferenceFrame fHandWorld = wholeBodyIK.getDesiredGripperAttachmentFrame(side, ReferenceFrame.getWorldFrame() );
    * ReferenceFrame fHandLocal = wholeBodyIK.getDesiredGripperAttachmentFrame(side, wholeBodyIK.getRootFrame() );
    *
    * RigidBodyTransform rBTWorld = fHandWorld.getTransformToParent();
    * RigidBodyTransform rBTLocal = fHandLocal.getTransformToParent();
    *
    * Matrix4d matrixWorld  = new Matrix4d();
    * Matrix4d matrixLocal  = new Matrix4d();
    *
    * rBTWorld.get(matrixWorld);
    * rBTLocal.get(matrixLocal);
    *
    * Vector4d v1 = new Vector4d(), v2 = new Vector4d(), v3 = new Vector4d(), v4 = new Vector4d();
    *
    * matrixWorld.getRow(0, v1);
    * matrixWorld.getRow(1, v2);
    * matrixWorld.getRow(2, v3);
    * matrixWorld.getRow(3, v4);
    *
    * System.out.println((side==RobotSide.RIGHT ? "RightHand":"LeftHand") +"ToWorldArray.add(new Matrix4d" + v1.toString() + v2.toString() + v3.toString() + v4.toString());
    *
    * matrixLocal.getRow(0, v1);
    * matrixLocal.getRow(1, v2);
    * matrixLocal.getRow(2, v3);
    * matrixLocal.getRow(3, v4);
    *
    * System.out.println((side==RobotSide.RIGHT ? "RightHand":"LeftHand") +"ToFootArray.add(new Matrix4d" + v1.toString() + v2.toString() + v3.toString() + v4.toString());
    *
    * }
    */

}
