package us.ihmc.wholeBodyController;

import static org.junit.Assert.fail;

import java.util.ArrayList;
import java.util.Random;

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
import org.apache.commons.lang3.tuple.ImmutablePair;
import us.ihmc.humanoidRobotics.model.FullRobotModel;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
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
   static private final double minAcceptableSuccessPercentage = 0.90;

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
      bagOfBalls = new BagOfBalls(1000, 0.03, "targets", YoAppearance.Transparent(), registry, listRegistry);
      scs.addYoGraphicsListRegistry(listRegistry);
   }

   public void executeHandTargetTest(ControlledDoF dofToControlLeft, ControlledDoF dofToControlRight,
                                     ArrayList<ImmutablePair<FramePose, FramePose>> handTargetPairsArray, boolean thisTestIsStaticsBased)
   {
      executeHandTargetTest(dofToControlLeft, dofToControlRight, handTargetPairsArray, thisTestIsStaticsBased, true);
   }

   public void executeHandTargetTest(ControlledDoF dofToControlLeft, 
                                     ControlledDoF dofToControlRight,
                                     ArrayList<ImmutablePair<FramePose, FramePose>> handTargetPairsArray, boolean thisTestIsStaticsBased,
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

      for (ImmutablePair<FramePose, FramePose> framePair : handTargetPairsArray)
      {
         SideDependentList<FramePose> endEffectorTarget = new SideDependentList<FramePose>();

         if (framePair.getLeft() != null)
         {
            endEffectorTarget.set(RobotSide.LEFT, framePair.getLeft());
         }

         if (framePair.getRight() != null)
         {
            endEffectorTarget.set(RobotSide.RIGHT, framePair.getRight());
         }

         actualRobotModel.updateFrames();

         for (RobotSide robotSide : RobotSide.values())
         {
            hikSolver.getConfiguration().setNumberOfControlledDoF(robotSide, dofToControl.get(robotSide));

            if (endEffectorTarget.get(robotSide) != null)
            {
               hikSolver.setGripperPalmTarget( robotSide, endEffectorTarget.get(robotSide));
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
               if (endEffectorTarget.get(side) != null && dofToControl.get(side) !=  ControlledDoF.DOF_NONE)
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
               ReferenceFrame calculatedHandFrame = hikSolver.getDesiredGripperPalmFrame(side, ReferenceFrame.getWorldFrame());
               RigidBodyTransform calculatedPalm = calculatedHandFrame.getTransformToWorldFrame();

               RigidBodyTransform desiredPalm = new RigidBodyTransform();
               endEffectorTarget.get(side).getRigidBodyTransform(desiredPalm);

               RigidBodyTransform actualPalm = desiredRobotModel.getHandControlFrame(side).getTransformToWorldFrame();

               Vector3d errorVector = RigidBodyTransform.getTranslationDifference(desiredPalm, actualPalm);

               if (DEBUG)
               {
                  System.out.println(" actual\n" + actualPalm);
                  System.out.println(" wanted\n" + desiredPalm);
                  System.out.println(" calculated Att\n" + calculatedPalm);

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

                 //FIXME assertTrue(errorVector.length() < ERROR_TOLERANCE);
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

   public ArrayList<ImmutablePair<FramePose, FramePose>> createWallOfTargetPoints(RobotSide robotSide)
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
      ArrayList<ImmutablePair<FramePose, FramePose>> handArrayList = new ArrayList<ImmutablePair<FramePose, FramePose>>();
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

               ImmutablePair<FramePose, FramePose> handLeftFramesPair = new ImmutablePair<FramePose, FramePose>(desiredFramePose, null);
               ImmutablePair<FramePose, FramePose> handRightFramesPair = new ImmutablePair<FramePose, FramePose>(null, desiredFramePose);
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

   public FullRobotModel getActualFullRobotModel()
   {
      return actualRobotModel;
   }

   public Robot getRobot()
   {
      return robot;
   }

   public ArrayList<ImmutablePair<FramePose, FramePose>> createHalfCylinderOfTargetPoints(RobotSide robotSide)
   {
      double maxReachRadius = wholeBodyHalfCylinderTargetParameters.getMaxReachRadius();
      double maxHeight = wholeBodyHalfCylinderTargetParameters.getMaxHeight();
      double maxTheta = wholeBodyHalfCylinderTargetParameters.getMaxTheta();
      int radiusIncrements = wholeBodyHalfCylinderTargetParameters.getRadiusIncrements();
      int heightIncrements = wholeBodyHalfCylinderTargetParameters.getHeightIncrements();
      int thetaIncrements = wholeBodyHalfCylinderTargetParameters.getThetaIncrements();

      // Creates a half cylinder of points for the robot to try to reach in its (Right/Left) arm workspace.
      int sign = ((robotSide == RobotSide.RIGHT) ? -1 : 1);

      ArrayList<ImmutablePair<FramePose, FramePose>> handArrayList = new ArrayList<ImmutablePair<FramePose, FramePose>>();

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

               FramePose desiredPose = new FramePose( hikSolver.getRootFrame(actualRobotModel) , point, noRotation);

               ImmutablePair<FramePose, FramePose> handLeftFramesPair = new ImmutablePair<FramePose, FramePose>(desiredPose, null);
               ImmutablePair<FramePose, FramePose> handRightFramesPair = new ImmutablePair<FramePose, FramePose>(null, desiredPose);

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


   public ArrayList<ImmutablePair<FramePose, FramePose>> createManualFramePosePairArrayList(
         ArrayList<SideDependentList<RigidBodyTransform>> handRigidBodyTransformData )
   {
      ArrayList<ImmutablePair<FramePose, FramePose>> arrayListToReturn = new ArrayList<ImmutablePair<FramePose, FramePose>>();

      FramePose desiredLeftHandFrame = null;
      FramePose desiredRightHandFrame = null;

      for (SideDependentList<RigidBodyTransform> targetPair: handRigidBodyTransformData )
      {
         desiredLeftHandFrame  = new FramePose( hikSolver.getRootFrame( actualRobotModel ), targetPair.get(RobotSide.LEFT)  );
         desiredRightHandFrame = new FramePose( hikSolver.getRootFrame( actualRobotModel ), targetPair.get(RobotSide.RIGHT) );

         ImmutablePair<FramePose, FramePose> pair = new ImmutablePair<FramePose, FramePose>(desiredLeftHandFrame, desiredRightHandFrame);
         arrayListToReturn.add(pair);
      }

      return arrayListToReturn;
   }


   public int getNumberOfRegressionPoses()
   {
      return NUMBER_OF_REGRESSION_POSES;
   }


}
