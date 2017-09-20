package us.ihmc.wholeBodyController;

public abstract class WholeBodyValidPoseGenerator
{
//   public void executeHandTargetTest(ControlledDoF dofToControlLeft, ControlledDoF dofToControlRight,
//         ArrayList<Pair<FramePose, FramePose>> handTargetPairsArray, boolean thisTestIsStaticsBased, boolean useRandomRobotLocations)
//   {
//      Random random = new Random();
//      if (useRandomRobotLocations)
//      {
//         randomRobotPositionX = maxDistanceFromOriginInCartesianDirection * random.nextDouble();
//         randomRobotPositionY = maxDistanceFromOriginInCartesianDirection * random.nextDouble();
//         randomRobotYaw = 2.0 * Math.PI * random.nextDouble();
//      }
//      else
//      {
//         randomRobotPositionX = 0;
//         randomRobotPositionY = 0;
//         randomRobotYaw = 0;
//      }
//
//      //  randomRobotPositionX = 0;
//      // randomRobotPositionY = 0;
//      //  randomRobotYaw = 0;
//
//      Vector3d rootPosition = new Vector3d(randomRobotPositionX, randomRobotPositionY, 0.93);
//
//      actualRobotModel.getRootJoint().setPosition(rootPosition);
//      actualRobotModel.getRootJoint().setRotation(randomRobotYaw, 0.0, 0.0);
//
//      hikSolver.setVerbosityLevel(scs == null ? 0 : 1);
//      hikSolver.getHierarchicalSolver().collisionAvoidance.setEnabled(false);
//
//      double successCount = 0;
//      ArrayList<FramePose> framesToPlotAsBlueSpheres = new ArrayList<FramePose>();
//
//      SideDependentList<ControlledDoF> dofToControl = new SideDependentList<ControlledDoF>();
//      dofToControl.set(RobotSide.LEFT, dofToControlLeft);
//      dofToControl.set(RobotSide.RIGHT, dofToControlRight);
//
//      for (Pair<FramePose, FramePose> framePair : handTargetPairsArray)
//      {
//         SideDependentList<FramePose> endEffectorTarget = new SideDependentList<FramePose>();
//
//         if (framePair.first() != null)
//         {
//            endEffectorTarget.set(RobotSide.LEFT, moveDesiredFrameToRobotLocation(framePair.first()));
//            //  endEffectorTargetLeft = endEffectorTarget.get(RobotSide.LEFT).getFramePointCopy();
//         }
//
//         if (framePair.second() != null)
//         {
//            endEffectorTarget.set(RobotSide.RIGHT, moveDesiredFrameToRobotLocation(framePair.second()));
//            //endEffectorTargetLeft = endEffectorTarget.get(RobotSide.RIGHT).getFramePointCopy();
//         }
//
//         actualRobotModel.updateFrames();
//
//         for (RobotSide robotSide : RobotSide.values())
//         {
//            hikSolver.setNumberOfControlledDoF(robotSide, dofToControl.get(robotSide));
//            if (endEffectorTarget.get(robotSide) != null)
//            {
//               hikSolver.setGripperAttachmentTarget(actualRobotModel, robotSide, endEffectorTarget.get(robotSide));
//            }
//         }
//
//         ComputeResult hikSolutionExists = ComputeResult.FAILED_INVALID;
//         try
//         {
//            hikSolutionExists = hikSolver.compute(actualRobotModel, desiredRobotModel, ComputeOption.USE_ACTUAL_MODEL_JOINTS);
//         }
//         catch (Exception e)
//         {
//            e.printStackTrace();
//         }
//
//         if (DEBUG)
//         {
//            System.out.println(getClass().getSimpleName() + ": ret(" + hikSolutionExists + ") hik solution "
//                  + (hikSolutionExists == ComputeResult.SUCCEEDED ? "found" : "not found"));
//         }
//
//         actualRobotModel.copyAllJointsButKeepOneFootFixed(desiredRobotModel.getOneDoFJoints(), RobotSide.RIGHT);
//         actualRobotModel.updateFrames();
//
//         if (scs != null)
//         {
//            AppearanceDefinition appearance = YoAppearance.Green();
//            if (hikSolutionExists != ComputeResult.SUCCEEDED)
//            {
//               appearance = YoAppearance.Red();
//            }
//
//            for (RobotSide side : RobotSide.values)
//            {
//               if (endEffectorTarget.get(side) != null)
//               {
//                  bagOfBalls.setBallLoop(endEffectorTarget.get(side).getFramePointCopy(), appearance);
//               }
//            }
//
//            modelVisualizer.update(0);
//
//         }
//
//         for (RobotSide side : RobotSide.values())
//         {
//            // you don't need to check if you are not trying to control any DoF
//            if (dofToControl.get(side) != ControlledDoF.DOF_NONE && endEffectorTarget.get(side) != null)
//            {
//               ReferenceFrame calculatedHandFrame = hikSolver.getDesiredGripperAttachmentFrame(side, ReferenceFrame.getWorldFrame());
//               RigidBodyTransform calculatedAttachmnent = calculatedHandFrame.getTransformToWorldFrame();
//
//               calculatedHandFrame = hikSolver.getDesiredGripperPalmFrame(side, ReferenceFrame.getWorldFrame());
//               RigidBodyTransform calculatedPalm = calculatedHandFrame.getTransformToWorldFrame();
//
//               RigidBodyTransform desiredAttachment = new RigidBodyTransform();
//               endEffectorTarget.get(side).getRigidBodyTransform(desiredAttachment);
//
//               RigidBodyTransform actualAttachment = desiredRobotModel.getHandControlFrame(side).getTransformToWorldFrame();
//
//               Vector3d errorVector = RigidBodyTransform.getTranslationDifference(desiredAttachment, actualAttachment);
//
//               if (DEBUG)
//               {
//                  System.out.println(" actual\n" + actualAttachment);
//                  System.out.println(" wanted\n" + desiredAttachment);
//                  System.out.println(" calculated Att\n" + calculatedAttachmnent);
//                  System.out.println(" calculated Palm\n" + calculatedPalm);
//
//                  System.out.println(" actual sole\n" + actualRobotModel.getSoleFrame(side).getTransformToWorldFrame());
//
//                  System.out.println("\n desired (local B)\n" + hikSolver.taskEndEffectorPosition.get(side).getTarget());
//                  System.out.println(" calculated (local)\n" + hikSolver.taskEndEffectorPosition.get(side).getCurrent() + "\n\n");
//               }
//
//               if (hikSolutionExists == ComputeResult.SUCCEEDED)
//               {
//                  if (DEBUG)
//                  {
//                     if (errorVector.length() > ERROR_TOLERANCE)
//                     {
//                        System.out.println(getClass().getSimpleName() + ": [" + side + "] FAILURE - error vector is \n" + errorVector.toString()
//                              + "\n length is " + errorVector.length() + " m");
//                     }
//                     else
//                     {
//                        System.out.println(getClass().getSimpleName() + ": [" + side + "] SUCCESS");
//                     }
//
//                     double error = hikSolver.calculateCenterOfMassError(desiredRobotModel);
//                     double EPS = 0.03; // 3 cm tolerance
//
//                     if (error > EPS)
//                     {
//                        System.out.println(getClass().getSimpleName() + ": - ERROR: CoM doesnt match");
//                     }
//
//                     boolean isSelfColliding = hikSolver.checkCollisions(desiredRobotModel);
//
//                     if (isSelfColliding)
//                     {
//                        System.out.println(getClass().getSimpleName() + ": - ERROR: Collisions detected");
//                     }
//                  }
//                  assertTrue(errorVector.length() < ERROR_TOLERANCE);
//                  framesToPlotAsBlueSpheres.add(endEffectorTarget.get(side));
//                  successCount++;
//
//               }
//               else
//               {
//                  if (!thisTestIsStaticsBased)
//                  {
//                     fail(getClass().getSimpleName() + ": hik solution not found");
//                  }
//               }
//
//            }
//         }
//      }
//
//      double successPercentage = successCount / handTargetPairsArray.size();
//      if (thisTestIsStaticsBased)
//      {
//         if (successPercentage < minAcceptableSuccessPercentage)
//         {
//            fail(getClass().getSimpleName() + ": Only " + successPercentage * 100 + "% of test points were feasible according to the IK solver.");
//         }
//      }
//   }

}
