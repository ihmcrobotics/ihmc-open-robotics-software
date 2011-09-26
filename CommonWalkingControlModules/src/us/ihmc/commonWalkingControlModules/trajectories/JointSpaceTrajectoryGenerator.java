package us.ihmc.commonWalkingControlModules.trajectories;

import javax.media.j3d.Transform3D;
import javax.vecmath.Matrix3d;
import javax.vecmath.Vector3d;

import us.ihmc.commonWalkingControlModules.kinematics.InverseKinematicsException;
import us.ihmc.commonWalkingControlModules.kinematics.LegInverseKinematicsCalculator;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.LegJointAccelerations;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.LegJointName;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.LegJointPositions;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.LegJointVelocities;
import us.ihmc.commonWalkingControlModules.referenceFrames.CommonWalkingReferenceFrames;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.robotSide.SideDependentList;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.Orientation;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.splines.QuinticSplineInterpolator;

public class JointSpaceTrajectoryGenerator
{
   private final YoVariableRegistry registry;
   private final QuinticSplineInterpolator hipYawSpline;
   private final QuinticSplineInterpolator[] jointSplines;

   private final SideDependentList<FramePoint[]> viaPoints;

   private final GroundTrajectoryGenerator groundTractory;
   private final int numberOfViaPoints;

   private final DoubleYoVariable[] heightOfViaPoints;

   private final LegInverseKinematicsCalculator inverseKinematicsCalculator;

   private final CommonWalkingReferenceFrames referenceFrames;
   private final Orientation finalOrientation;
   private final Orientation intermediateOrientation;
   private final SideDependentList<LegJointPositions> intermediatePositions;
   
   private final double controlDT;
   
   double[][] yawSplineResult = new double[1][3];
   double[][] jointSplineResult = new double[5][3];
   
   private final LegJointName[] arrayOfLegJointNames = new LegJointName[] { LegJointName.HIP_PITCH, LegJointName.HIP_ROLL, LegJointName.KNEE, LegJointName.ANKLE_PITCH, LegJointName.ANKLE_ROLL }; 

   public JointSpaceTrajectoryGenerator(String name, int numberOfViaPoints, CommonWalkingReferenceFrames referenceFrames,
         LegInverseKinematicsCalculator inverseKinematicsCalculator, double controlDT, YoVariableRegistry parentRegistry)
   {
      registry = new YoVariableRegistry(name);
      this.numberOfViaPoints = numberOfViaPoints;
      this.referenceFrames = referenceFrames;

      hipYawSpline = new QuinticSplineInterpolator("hipYawSpline", 2, 1, registry);
      jointSplines = new QuinticSplineInterpolator[numberOfViaPoints + 1];
      for (int i = 0; i <= numberOfViaPoints; i++)
      {
         jointSplines[i] = new QuinticSplineInterpolator("jointSpline[" + i + "]", i + 2, 5, registry);
      }

      if (numberOfViaPoints > 0)
      {
         groundTractory = new ParameterizedGroundTrajectoryGenerator("groundTrajectory", referenceFrames);
         viaPoints = new SideDependentList<FramePoint[]>();
         heightOfViaPoints = new DoubleYoVariable[numberOfViaPoints];

         for (RobotSide side : RobotSide.values())
         {
            viaPoints.set(side, new FramePoint[numberOfViaPoints]);
         }

         for (int i = 0; i < numberOfViaPoints; i++)
         {
            for (RobotSide side : RobotSide.values())
            {
               viaPoints.get(side)[i] = new FramePoint(referenceFrames.getAnkleZUpFrame(side));
            }

            heightOfViaPoints[i] = new DoubleYoVariable("swingHeight[" + i + "]", registry);
            heightOfViaPoints[i].set(0.10);
         }
      } else
      {
         groundTractory = null;
         viaPoints = null;
         heightOfViaPoints = null;
      }

      this.inverseKinematicsCalculator = inverseKinematicsCalculator;
      finalOrientation = new Orientation(referenceFrames.getPelvisFrame());
      intermediateOrientation = new Orientation(referenceFrames.getPelvisFrame());
      intermediatePositions = new SideDependentList<LegJointPositions>();
      for (RobotSide side : RobotSide.values())
      {
         intermediatePositions.set(side, new LegJointPositions(side));
      }
      
      this.controlDT = controlDT;
      parentRegistry.addChild(registry);
   }

   public void compute(LegJointPositions jointAnglesToPack, LegJointVelocities jointVelocitiesToPack, LegJointAccelerations jointAccelerationsToPack, LegJointPositions currentJointPositions,
         LegJointVelocities currentJointVelocities, LegJointAccelerations currentJointAccelerations, double swingDuration, double timeInSwing,
         FramePoint currentSwingFootPosition, FrameVector currentSwingFootVelocity, FramePoint desiredFinalPosition, Orientation finalOrientationIn)
   {
      RobotSide swingSide = currentJointPositions.getRobotSide();
      finalOrientation.setIncludingFrame(finalOrientationIn);
      finalOrientation.changeFrame(referenceFrames.getPelvisFrame());

      double timePerPoint = (swingDuration / ((double) (numberOfViaPoints + 1)));
      int pointsInSwing = (int) (timeInSwing / timePerPoint);
      int numberOfPointsRemaining = numberOfViaPoints - pointsInSwing;
      double[] t = new double[numberOfPointsRemaining + 2];
      t[0] = timeInSwing;

      double[] tOfViaPoints;

      if (numberOfPointsRemaining > 0)
      {
         tOfViaPoints = new double[numberOfPointsRemaining];
         double[] heightOfViaPointsDouble = new double[numberOfPointsRemaining];
         for (int i = 1; i <= numberOfPointsRemaining; i++)
         {
            tOfViaPoints[numberOfPointsRemaining - i] = swingDuration - ((double) i) * timePerPoint;
            heightOfViaPointsDouble[numberOfPointsRemaining - i] = heightOfViaPoints[numberOfViaPoints - i].getDoubleValue();
         }

         for (int i = 0; i < numberOfPointsRemaining; i++)
         {
            t[i + 1] = tOfViaPoints[i];
         }

         groundTractory.getViaPoints(viaPoints.get(swingSide), swingSide, timeInSwing, currentSwingFootPosition, swingDuration, desiredFinalPosition,
               currentSwingFootVelocity, tOfViaPoints, heightOfViaPointsDouble);
      } else
      {
         tOfViaPoints = null;
      }

      t[numberOfPointsRemaining + 1] = swingDuration;

      double currentYaw = currentJointPositions.getJointPosition(LegJointName.HIP_YAW);
      double finalYaw = finalOrientation.getYawPitchRoll()[0];

      double yawIn[] = new double[] { currentYaw, finalYaw };
      double tYaw[] = new double[] { timeInSwing, swingDuration };
      hipYawSpline.initialize(tYaw);
      hipYawSpline.determineCoefficients(0, yawIn, currentJointVelocities.getJointVelocity(LegJointName.HIP_YAW), 0.0,
            currentJointAccelerations.getJointAcceleration(LegJointName.HIP_YAW), 0.0);

      double[][] yIn = new double[5][numberOfPointsRemaining + 2];
      
      computePointOnSpline(swingSide, 0, timeInSwing, yIn);
      for (int i = 0; i < numberOfPointsRemaining; i++)
      {
         computePointOnSpline(swingSide, i+1, tOfViaPoints[i], yIn);
      }
      computePointOnSpline(swingSide, numberOfPointsRemaining+1, swingDuration, yIn);
      
      QuinticSplineInterpolator currentInterpolator = jointSplines[numberOfPointsRemaining];
      currentInterpolator.initialize(t);
      
      for (int j = 0; j < arrayOfLegJointNames.length; j++)
      {
         currentInterpolator.determineCoefficients(0, yIn[j], currentJointVelocities.getJointVelocity(arrayOfLegJointNames[j]), 0.0, currentJointAccelerations.getJointAcceleration(arrayOfLegJointNames[j]), 0.0);
      }
      
      
      currentInterpolator.compute(timeInSwing+controlDT, 2, jointSplineResult);
      hipYawSpline.compute(timeInSwing+controlDT, 2, yawSplineResult);
      
      jointAnglesToPack.setJointPosition(LegJointName.HIP_YAW, yawSplineResult[0][0]);
      jointVelocitiesToPack.setJointVelocity(LegJointName.HIP_YAW, yawSplineResult[0][1]);
      jointAccelerationsToPack.setJointAcceleration(LegJointName.HIP_YAW, yawSplineResult[0][2]);
      
      for (int j = 0; j < arrayOfLegJointNames.length; j++)
      {
         jointAnglesToPack.setJointPosition(arrayOfLegJointNames[j], jointSplineResult[j][0]);
         jointVelocitiesToPack.setJointVelocity(arrayOfLegJointNames[j], yawSplineResult[0][1]);
         jointAccelerationsToPack.setJointAcceleration(arrayOfLegJointNames[j], yawSplineResult[0][2]);
      }
      
      
   }
   
   private void computePointOnSpline(RobotSide swingSide, int i, double t, double[][] yIn)
   {
      double[][] yawResult = new double[1][1];
      hipYawSpline.compute(t, 0, yawResult);
      intermediateOrientation.setYawPitchRoll(yawResult[0][0], 0.0, 0.0);
      Transform3D footToPelvis = computeDesiredTransform(referenceFrames.getPelvisFrame(), viaPoints.get(swingSide)[i], intermediateOrientation);

      try
      {
         inverseKinematicsCalculator.solve(intermediatePositions.get(swingSide), footToPelvis, swingSide, yawResult[0][0]);
      } catch (InverseKinematicsException e)
      {
         System.err.println("Cannot computer joint angles");
      }

      for (int j = 0; j < arrayOfLegJointNames.length; j++)
      {
         yIn[j][i] = intermediatePositions.get(swingSide).getJointPosition(arrayOfLegJointNames[i]);
      }
         
         
      
   }

   private Transform3D computeDesiredTransform(ReferenceFrame pelvisFrame, FramePoint desiredFootPosition, Orientation desiredFootOrientation)
   {
      desiredFootOrientation.changeFrame(pelvisFrame);
      desiredFootPosition.changeFrame(pelvisFrame);
      Transform3D footToPelvis = createTransform(desiredFootOrientation, desiredFootPosition);

      return footToPelvis;
   }

   private static Transform3D createTransform(Orientation orientation, FramePoint framePoint)
   {
      orientation.checkReferenceFrameMatch(framePoint);
      Matrix3d rotationMatrix = orientation.getMatrix3d();
      Transform3D ret = new Transform3D(rotationMatrix, new Vector3d(framePoint.getPoint()), 1.0);

      return ret;
   }

}
