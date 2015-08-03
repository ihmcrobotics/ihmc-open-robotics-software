package us.ihmc.darpaRoboticsChallenge.reachabilityMapCalculator;

import java.awt.Color;
import java.util.ArrayList;
import java.util.Random;

import javax.vecmath.AxisAngle4d;
import javax.vecmath.Matrix3d;
import javax.vecmath.Vector3d;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.darpaRoboticsChallenge.reachabilityMapCalculator.voxelPrimitiveShapes.SphereVoxelShape;
import us.ihmc.darpaRoboticsChallenge.reachabilityMapCalculator.voxelPrimitiveShapes.SphereVoxelShape.SphereVoxelType;
import us.ihmc.graphics3DAdapter.graphics.Graphics3DObject;
import us.ihmc.graphics3DAdapter.graphics.appearances.AppearanceDefinition;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.robotics.kinematics.NumericalInverseKinematicsCalculator;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.ReferenceFrame;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.screwTheory.GeometricJacobian;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.ScrewTestTools;
import us.ihmc.robotics.screwTheory.SpatialMotionVector;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;

public class ReachabilitySphereMapCalculator
{
   private static final boolean DEBUG = false;
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final NumericalInverseKinematicsCalculator spatialInverseKinematicsCalculator;
   private final NumericalInverseKinematicsCalculator linearInverseKinematicsCalculator;
   private final Voxel3DGrid voxel3dGrid;
   private final SphereVoxelShape sphereVoxelShape;

   private final SimulationConstructionSet scs;
   private final OneDoFJoint[] robotArmJoints;
   private final OneDoFJoint lastJoint;
   private final int gridSizeInNumberOfVoxels = 50;
   private final double voxelSize = 0.05;
   private final int numberOfRays = 100;
   private final int numberOfRotationsAroundRay = 20;
   private final FramePoint voxelLocation = new FramePoint();
   private final FramePoint modifiableVoxelLocation = new FramePoint();
   private final RigidBodyTransform desiredEndEffectorPose = new RigidBodyTransform();
   private final RigidBodyTransform endEffectorToControlFrameOffset = new RigidBodyTransform();
   private final GeometricJacobian jacobian;
   private final Random random = new Random(645216L);
   private final ArrayList<ReachabilityMapListener> reachabilityMapListeners = new ArrayList<>();

   private ReachabilityMapFileWriter reachabilityMapFileWriter;

   public ReachabilitySphereMapCalculator(OneDoFJoint[] robotArmJoints, SimulationConstructionSet scs)
   {
      this.robotArmJoints = robotArmJoints;
      this.scs = scs;
      lastJoint = robotArmJoints[robotArmJoints.length - 1];
      jacobian = new GeometricJacobian(robotArmJoints, lastJoint.getSuccessor().getBodyFixedFrame());
      int maxIterations = 500;

      spatialInverseKinematicsCalculator = createNumericalInverseKinematicsCalculator(jacobian, maxIterations, true);
      linearInverseKinematicsCalculator = createNumericalInverseKinematicsCalculator(jacobian, maxIterations, false);

      ReferenceFrame frameBeforeRootJoint = robotArmJoints[0].getFrameBeforeJoint();
      RigidBodyTransform gridTransformToParent = new RigidBodyTransform(new AxisAngle4d(), new Vector3d(gridSizeInNumberOfVoxels * voxelSize / 3.0, 0.0, 0.0));
      ReferenceFrame gridFrame = ReferenceFrame.constructFrameWithUnchangingTransformToParent("gridFrame", frameBeforeRootJoint, gridTransformToParent);
      Graphics3DObject gridFrameViz = new Graphics3DObject();
      gridFrameViz.transform(gridFrame.getTransformToDesiredFrame(ReferenceFrame.getWorldFrame()));
      gridFrameViz.addCoordinateSystem(1.0, YoAppearance.Blue());
      scs.addStaticLinkGraphics(gridFrameViz);
      sphereVoxelShape = new SphereVoxelShape(gridFrame, voxelSize, numberOfRays, numberOfRotationsAroundRay, SphereVoxelType.graspOrigin);
      voxel3dGrid = new Voxel3DGrid(gridFrame, sphereVoxelShape, gridSizeInNumberOfVoxels, voxelSize);

      scs.addYoVariableRegistry(registry);
   }

   private NumericalInverseKinematicsCalculator createNumericalInverseKinematicsCalculator(GeometricJacobian jacobian, int maxIterations, boolean doOrientation)
   {
      double tolerance = 1e-8;
      double maxStepSize = 0.2;
      double lambdaLeastSquares = 0.0009;
      double minRandomSearchScalar = 0.01;
      double maxRandomSearchScalar = 0.8;
      DenseMatrix64F selectionMatrix;
      if (doOrientation)
         selectionMatrix = CommonOps.identity(SpatialMotionVector.SIZE);
      else
      {
         selectionMatrix = new DenseMatrix64F(3, SpatialMotionVector.SIZE);
         selectionMatrix.set(0, 3, 1.0);
         selectionMatrix.set(1, 4, 1.0);
         selectionMatrix.set(2, 5, 1.0);
      }

      NumericalInverseKinematicsCalculator numericalInverseKinematicsCalculator = new NumericalInverseKinematicsCalculator(jacobian, lambdaLeastSquares,
            tolerance, maxIterations, maxStepSize, minRandomSearchScalar, maxRandomSearchScalar);
      numericalInverseKinematicsCalculator.setSelectionMatrix(selectionMatrix);
      return numericalInverseKinematicsCalculator;
   }

   public void setControlFrameFixedInEndEffector(ReferenceFrame controlFrame)
   {
      controlFrame.getTransformToDesiredFrame(endEffectorToControlFrameOffset, jacobian.getEndEffectorFrame());
      endEffectorToControlFrameOffset.invert();
   }

   public void setTransformFromEndEffectorBodyFixedFrameToControlFrame(RigidBodyTransform transformFromEndEffectorBodyFixedFrameToControlFrame)
   {
      endEffectorToControlFrameOffset.set(transformFromEndEffectorBodyFixedFrameToControlFrame);
   }

   public void setTransformFromControlFrameToEndEffectorBodyFixedFrame(RigidBodyTransform transformFromControlFrameToEndEffectorBodyFixedFrame)
   {
      endEffectorToControlFrameOffset.set(transformFromControlFrameToEndEffectorBodyFixedFrame);
      endEffectorToControlFrameOffset.invert();
   }

   public void setupCalculatorToRecordInFile(String robotName, Class<?> classForFilePath)
   {
      if (robotName == null || robotName.isEmpty())
      {
         System.err.println("Invalid robot name (either null or empty)");
         return;
      }
      reachabilityMapFileWriter = new ReachabilityMapFileWriter(robotName, robotArmJoints, voxel3dGrid, classForFilePath);
   }

   public void attachReachabilityMapListener(ReachabilityMapListener listener)
   {
      reachabilityMapListeners.add(listener);
   }

   public void buildReachabilitySpace()
   {
      int numberOfTrials = 10;
      int counter = 0;

      for (double z = 0; z <= 0.7; z += 0.7 * 0.1)
      {
         AppearanceDefinition appearance = YoAppearance.RGBColorFromHex(Color.HSBtoRGB((float) z, 1.0f, 1.0f));
         Graphics3DObject voxelViz = new Graphics3DObject();
         voxelLocation.changeFrame(ReferenceFrame.getWorldFrame());
         voxelViz.translate(-1.0, -1.0, 0.1 + z);
         voxelViz.addSphere(0.025, appearance);
         scs.addStaticLinkGraphics(voxelViz);
      }

      FrameVector translationFromVoxelOrigin = new FrameVector();
      FrameOrientation orientation = new FrameOrientation();
      Matrix3d rotationMatrix = new Matrix3d();
      Vector3d positionError = new Vector3d();
      Vector3d orientationErrorAsVector = new Vector3d();
      AxisAngle4d orientationError = new AxisAngle4d();

      for (int xIndex = 0; xIndex < gridSizeInNumberOfVoxels; xIndex++)
      {
         for (int yIndex = 0; yIndex < gridSizeInNumberOfVoxels; yIndex++)
         {
            for (int zIndex = 0; zIndex < gridSizeInNumberOfVoxels; zIndex++)
            {
               if (!isPositionReachable(numberOfTrials, xIndex, yIndex, zIndex))
                  continue;

               for (int rayIndex = 0; rayIndex < numberOfRays; rayIndex++)
               {
                  ScrewTestTools.setRandomPositionsWithinJointLimits(robotArmJoints, random);
                  updateRobotFrames();
                  jacobian.compute();
                  voxel3dGrid.getVoxel(voxelLocation, xIndex, yIndex, zIndex);
                  counter = 0;

                  for (int rotationAroundRayIndex = 0; rotationAroundRayIndex < numberOfRotationsAroundRay; rotationAroundRayIndex++)
                  {
                     modifiableVoxelLocation.setIncludingFrame(voxelLocation);
                     sphereVoxelShape.getPose(translationFromVoxelOrigin, orientation, rayIndex, rotationAroundRayIndex);
                     modifiableVoxelLocation.add(translationFromVoxelOrigin);
                     modifiableVoxelLocation.changeFrame(jacobian.getBaseFrame());
                     orientation.changeFrame(jacobian.getBaseFrame());
                     orientation.getMatrix3d(rotationMatrix);

                     desiredEndEffectorPose.setRotation(rotationMatrix);
                     desiredEndEffectorPose.setTranslation(modifiableVoxelLocation.getX(), modifiableVoxelLocation.getY(), modifiableVoxelLocation.getZ());
                     desiredEndEffectorPose.multiply(endEffectorToControlFrameOffset);
                     boolean success = spatialInverseKinematicsCalculator.solve(desiredEndEffectorPose);

                     updateRobotFrames();

                     if (success)
                     {
                        voxel3dGrid.registerReachablePose(xIndex, yIndex, zIndex, rayIndex, rotationAroundRayIndex);
                        if (reachabilityMapFileWriter != null)
                           reachabilityMapFileWriter.registerReachablePose(xIndex, yIndex, zIndex, rayIndex, rotationAroundRayIndex);
                        if (DEBUG)
                        {
                           orientation.changeFrame(jacobian.getEndEffectorFrame());
                           orientation.getAxisAngle(orientationError);
                           orientationErrorAsVector.set(orientationError.x, orientationError.y, orientationError.z);
                           orientationErrorAsVector.scale(orientationError.angle);
                           modifiableVoxelLocation.changeFrame(jacobian.getEndEffectorFrame());
                           modifiableVoxelLocation.get(positionError);

                           System.out.println("Position error: " + positionError + ", orientation error: " + orientationErrorAsVector);
                        }

                        for (int i = 0; i < reachabilityMapListeners.size(); i++)
                        {
                           reachabilityMapListeners.get(i).hasReachedNewConfiguration();
                        }

                        scs.tickAndUpdate();
                        break;
                     }
                     else if (counter <= numberOfTrials)
                     {
                        ScrewTestTools.setRandomPositionsWithinJointLimits(robotArmJoints, random);
                        updateRobotFrames();
                        jacobian.compute();

                        counter++;
                        rotationAroundRayIndex--;
                     }
                  }
               }

               double reachabilityValue = voxel3dGrid.getD(xIndex, yIndex, zIndex);

               if (reachabilityValue > 1e-3)
               {
                  Graphics3DObject voxelViz = sphereVoxelShape.createVisualization(voxelLocation, 0.25, reachabilityValue);
                  scs.addStaticLinkGraphics(voxelViz);
               }
            }
         }
      }

      if (reachabilityMapFileWriter != null)
         reachabilityMapFileWriter.exportAndClose();
      System.out.println("Done!");
   }

   private void updateRobotFrames()
   {
      robotArmJoints[0].getPredecessor().updateFramesRecursively();
   }

   private boolean isPositionReachable(int numberOfTrials, int xIndex, int yIndex, int zIndex)
   {
      boolean success = false;
      int counter = 0;

      ScrewTestTools.setRandomPositionsWithinJointLimits(robotArmJoints, random);
      updateRobotFrames();
      voxel3dGrid.getVoxel(voxelLocation, xIndex, yIndex, zIndex);

      modifiableVoxelLocation.setIncludingFrame(voxelLocation);
      modifiableVoxelLocation.changeFrame(jacobian.getBaseFrame());
      desiredEndEffectorPose.setTranslation(modifiableVoxelLocation.getX(), modifiableVoxelLocation.getY(), modifiableVoxelLocation.getZ());

      while (counter < numberOfTrials)
      {
         success = linearInverseKinematicsCalculator.solve(desiredEndEffectorPose);

         if (success)
            break;

         ScrewTestTools.setRandomPositionsWithinJointLimits(robotArmJoints, random);
         updateRobotFrames();

         counter++;
      }

      return success;
   }
}
