package us.ihmc.commonWalkingControlModules.staticEquilibrium;

import gnu.trove.list.array.TIntArrayList;
import org.ejml.data.DMatrixRMaj;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DBasics;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple2DReadOnly;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.spatial.Twist;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.SCS2YoGraphicHolder;
import us.ihmc.robotics.math.filters.GlitchFilteredYoBoolean;
import us.ihmc.robotics.time.ExecutionTimer;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinitionFactory;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinitionFactory.DefaultPoint2DGraphic;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicGroupDefinition;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameLineSegment2D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint2D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector2D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

import static us.ihmc.commonWalkingControlModules.staticEquilibrium.CenterOfMassStabilityMarginRegionCalculator.*;

public class CoMMarginSensitivityCalculator implements SCS2YoGraphicHolder
{
   public static final double DT = 1.0e-2;
   private static final int MAX_NULLSPACE = 10;

   private final YoRegistry registry;
   private final VertexLookAheadData[] predictionData = new VertexLookAheadData[DIRECTIONS_TO_OPTIMIZE];
   private final CenterOfMassStabilityMarginRegionCalculator staticStabilityRegionCalculator;
   private final CenterOfMassStabilityMarginOptimizationModule optimizationModule;
   private final FullHumanoidRobotModel fullRobotModel;
   private final OneDoFJointBasics[] oneDoFJoints;
   private final WholeBodyContactState contactState;
   private final GlitchFilteredYoBoolean predictedSubOptimalVertex;

   // Fields to adjust and cache posture if requested
   private final Pose3D cachedRootJointPosition = new Pose3D();
   private final Twist cachedRootJointTwist = new Twist();
   private final double[] cachedJointAngles;
   private final double[] cachedJointVelocities;
   private final Vector3D pelvisAngularVelocityPostureAdjustment = new Vector3D();
   private final Vector3D pelvisLinearVelocityPostureAdjustment = new Vector3D();
   private final Vector3D pelvisRotationVectorAdjustment = new Vector3D();
   private final Quaternion pelvisRotationQuaternionAdjustment = new Quaternion();

   // Computed sensitivity values, indexed as [vertex index identifier][nullspace index identifier]
   private final YoFrameVector2D[][] sensitivityValues = new YoFrameVector2D[DIRECTIONS_TO_OPTIMIZE][MAX_NULLSPACE];

   private final ExecutionTimer fixedBasisSolveTimer;

   public CoMMarginSensitivityCalculator(String prefix,
                                         CenterOfMassStabilityMarginRegionCalculator staticStabilityRegionCalculator,
                                         FullHumanoidRobotModel fullRobotModel,
                                         WholeBodyContactState contactState,
                                         YoRegistry parentRegistry)
   {
      registry = new YoRegistry(prefix + getClass().getSimpleName());
      this.staticStabilityRegionCalculator = staticStabilityRegionCalculator;
      this.optimizationModule = staticStabilityRegionCalculator.getOptimizationModule();
      this.fullRobotModel = fullRobotModel;
      this.contactState = contactState;
      this.oneDoFJoints = contactState.getOneDoFJoints();
      this.predictedSubOptimalVertex = new GlitchFilteredYoBoolean("predictedSubOptimalVertex", registry, 3);

      for (int i = 0; i < DIRECTIONS_TO_OPTIMIZE; i++)
      {
         predictionData[i] = new VertexLookAheadData(i);
      }

      for (int nullspace_idx = 0; nullspace_idx < MAX_NULLSPACE; nullspace_idx++)
      {
         for (int verted_idx = 0; verted_idx < DIRECTIONS_TO_OPTIMIZE; verted_idx++)
         {
            sensitivityValues[verted_idx][nullspace_idx] = new YoFrameVector2D("sensitivity_" + nullspace_idx + "_" + verted_idx, ReferenceFrame.getWorldFrame(), registry);
         }
      }

      cachedJointAngles = new double[contactState.getNumberOfJoints()];
      cachedJointVelocities = new double[contactState.getNumberOfJoints()];
      fixedBasisSolveTimer = new ExecutionTimer("fixedBasisSolveTimer", registry);

      parentRegistry.addChild(registry);
   }

   public void compute(boolean setJointVelocitiesToZero,
                       boolean integrateWithCurrentVelocities,
                       DMatrixRMaj wholeBodyVelocityForIntegration,
                       double integrationDT,
                       TIntArrayList predictionVertices)
   {
      boolean mutateRobotState = setJointVelocitiesToZero || integrateWithCurrentVelocities || wholeBodyVelocityForIntegration != null;

      if (mutateRobotState)
      {
         saveCurrentRobotState();
      }

      if (integrateWithCurrentVelocities || wholeBodyVelocityForIntegration != null)
      {
         integrateOneTimestep(integrateWithCurrentVelocities, wholeBodyVelocityForIntegration);
      }

      if (setJointVelocitiesToZero)
      {
         setJointVelocitiesToZero();
      }

      if (mutateRobotState)
      { // Update stability margin constraints
         fullRobotModel.updateFrames();
         contactState.update();
         optimizationModule.updateContactState(contactState);
      }

      for (int i = 0; i < predictionData.length; i++)
      {
         predictionData[i].clear();
      }

      fixedBasisSolveTimer.startMeasurement();
      boolean containsSubOptimalVertex = false;

      for (int i = 0; i < predictionVertices.size(); i++)
      {
         int index = predictionVertices.get(i);

         TIntArrayList solutionBasisIndices = staticStabilityRegionCalculator.getSolutionBasisIndices(index);
         optimizationModule.solveForFixedBasis(solutionBasisIndices);
         containsSubOptimalVertex = containsSubOptimalVertex || predictionData[index].update(optimizationModule.getOptimizedCoM());
      }

      predictedSubOptimalVertex.update(containsSubOptimalVertex);
      fixedBasisSolveTimer.stopMeasurement();

      if (mutateRobotState)
      {
         writeInitialRobotState();
      }
   }

   // Assumes velocity is unit magnitude
   public Tuple2DReadOnly computeSensitivity(int nullspaceIndex, int vertexIndex, DMatrixRMaj velocity, Runnable integrationCallback)
   {
      YoFramePoint2D v0 = staticStabilityRegionCalculator.getOptimizedVertex(vertexIndex);

      saveCurrentRobotState();
      integrateOneTimestep(false, velocity);

      integrationCallback.run();

      TIntArrayList solutionBasis = staticStabilityRegionCalculator.getSolutionBasisIndices(vertexIndex);
      Point2D v1 = staticStabilityRegionCalculator.getOptimizationModule().solveForFixedBasis(solutionBasis);
      writeInitialRobotState();

      // delta c
      sensitivityValues[vertexIndex][nullspaceIndex].sub(v1, v0);

      // delta c / dt (we leave out |q_dot| from the denominator)
//      sensitivityValues[vertexIndex][nullspaceIndex].scale(1.0 / DT);

//      return v1;
      return sensitivityValues[vertexIndex][nullspaceIndex];

//      YoFramePoint2D v0 = staticStabilityRegionCalculator.getOptimizedVertex(vertexIndex);
//      saveCurrentRobotState();
////      integrateOneTimestep(false, velocity);
//      integrationCallback.run();
//
////      fullRobotModel.updateFrames();
////      contactState.update();
//      optimizationModule.updateContactState(contactState);
//
////      TIntArrayList solutionBasisIndices = staticStabilityRegionCalculator.getSolutionBasisIndices(vertexIndex);
////      Point2D v1 = optimizationModule.solveForFixedBasis(solutionBasisIndices);
//
//      staticStabilityRegionCalculator.performCoMRegionQuery(vertexIndex);
//      YoFramePoint2D v1 = staticStabilityRegionCalculator.getOptimizedVertex(vertexIndex);
//
//      // delta c
//      sensitivityValues[vertexIndex][nullspaceIndex].sub(v1, v0);
//
//      // delta c / dt (we leave out |q_dot| from the denominator)
//      sensitivityValues[vertexIndex][nullspaceIndex].scale(1.0 / DT);
//
//      writeInitialRobotState();
//
//      return v1; // sensitivityValues[vertexIndex][nullspaceIndex];
   }

   private void setJointVelocitiesToZero()
   {
      // Set all joint velocities to zero
      for (int i = 0; i < contactState.getNumberOfJoints(); i++)
      {
         oneDoFJoints[i].setQd(0.0);
         oneDoFJoints[i].setQdd(0.0);
      }

      fullRobotModel.getRootJoint().getJointTwist().setToZero();
      fullRobotModel.getRootJoint().getJointAcceleration().setToZero();
   }

   private void saveCurrentRobotState()
   {
      // Cache initial robot state
      Pose3DBasics fullRobotModelRootJointPose = fullRobotModel.getRootJoint().getJointPose();
      cachedRootJointPosition.set(fullRobotModelRootJointPose);
      cachedRootJointTwist.setIncludingFrame(fullRobotModel.getRootJoint().getJointTwist());
      // TODO save root acceleration?

      for (int i = 0; i < contactState.getNumberOfJoints(); i++)
      {
         cachedJointAngles[i] = oneDoFJoints[i].getQ();
         cachedJointVelocities[i] = oneDoFJoints[i].getQd();
      }
   }

   private void integrateOneTimestep(boolean integrateWithCurrentVelocities, DMatrixRMaj wholeBodyVelocityForIntegration)
   {
      FloatingJointBasics rootJoint = fullRobotModel.getRootJoint();

      if (integrateWithCurrentVelocities)
      {
         pelvisAngularVelocityPostureAdjustment.set(rootJoint.getJointTwist().getAngularPart());
         pelvisLinearVelocityPostureAdjustment.set(rootJoint.getJointTwist().getLinearPart());
      }
      else
      {
         for (int i = 0; i < 3; i++)
         {
            pelvisAngularVelocityPostureAdjustment.setElement(i, wholeBodyVelocityForIntegration.get(i));
            pelvisLinearVelocityPostureAdjustment.setElement(i, wholeBodyVelocityForIntegration.get(3 + i, 0));
         }
      }

      Pose3DBasics fullRobotModelRootJointPose = rootJoint.getJointPose();
      Point3DBasics rootJointPosition = fullRobotModelRootJointPose.getPosition();
      rootJointPosition.scaleAdd(DT, pelvisLinearVelocityPostureAdjustment, rootJointPosition);

      pelvisRotationVectorAdjustment.setAndScale(DT, pelvisAngularVelocityPostureAdjustment);
      pelvisRotationQuaternionAdjustment.setRotationVector(pelvisRotationVectorAdjustment);
      fullRobotModelRootJointPose.getOrientation().append(pelvisRotationQuaternionAdjustment);

      for (int i = 0; i < contactState.getNumberOfJoints(); i++)
      {
         int spatialDimensions = 6;
         double qd = integrateWithCurrentVelocities ? oneDoFJoints[i].getQd() : wholeBodyVelocityForIntegration.get(spatialDimensions + i, 0);
         oneDoFJoints[i].setQ(oneDoFJoints[i].getQ() + DT * qd);
      }
   }

   private void writeInitialRobotState()
   {
      Pose3DBasics fullRobotModelRootJointPose = fullRobotModel.getRootJoint().getJointPose();
      fullRobotModelRootJointPose.set(cachedRootJointPosition);
      fullRobotModel.getRootJoint().getJointTwist().set(cachedRootJointTwist);

      for (int i = 0; i < contactState.getNumberOfJoints(); i++)
      {
         oneDoFJoints[i].setQ(cachedJointAngles[i]);
         oneDoFJoints[i].setQd(cachedJointVelocities[i]);
      }

      fullRobotModel.updateFrames();
   }

   private class VertexLookAheadData
   {
      final YoFramePoint2D nominalVertex;
      final YoFramePoint2D predictedVertex;
      final YoFrameLineSegment2D predictionOffset;
      final YoDouble predictedVertexDistance;
      final YoBoolean isPredictedVertexOptimal;

      final double queryDirectionX;
      final double queryDirectionY;

      VertexLookAheadData(int index)
      {
         nominalVertex = staticStabilityRegionCalculator.getOptimizedVertex(index);
         predictedVertex = new YoFramePoint2D("comStabilityMarginVertexPredicted" + index, ReferenceFrame.getWorldFrame(), registry);
         predictionOffset = new YoFrameLineSegment2D(nominalVertex, predictedVertex);
         predictedVertexDistance = new YoDouble("predictedVertexDistance" + index, registry);
         isPredictedVertexOptimal = new YoBoolean("isPredictedVertexOptimal" + index, registry);

         queryDirectionX = queryDirectionX(index);
         queryDirectionY = queryDirectionY(index);
      }

      void clear()
      {
         predictedVertex.setToNaN();
         predictedVertexDistance.setToNaN();
         isPredictedVertexOptimal.set(true);
      }

      boolean update(Point2DReadOnly predictedVertex)
      {
         this.predictedVertex.set(predictedVertex);
         double deltaVertexX = predictedVertex.getX() - nominalVertex.getX();
         double deltaVertexY = predictedVertex.getY() - nominalVertex.getY();
         predictedVertexDistance.set(nominalVertex.distance(predictedVertex));
         isPredictedVertexOptimal.set(deltaVertexX * queryDirectionX + deltaVertexY * queryDirectionY >= 0.0);

         return isPredictedVertexOptimal.getValue();
      }
   }

   @Override
   public YoGraphicDefinition getSCS2YoGraphics()
   {
      YoGraphicGroupDefinition groupDefinition = new YoGraphicGroupDefinition(getClass().getSimpleName());

      for (int i = 0; i < DIRECTIONS_TO_OPTIMIZE; i++)
      {
         groupDefinition.addChild(YoGraphicDefinitionFactory.newYoGraphicPoint2D("predicted_point_" + i,
                                                                                 predictionData[i].predictedVertex,
                                                                                 0.005,
                                                                                 ColorDefinitions.Green(),
                                                                                 DefaultPoint2DGraphic.SQUARE_FILLED));
         groupDefinition.addChild(YoGraphicDefinitionFactory.newYoGraphicLineSegment2DDefinition("predicted_point_seg_" + i,
                                                                                                 predictionData[i].predictionOffset,
                                                                                                 ColorDefinitions.Green()));
      }

      return groupDefinition;
   }
}