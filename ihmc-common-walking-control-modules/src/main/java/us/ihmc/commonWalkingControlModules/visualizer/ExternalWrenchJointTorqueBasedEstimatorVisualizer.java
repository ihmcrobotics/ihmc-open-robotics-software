package us.ihmc.commonWalkingControlModules.visualizer;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;

import us.ihmc.commonWalkingControlModules.controlModules.CenterOfPressureResolver;
import us.ihmc.commonWalkingControlModules.momentumBasedController.GeometricJacobianHolder;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition.GraphicType;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicVector;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactPosition;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.Wrench;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.commons.robotics.contactable.ContactablePlaneBody;
import us.ihmc.math.linearAlgebra.DampedLeastSquaresSolver;
import us.ihmc.robotics.screwTheory.GeometricJacobian;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint2D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;

public class ExternalWrenchJointTorqueBasedEstimatorVisualizer
{
   private static final double FORCE_VECTOR_SCALE = 0.0015;
   private static final double TORQUE_VECTOR_SCALE = 0.0015;

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final DampedLeastSquaresSolver pseudoInverseSolver = new DampedLeastSquaresSolver(0, 0.005);

   private final DMatrixRMaj jacobianInverseMatrix = new DMatrixRMaj(0, 0);
   private final DMatrixRMaj jointChainTorquesVector = new DMatrixRMaj(0, 1);
   private final DMatrixRMaj wrenchVector = new DMatrixRMaj(6, 1);
   private final Wrench estimatedWrench = new Wrench();;

   private final String name;
   private final RigidBodyBasics rootBody;
   private final List<RigidBodyCalculator> calculators = new ArrayList<>();
   private final CenterOfPressureResolver centerOfPressureResolver = new CenterOfPressureResolver();

   public static ExternalWrenchJointTorqueBasedEstimatorVisualizer createWrenchVisualizerWithContactableBodies(String name,
                                                                                                               RigidBodyBasics rootBody,
                                                                                                               List<? extends ContactablePlaneBody> contactableBodies,
                                                                                                               double vizScaling,
                                                                                                               YoGraphicsListRegistry yoGraphicsListRegistry,
                                                                                                               YoRegistry parentRegistry)
   {
      return new ExternalWrenchJointTorqueBasedEstimatorVisualizer(name,
                                                                   null,
                                                                   rootBody,
                                                                   contactableBodies.stream().map(ContactablePlaneBody::getRigidBody)
                                                                                    .collect(Collectors.toList()),
                                                                   contactableBodies.stream().map(ContactablePlaneBody::getContactFrame)
                                                                                    .collect(Collectors.toList()),
                                                                   vizScaling,
                                                                   yoGraphicsListRegistry,
                                                                   parentRegistry);
   }

   public ExternalWrenchJointTorqueBasedEstimatorVisualizer(String name,
                                                            GeometricJacobianHolder jacobianHolder,
                                                            RigidBodyBasics rootBody,
                                                            List<RigidBodyBasics> rigidBodies,
                                                            List<ReferenceFrame> soleFrames,
                                                            double vizScaling,
                                                            YoGraphicsListRegistry yoGraphicsListRegistry,
                                                            YoRegistry parentRegistry)
   {
      this(name, jacobianHolder, rootBody, rigidBodies, soleFrames, vizScaling, yoGraphicsListRegistry, parentRegistry, YoAppearance.AliceBlue(),
           YoAppearance.YellowGreen(), YoAppearance.DarkViolet(), GraphicType.CROSS);
   }

   public ExternalWrenchJointTorqueBasedEstimatorVisualizer(String name,
                                                            GeometricJacobianHolder jacobianHolder,
                                                            RigidBodyBasics rootBody,
                                                            List<RigidBodyBasics> rigidBodies,
                                                            List<ReferenceFrame> soleFrames,
                                                            double vizScaling,
                                                            YoGraphicsListRegistry yoGraphicsListRegistry,
                                                            YoRegistry parentRegistry,
                                                            AppearanceDefinition forceAppearance,
                                                            AppearanceDefinition torqueAppearance,
                                                            AppearanceDefinition copAppearance,
                                                            GraphicType copGraphicType)
   {
      this.name = name;
      this.rootBody = rootBody;

      for (int i = 0; i < rigidBodies.size(); i++)
      {
         RigidBodyBasics rigidBody = rigidBodies.get(i);
         ReferenceFrame soleFrame = soleFrames == null ? null : soleFrames.get(i);
         calculators.add(new RigidBodyCalculator(rigidBody,
                                                 soleFrame,
                                                 jacobianHolder,
                                                 vizScaling,
                                                 forceAppearance,
                                                 torqueAppearance,
                                                 copAppearance,
                                                 copGraphicType,
                                                 yoGraphicsListRegistry));
      }

      parentRegistry.addChild(registry);
   }

   public void update()
   {
      for (int i = 0; i < calculators.size(); i++)
      {
         calculators.get(i).update();
      }
   }

   private class RigidBodyCalculator
   {
      private final OneDoFJointBasics[] joints;
      private final RigidBodyBasics rigidBody;
      private final GeometricJacobian jacobian;

      private final YoFrameVector3D force;
      private final YoFrameVector3D torque;
      private final YoFramePoint3D pointOfApplication;
      private final YoGraphicVector forceVisualizer;
      private final YoGraphicVector torqueVisualizer;

      private final YoFramePoint2D cop2D;
      private final YoFramePoint3D cop3D;

      public RigidBodyCalculator(RigidBodyBasics rigidBody,
                                 ReferenceFrame soleFrame,
                                 GeometricJacobianHolder jacobianHolder,
                                 double vizScaling,
                                 AppearanceDefinition forceAppearance,
                                 AppearanceDefinition torqueAppearance,
                                 AppearanceDefinition copAppearance,
                                 GraphicType copGraphicType,
                                 YoGraphicsListRegistry yoGraphicsListRegistry)
      {
         this.rigidBody = rigidBody;
         joints = MultiBodySystemTools.createOneDoFJointPath(rootBody, rigidBody);
         if (jacobianHolder != null)
            jacobian = jacobianHolder.getJacobian(jacobianHolder.getOrCreateGeometricJacobian(joints, rigidBody.getBodyFixedFrame()));
         else
            jacobian = new GeometricJacobian(joints, rigidBody.getBodyFixedFrame());

         String prefix = name + rigidBody.getName();
         force = new YoFrameVector3D(prefix + "Force", ReferenceFrame.getWorldFrame(), registry);
         torque = new YoFrameVector3D(prefix + "Torque", ReferenceFrame.getWorldFrame(), registry);
         pointOfApplication = new YoFramePoint3D(prefix + "PointOfApplication", ReferenceFrame.getWorldFrame(), registry);
         forceVisualizer = new YoGraphicVector(prefix + "ForceViz", pointOfApplication, force, FORCE_VECTOR_SCALE * vizScaling, forceAppearance, true);
         torqueVisualizer = new YoGraphicVector(prefix + "TorqueViz", pointOfApplication, torque, TORQUE_VECTOR_SCALE * vizScaling, torqueAppearance, true);
         yoGraphicsListRegistry.registerYoGraphic(name, forceVisualizer);
         yoGraphicsListRegistry.registerYoGraphic(name, torqueVisualizer);

         if (soleFrame != null)
         {
            cop2D = new YoFramePoint2D(prefix + "CoPInSole", "", soleFrame, registry);
            cop3D = new YoFramePoint3D(prefix + "CoPInWorld", "", ReferenceFrame.getWorldFrame(), registry);

            YoGraphicPosition copYoGraphic = new YoGraphicPosition(prefix + "CoP", cop3D, 0.008, copAppearance, copGraphicType);
            YoArtifactPosition copArtifact = copYoGraphic.createArtifact();
            yoGraphicsListRegistry.registerArtifact(name, copArtifact);
         }
         else
         {
            cop2D = null;
            cop3D = null;
         }
      }

      public void update()
      {
         jacobian.compute();

         DMatrixRMaj jacobianMatrix = jacobian.getJacobianMatrix();
         jointChainTorquesVector.reshape(jacobian.getNumberOfColumns(), 1);
         jacobianInverseMatrix.reshape(jacobian.getNumberOfColumns(), 6);

         pseudoInverseSolver.setA(jacobianMatrix);
         pseudoInverseSolver.invert(jacobianInverseMatrix);

         for (int jointIndex = 0; jointIndex < joints.length; jointIndex++)
            jointChainTorquesVector.set(jointIndex, 0, joints[jointIndex].getTau());

         CommonOps_DDRM.multTransA(jacobianInverseMatrix, jointChainTorquesVector, wrenchVector);
         CommonOps_DDRM.scale(-1.0, wrenchVector);

         estimatedWrench.setToZero(rigidBody.getBodyFixedFrame(), jacobian.getJacobianFrame());
         estimatedWrench.set(wrenchVector);
         torque.setMatchingFrame(estimatedWrench.getAngularPart());
         force.setMatchingFrame(estimatedWrench.getLinearPart());

         pointOfApplication.setFromReferenceFrame(rigidBody.getBodyFixedFrame());

         if (cop2D != null)
         {
            centerOfPressureResolver.resolveCenterOfPressureAndNormalTorque(cop2D, estimatedWrench, cop2D.getReferenceFrame());
            cop3D.setMatchingFrame(cop2D, 0.0);
         }
      }
   }
}
