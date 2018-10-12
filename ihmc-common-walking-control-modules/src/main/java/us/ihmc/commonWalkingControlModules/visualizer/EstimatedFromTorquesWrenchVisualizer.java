package us.ihmc.commonWalkingControlModules.visualizer;

import org.ejml.alg.dense.misc.UnrolledInverseFromMinor;
import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import us.ihmc.commonWalkingControlModules.momentumBasedController.GeometricJacobianHolder;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector2DReadOnly;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicVector;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsList;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactableBody;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.robotics.functionApproximation.DampedLeastSquaresSolver;
import us.ihmc.robotics.linearAlgebra.DampedLeastSquaresNullspaceCalculator;
import us.ihmc.robotics.linearAlgebra.MatrixTools;
import us.ihmc.robotics.math.YoSolvePseudoInverseSVDWithDampedLeastSquaresNearSingularities;
import us.ihmc.robotics.screwTheory.*;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoFramePoint3D;
import us.ihmc.yoVariables.variable.YoFrameVector3D;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

public class EstimatedFromTorquesWrenchVisualizer
{
   private static final double FORCE_VECTOR_SCALE = 0.0015;
   private static final double TORQUE_VECTOR_SCALE = 0.0015;

   private YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final Map<RigidBody, YoFrameVector3D> forces = new LinkedHashMap<>();
   private final Map<RigidBody, YoFrameVector3D> torques = new LinkedHashMap<>();
   private final Map<RigidBody, YoFramePoint3D> pointsOfApplication = new LinkedHashMap<>();
   private final Map<RigidBody, YoGraphicVector> forceVisualizers = new LinkedHashMap<>();
   private final Map<RigidBody, YoGraphicVector> torqueVisualizers = new LinkedHashMap<>();
   private final Map<RigidBody, GeometricJacobian> jacobians = new LinkedHashMap<>();
   private final Map<RigidBody, OneDoFJoint[]> jointLists = new LinkedHashMap<>();

   private final FrameVector3D tempVector = new FrameVector3D();
   private final FramePoint3D tempPoint = new FramePoint3D();
   private final ArrayList<RigidBody> rigidBodies = new ArrayList<>();

   private final RigidBody rootBody;

   private final DampedLeastSquaresSolver pseudoInverseSolver = new DampedLeastSquaresSolver(0, 0.005);

   private final DenseMatrix64F jacobianInverseMatrix = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F jointChainTorquesVector = new DenseMatrix64F(0, 1);
   private final DenseMatrix64F jointTorquesVector = new DenseMatrix64F(0, 1);
   private final DenseMatrix64F wrenchVector = new DenseMatrix64F(6, 1);

   public static EstimatedFromTorquesWrenchVisualizer createWrenchVisualizerWithContactableBodies(String name, RigidBody rootBody,
                                                                                                  List<? extends ContactablePlaneBody> contactableBodies,
                                                                                                  double vizScaling,
                                                                                                  YoGraphicsListRegistry yoGraphicsListRegistry,
                                                                                                  YoVariableRegistry parentRegistry)
   {
      return new EstimatedFromTorquesWrenchVisualizer(name, null, rootBody, extractRigidBodyList(contactableBodies), vizScaling, yoGraphicsListRegistry,
                                                      parentRegistry);
   }

   public EstimatedFromTorquesWrenchVisualizer(String name, GeometricJacobianHolder jacobianHolder, RigidBody rootBody, List<RigidBody> rigidBodies,
                                               double vizScaling, YoGraphicsListRegistry yoGraphicsListRegistry, YoVariableRegistry parentRegistry)
   {
      this(name, jacobianHolder, rootBody, rigidBodies, vizScaling, yoGraphicsListRegistry, parentRegistry, YoAppearance.AliceBlue(),
           YoAppearance.YellowGreen());
   }

   public EstimatedFromTorquesWrenchVisualizer(String name, GeometricJacobianHolder jacobianHolder, RigidBody rootBody, List<RigidBody> rigidBodies,
                                               double vizScaling, YoGraphicsListRegistry yoGraphicsListRegistry, YoVariableRegistry parentRegistry,
                                               AppearanceDefinition forceAppearance, AppearanceDefinition torqueAppearance)
   {
      for (RigidBody rigidBody : rigidBodies)
      {
         OneDoFJoint[] joints = ScrewTools.createOneDoFJointPath(rootBody, rigidBody);
         jointLists.put(rigidBody, joints);
         if (jacobianHolder != null)
         {
            long jacobianId = jacobianHolder.getOrCreateGeometricJacobian(joints, rigidBody.getBodyFixedFrame());
            jacobians.put(rigidBody, jacobianHolder.getJacobian(jacobianId));
         }
         else
         {
            jacobians.put(rigidBody, new GeometricJacobian(joints, rigidBody.getBodyFixedFrame()));
         }

      }
      if (jacobianHolder != null)
      {
         for (RigidBody rigidBody : rigidBodies)
         {
            long jacobianId = jacobianHolder.getOrCreateGeometricJacobian(rootBody, rigidBody, ReferenceFrame.getWorldFrame());
            jacobians.put(rigidBody, jacobianHolder.getJacobian(jacobianId));
         }
      }

      this.rootBody = rootBody;

      YoGraphicsList yoGraphicsList = new YoGraphicsList(name);

      this.rigidBodies.addAll(rigidBodies);
      for (RigidBody rigidBody : rigidBodies)
      {
         String prefix = name + rigidBody.getName();
         YoFrameVector3D force = new YoFrameVector3D(prefix + "Force", ReferenceFrame.getWorldFrame(), registry);
         forces.put(rigidBody, force);

         YoFrameVector3D torque = new YoFrameVector3D(prefix + "Torque", ReferenceFrame.getWorldFrame(), registry);
         torques.put(rigidBody, torque);

         YoFramePoint3D pointOfApplication = new YoFramePoint3D(prefix + "PointOfApplication", ReferenceFrame.getWorldFrame(), registry);
         pointsOfApplication.put(rigidBody, pointOfApplication);

         YoGraphicVector forceVisualizer = new YoGraphicVector(prefix + "ForceViz", pointOfApplication, force, FORCE_VECTOR_SCALE * vizScaling, forceAppearance,
                                                               true);
         forceVisualizers.put(rigidBody, forceVisualizer);
         yoGraphicsList.add(forceVisualizer);

         YoGraphicVector torqueVisualizer = new YoGraphicVector(prefix + "TorqueViz", pointOfApplication, torque, TORQUE_VECTOR_SCALE * vizScaling,
                                                                torqueAppearance, true);
         torqueVisualizers.put(rigidBody, torqueVisualizer);
         yoGraphicsList.add(torqueVisualizer);
      }

      yoGraphicsListRegistry.registerYoGraphicsList(yoGraphicsList);

      parentRegistry.addChild(registry);
   }

   public void update()
   {
      for (int i = 0; i < rigidBodies.size(); i++)
      {
         RigidBody rigidBody = rigidBodies.get(i);

         GeometricJacobian jacobian = jacobians.get(rigidBody);
         jacobian.compute();

         DenseMatrix64F jacobianMatrix = jacobian.getJacobianMatrix();
         jointChainTorquesVector.reshape(jacobian.getNumberOfColumns(), 1);
         jacobianInverseMatrix.reshape(jacobian.getNumberOfColumns(), 6);

         pseudoInverseSolver.setA(jacobianMatrix);
         pseudoInverseSolver.invert(jacobianInverseMatrix);

         OneDoFJoint[] joints = jointLists.get(rigidBody);
         for (int jointIndex = 0; jointIndex < jointLists.get(rigidBody).length; jointIndex++)
            jointChainTorquesVector.set(jointIndex, 0, joints[jointIndex].getTauMeasured());

         CommonOps.multTransA(jacobianInverseMatrix, jointChainTorquesVector, wrenchVector);
         CommonOps.scale(-1.0, wrenchVector);

         YoFrameVector3D torque = torques.get(rigidBody);
         tempVector.setToZero(jacobian.getJacobianFrame());
         tempVector.set(wrenchVector);
         torque.setMatchingFrame(tempVector);

         YoFrameVector3D force = forces.get(rigidBody);
         tempVector.setToZero(jacobian.getJacobianFrame());
         tempVector.set(3, wrenchVector);
         force.setMatchingFrame(tempVector);

         YoFramePoint3D pointOfApplication = pointsOfApplication.get(rigidBody);
         tempPoint.setToZero(rigidBody.getBodyFixedFrame());
         pointOfApplication.setMatchingFrame(tempPoint);
      }
   }

   private static List<RigidBody> extractRigidBodyList(List<? extends ContactableBody> contactableBodies)
   {
      List<RigidBody> ret = new ArrayList<>(contactableBodies.size());
      for (int i = 0; i < contactableBodies.size(); i++)
         ret.add(contactableBodies.get(i).getRigidBody());
      return ret;
   }
}
