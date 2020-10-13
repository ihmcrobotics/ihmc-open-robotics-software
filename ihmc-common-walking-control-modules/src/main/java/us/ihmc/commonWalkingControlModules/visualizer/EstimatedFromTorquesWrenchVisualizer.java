package us.ihmc.commonWalkingControlModules.visualizer;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;

import us.ihmc.commonWalkingControlModules.momentumBasedController.GeometricJacobianHolder;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicVector;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsList;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotics.contactable.ContactableBody;
import us.ihmc.robotics.contactable.ContactablePlaneBody;
import us.ihmc.robotics.functionApproximation.DampedLeastSquaresSolver;
import us.ihmc.robotics.screwTheory.GeometricJacobian;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;

public class EstimatedFromTorquesWrenchVisualizer
{
   private static final double FORCE_VECTOR_SCALE = 0.0015;
   private static final double TORQUE_VECTOR_SCALE = 0.0015;

   private YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final Map<RigidBodyBasics, YoFrameVector3D> forces = new LinkedHashMap<>();
   private final Map<RigidBodyBasics, YoFrameVector3D> torques = new LinkedHashMap<>();
   private final Map<RigidBodyBasics, YoFramePoint3D> pointsOfApplication = new LinkedHashMap<>();
   private final Map<RigidBodyBasics, YoGraphicVector> forceVisualizers = new LinkedHashMap<>();
   private final Map<RigidBodyBasics, YoGraphicVector> torqueVisualizers = new LinkedHashMap<>();
   private final Map<RigidBodyBasics, GeometricJacobian> jacobians = new LinkedHashMap<>();
   private final Map<RigidBodyBasics, OneDoFJointBasics[]> jointLists = new LinkedHashMap<>();

   private final FrameVector3D tempVector = new FrameVector3D();
   private final FramePoint3D tempPoint = new FramePoint3D();
   private final ArrayList<RigidBodyBasics> rigidBodies = new ArrayList<>();

   private final RigidBodyBasics rootBody;

   private final DampedLeastSquaresSolver pseudoInverseSolver = new DampedLeastSquaresSolver(0, 0.005);

   private final DMatrixRMaj jacobianInverseMatrix = new DMatrixRMaj(0, 0);
   private final DMatrixRMaj jointChainTorquesVector = new DMatrixRMaj(0, 1);
   private final DMatrixRMaj jointTorquesVector = new DMatrixRMaj(0, 1);
   private final DMatrixRMaj wrenchVector = new DMatrixRMaj(6, 1);

   public static EstimatedFromTorquesWrenchVisualizer createWrenchVisualizerWithContactableBodies(String name, RigidBodyBasics rootBody,
                                                                                                  List<? extends ContactablePlaneBody> contactableBodies,
                                                                                                  double vizScaling,
                                                                                                  YoGraphicsListRegistry yoGraphicsListRegistry,
                                                                                                  YoRegistry parentRegistry)
   {
      return new EstimatedFromTorquesWrenchVisualizer(name, null, rootBody, extractRigidBodyList(contactableBodies), vizScaling, yoGraphicsListRegistry,
                                                      parentRegistry);
   }

   public EstimatedFromTorquesWrenchVisualizer(String name, GeometricJacobianHolder jacobianHolder, RigidBodyBasics rootBody, List<RigidBodyBasics> rigidBodies,
                                               double vizScaling, YoGraphicsListRegistry yoGraphicsListRegistry, YoRegistry parentRegistry)
   {
      this(name, jacobianHolder, rootBody, rigidBodies, vizScaling, yoGraphicsListRegistry, parentRegistry, YoAppearance.AliceBlue(),
           YoAppearance.YellowGreen());
   }

   public EstimatedFromTorquesWrenchVisualizer(String name, GeometricJacobianHolder jacobianHolder, RigidBodyBasics rootBody, List<RigidBodyBasics> rigidBodies,
                                               double vizScaling, YoGraphicsListRegistry yoGraphicsListRegistry, YoRegistry parentRegistry,
                                               AppearanceDefinition forceAppearance, AppearanceDefinition torqueAppearance)
   {
      for (RigidBodyBasics rigidBody : rigidBodies)
      {
         OneDoFJointBasics[] joints = MultiBodySystemTools.createOneDoFJointPath(rootBody, rigidBody);
         jointLists.put(rigidBody, joints);
         if (jacobianHolder != null)
         {
            int jacobianId = jacobianHolder.getOrCreateGeometricJacobian(joints, rigidBody.getBodyFixedFrame());
            jacobians.put(rigidBody, jacobianHolder.getJacobian(jacobianId));
         }
         else
         {
            jacobians.put(rigidBody, new GeometricJacobian(joints, rigidBody.getBodyFixedFrame()));
         }

      }
      if (jacobianHolder != null)
      {
         for (RigidBodyBasics rigidBody : rigidBodies)
         {
            int jacobianId = jacobianHolder.getOrCreateGeometricJacobian(rootBody, rigidBody, ReferenceFrame.getWorldFrame());
            jacobians.put(rigidBody, jacobianHolder.getJacobian(jacobianId));
         }
      }

      this.rootBody = rootBody;

      YoGraphicsList yoGraphicsList = new YoGraphicsList(name);

      this.rigidBodies.addAll(rigidBodies);
      for (RigidBodyBasics rigidBody : rigidBodies)
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
         RigidBodyBasics rigidBody = rigidBodies.get(i);

         GeometricJacobian jacobian = jacobians.get(rigidBody);
         jacobian.compute();

         DMatrixRMaj jacobianMatrix = jacobian.getJacobianMatrix();
         jointChainTorquesVector.reshape(jacobian.getNumberOfColumns(), 1);
         jacobianInverseMatrix.reshape(jacobian.getNumberOfColumns(), 6);

         pseudoInverseSolver.setA(jacobianMatrix);
         pseudoInverseSolver.invert(jacobianInverseMatrix);

         OneDoFJointBasics[] joints = jointLists.get(rigidBody);
         for (int jointIndex = 0; jointIndex < jointLists.get(rigidBody).length; jointIndex++)
            jointChainTorquesVector.set(jointIndex, 0, joints[jointIndex].getTau());

         CommonOps_DDRM.multTransA(jacobianInverseMatrix, jointChainTorquesVector, wrenchVector);
         CommonOps_DDRM.scale(-1.0, wrenchVector);

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

   private static List<RigidBodyBasics> extractRigidBodyList(List<? extends ContactableBody> contactableBodies)
   {
      List<RigidBodyBasics> ret = new ArrayList<>(contactableBodies.size());
      for (int i = 0; i < contactableBodies.size(); i++)
         ret.add(contactableBodies.get(i).getRigidBody());
      return ret;
   }
}
