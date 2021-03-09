package us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation;

import java.util.Collections;
import java.util.Map;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.dense.row.factory.LinearSolverFactory_DDRM;
import org.ejml.interfaces.linsol.LinearSolverDense;

import us.ihmc.commonWalkingControlModules.controlModules.CenterOfPressureResolver;
import us.ihmc.commonWalkingControlModules.visualizer.WrenchVisualizer;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition.GraphicType;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactPosition;
import us.ihmc.mecano.algorithms.GeometricJacobianCalculator;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.interfaces.WrenchReadOnly;
import us.ihmc.mecano.tools.JointStateType;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.mecano.yoVariables.spatial.YoFixedFrameWrench;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint2D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.registry.YoRegistry;

public class JointTorqueBasedWrenchCalculator
{
   private OneDoFJointBasics[] joints;
   private final GeometricJacobianCalculator geometricJacobianCalculator = new GeometricJacobianCalculator();
   private final DMatrixRMaj jointTorques = new DMatrixRMaj(6, 1);
   private final DMatrixRMaj jacobianMatrixTranspose = new DMatrixRMaj(6, 6);
   private final LinearSolverDense<DMatrixRMaj> solver = LinearSolverFactory_DDRM.pseudoInverse(false);
   private final DMatrixRMaj wrench = new DMatrixRMaj(6, 1);

   private final YoFixedFrameWrench estimatedWrench;
   private final WrenchVisualizer visualizer;
   private final Map<RigidBodyBasics, WrenchReadOnly> wrenchMap;
   private final YoFramePoint2D cop2D;
   private final YoFramePoint3D cop3D;

   private final CenterOfPressureResolver centerOfPressureResolver = new CenterOfPressureResolver();

   public JointTorqueBasedWrenchCalculator(String namePrefix,
                                           OneDoFJointBasics[] joints,
                                           RigidBodyBasics wrenchBody,
                                           ReferenceFrame copFrame,
                                           YoRegistry registry,
                                           YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this.joints = joints;
      geometricJacobianCalculator.setKinematicChain(joints);
      geometricJacobianCalculator.setJacobianFrame(wrenchBody.getBodyFixedFrame());
      estimatedWrench = new YoFixedFrameWrench(namePrefix, wrenchBody.getBodyFixedFrame(), wrenchBody.getBodyFixedFrame(), registry);
      cop2D = new YoFramePoint2D(namePrefix + "InSole", "", copFrame, registry);
      cop3D = new YoFramePoint3D(namePrefix + "InWorld", "", ReferenceFrame.getWorldFrame(), registry);
      visualizer = new WrenchVisualizer(namePrefix,
                                        Collections.singletonList(wrenchBody),
                                        1.0,
                                        yoGraphicsListRegistry,
                                        registry,
                                        YoAppearance.Blue(),
                                        YoAppearance.BlueViolet());

      YoGraphicPosition copYoGraphic = new YoGraphicPosition("Test " + wrenchBody + "CoP", cop3D, 0.008, YoAppearance.DarkViolet(), GraphicType.CROSS);
      YoArtifactPosition copArtifact = copYoGraphic.createArtifact();
      yoGraphicsListRegistry.registerArtifact("JointTorqueBasedCoP", copArtifact);

      wrenchMap = Collections.singletonMap(wrenchBody, estimatedWrench);
   }

   public void update()
   {
      MultiBodySystemTools.extractJointsState(joints, JointStateType.EFFORT, jointTorques);

      geometricJacobianCalculator.reset();
      CommonOps_DDRM.transpose(geometricJacobianCalculator.getJacobianMatrix(), jacobianMatrixTranspose);
      solver.setA(jacobianMatrixTranspose);
      solver.solve(jointTorques, wrench);
      CommonOps_DDRM.scale(-1, wrench);
      estimatedWrench.set(wrench);
      visualizer.visualize(wrenchMap);
      centerOfPressureResolver.resolveCenterOfPressureAndNormalTorque(cop2D, estimatedWrench, cop2D.getReferenceFrame());
      cop3D.setMatchingFrame(cop2D, 0.0);
   }
}
