package us.ihmc.commonWalkingControlModules.controllerCore;

import java.util.List;

import us.ihmc.SdfLoader.models.FullRobotModel;
import us.ihmc.commonWalkingControlModules.momentumBasedController.GeometricJacobianHolder;
import us.ihmc.commonWalkingControlModules.momentumBasedController.PlaneContactWrenchProcessor;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.JointIndexHandler;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.MomentumOptimizationSettings;
import us.ihmc.commonWalkingControlModules.visualizer.WrenchVisualizer;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.*;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicsListRegistry;

public class WholeBodyControlCoreToolbox
{
   public static final int nBasisVectorsPerContactPoint = 4;
   public static final int nContactPointsPerContactableBody = 4;
   public static final int nContactableBodies = 2;
   public static final int rhoSize = nContactableBodies * nContactPointsPerContactableBody * nBasisVectorsPerContactPoint;

   private final GeometricJacobianHolder geometricJacobianHolder;
   private final TwistCalculator twistCalculator;
   private final SpatialAccelerationCalculator spatialAccelerationCalculator;
   private final InverseDynamicsCalculator inverseDynamicsCalculator;
   private final double controlDT;
   private final FullRobotModel fullRobotModel;
   private final RigidBody[] endEffectors;
   private final YoGraphicsListRegistry yoGraphicsListRegistry;
   private final List<? extends ContactablePlaneBody> contactablePlaneBodies;
   private final CommonHumanoidReferenceFrames referenceFrames;
   private final double gravityZ;

   private final MomentumOptimizationSettings momentumOptimizationSettings;

   private final PlaneContactWrenchProcessor planeContactWrenchProcessor;
   private final WrenchVisualizer wrenchVisualizer;

   private final YoFrameVector yoDesiredMomentumRateLinear;
   private final YoFrameVector yoAchievedMomentumRateLinear;

   private final YoFrameVector yoResidualRootJointForce;
   private final YoFrameVector yoResidualRootJointTorque;

   private final JointIndexHandler jointIndexHandler;

   public WholeBodyControlCoreToolbox(FullRobotModel fullRobotModel, RigidBody[] endEffectors, InverseDynamicsJoint[] controlledJoints,
         MomentumOptimizationSettings momentumOptimizationSettings, CommonHumanoidReferenceFrames referenceFrames, double controlDT, double gravityZ,
         GeometricJacobianHolder geometricJacobianHolder, TwistCalculator twistCalculator, List<? extends ContactablePlaneBody> contactablePlaneBodies,
         YoGraphicsListRegistry yoGraphicsListRegistry, YoVariableRegistry registry)
   {
      this.fullRobotModel = fullRobotModel;
      this.endEffectors = endEffectors;
      this.momentumOptimizationSettings = momentumOptimizationSettings;
      this.referenceFrames = referenceFrames;
      this.controlDT = controlDT;
      this.gravityZ = gravityZ;
      this.twistCalculator = twistCalculator;
      this.geometricJacobianHolder = geometricJacobianHolder;
      this.contactablePlaneBodies = contactablePlaneBodies;
      this.yoGraphicsListRegistry = yoGraphicsListRegistry;

      this.planeContactWrenchProcessor = new PlaneContactWrenchProcessor(contactablePlaneBodies, yoGraphicsListRegistry, registry);
      this.wrenchVisualizer = WrenchVisualizer.createWrenchVisualizerWithContactableBodies("DesiredExternalWrench", contactablePlaneBodies, 1.0,
            yoGraphicsListRegistry, registry);

      this.yoDesiredMomentumRateLinear = new YoFrameVector("desiredMomentumRateLinear", referenceFrames.getCenterOfMassFrame(), registry);
      this.yoAchievedMomentumRateLinear = new YoFrameVector("achievedMomentumRateLinear", referenceFrames.getCenterOfMassFrame(), registry);

      this.yoResidualRootJointForce = new YoFrameVector("residualRootJointForce", ReferenceFrame.getWorldFrame(), registry);
      this.yoResidualRootJointTorque = new YoFrameVector("residualRootJointTorque", ReferenceFrame.getWorldFrame(), registry);

      this.jointIndexHandler = new JointIndexHandler(controlledJoints);
      this.inverseDynamicsCalculator = new InverseDynamicsCalculator(twistCalculator, gravityZ);
      this.spatialAccelerationCalculator = inverseDynamicsCalculator.getSpatialAccelerationCalculator();
   }

   public MomentumOptimizationSettings getMomentumOptimizationSettings()
   {
      return momentumOptimizationSettings;
   }

   public RigidBody[] getEndEffectors()
   {
      return endEffectors;
   }

   public TwistCalculator getTwistCalculator()
   {
      return twistCalculator;
   }

   public SpatialAccelerationCalculator getSpatialAccelerationCalculator()
   {
      return spatialAccelerationCalculator;
   }

   public InverseDynamicsCalculator getInverseDynamicsCalculator()
   {
      return inverseDynamicsCalculator;
   }

   public SixDoFJoint getRobotRootJoint()
   {
      return fullRobotModel.getRootJoint();
   }

   public FullRobotModel getFullRobotModel()
   {
      return fullRobotModel;
   }

   public double getControlDT()
   {
      return controlDT;
   }

   public CommonHumanoidReferenceFrames getReferenceFrames()
   {
      return referenceFrames;
   }

   public ReferenceFrame getCenterOfMassFrame()
   {
      return referenceFrames.getCenterOfMassFrame();
   }

   public double getGravityZ()
   {
      return gravityZ;
   }

   public GeometricJacobianHolder getGeometricJacobianHolder()
   {
      return geometricJacobianHolder;
   }

   public long getOrCreateGeometricJacobian(RigidBody ancestor, RigidBody descendant, ReferenceFrame jacobianFrame)
   {
      return geometricJacobianHolder.getOrCreateGeometricJacobian(ancestor, descendant, jacobianFrame);
   }

   public long getOrCreateGeometricJacobian(InverseDynamicsJoint[] joints, ReferenceFrame jacobianFrame)
   {
      return geometricJacobianHolder.getOrCreateGeometricJacobian(joints, jacobianFrame);
   }

   /**
    * Return a jacobian previously created with the getOrCreate method using a jacobianId.
    * @param jacobianId
    * @return
    */
   public GeometricJacobian getJacobian(long jacobianId)
   {
      return geometricJacobianHolder.getJacobian(jacobianId);
   }

   public YoGraphicsListRegistry getYoGraphicsListRegistry()
   {
      return yoGraphicsListRegistry;
   }

   public List<? extends ContactablePlaneBody> getContactablePlaneBodies()
   {
      return contactablePlaneBodies;
   }

   public PlaneContactWrenchProcessor getPlaneContactWrenchProcessor()
   {
      return planeContactWrenchProcessor;
   }

   public WrenchVisualizer getWrenchVisualizer()
   {
      return wrenchVisualizer;
   }

   public YoFrameVector getYoDesiredMomentumRateLinear()
   {
      return yoDesiredMomentumRateLinear;
   }

   public YoFrameVector getYoAchievedMomentumRateLinear()
   {
      return yoAchievedMomentumRateLinear;
   }

   public YoFrameVector getYoResidualRootJointForce()
   {
      return yoResidualRootJointForce;
   }

   public YoFrameVector getYoResidualRootJointTorque()
   {
      return yoResidualRootJointTorque;
   }

   public JointIndexHandler getJointIndexHandler()
   {
      return jointIndexHandler;
   }
}
