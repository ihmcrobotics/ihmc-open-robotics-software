package us.ihmc.commonWalkingControlModules.controllerCore;

import java.util.List;

import us.ihmc.commonWalkingControlModules.configurations.JointPrivilegedConfigurationParameters;
import us.ihmc.commonWalkingControlModules.momentumBasedController.GeometricJacobianHolder;
import us.ihmc.commonWalkingControlModules.momentumBasedController.PlaneContactWrenchProcessor;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.JointIndexHandler;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.MomentumOptimizationSettings;
import us.ihmc.commonWalkingControlModules.visualizer.WrenchVisualizer;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.FloatingInverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.GeometricJacobian;
import us.ihmc.robotics.screwTheory.InverseDynamicsCalculator;
import us.ihmc.robotics.screwTheory.InverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.robotics.screwTheory.SpatialAccelerationCalculator;
import us.ihmc.robotics.screwTheory.TotalMassCalculator;
import us.ihmc.robotics.screwTheory.TwistCalculator;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;
import us.ihmc.sensorProcessing.frames.ReferenceFrames;

public class WholeBodyControlCoreToolbox
{
   private final int nBasisVectorsPerContactPoint;
   private final int nContactPointsPerContactableBody;
   private final int nContactableBodies;
   private final int rhoSize;

   private final GeometricJacobianHolder geometricJacobianHolder;
   private final TwistCalculator twistCalculator;
   private final SpatialAccelerationCalculator spatialAccelerationCalculator;
   private final InverseDynamicsCalculator inverseDynamicsCalculator;
   private final double controlDT;
   private final FullRobotModel fullRobotModel;
   private final RigidBody[] controlledBodies;
   private final YoGraphicsListRegistry yoGraphicsListRegistry;
   private final List<? extends ContactablePlaneBody> contactablePlaneBodies;
   private final ReferenceFrames referenceFrames;
   private final double gravityZ;
   private final double totalRobotMass;

   private final MomentumOptimizationSettings momentumOptimizationSettings;
   private final JointPrivilegedConfigurationParameters jointPrivilegedConfigurationParameters;

   private final PlaneContactWrenchProcessor planeContactWrenchProcessor;
   private final WrenchVisualizer wrenchVisualizer;

   private final YoFrameVector yoDesiredMomentumRateLinear;
   private final YoFrameVector yoAchievedMomentumRateLinear;
   private final YoFrameVector yoDesiredMomentumRateAngular;
   private final YoFrameVector yoAchievedMomentumRateAngular;

   private final YoFrameVector yoResidualRootJointForce;
   private final YoFrameVector yoResidualRootJointTorque;

   private final JointIndexHandler jointIndexHandler;

   public WholeBodyControlCoreToolbox(FullRobotModel fullRobotModel, RigidBody[] controlledBodies, InverseDynamicsJoint[] controlledJoints,
         MomentumOptimizationSettings momentumOptimizationSettings, JointPrivilegedConfigurationParameters jointPrivilegedConfigurationParameters,
         ReferenceFrames referenceFrames, double controlDT, double gravityZ, GeometricJacobianHolder geometricJacobianHolder, TwistCalculator twistCalculator,
         List<? extends ContactablePlaneBody> contactablePlaneBodies, YoGraphicsListRegistry yoGraphicsListRegistry, YoVariableRegistry registry)
   {
      this.fullRobotModel = fullRobotModel;
      this.controlledBodies = controlledBodies;
      this.momentumOptimizationSettings = momentumOptimizationSettings;
      this.jointPrivilegedConfigurationParameters = jointPrivilegedConfigurationParameters;
      this.referenceFrames = referenceFrames;
      this.controlDT = controlDT;
      this.gravityZ = gravityZ;
      this.twistCalculator = twistCalculator;
      this.geometricJacobianHolder = geometricJacobianHolder;
      this.contactablePlaneBodies = contactablePlaneBodies;
      this.yoGraphicsListRegistry = yoGraphicsListRegistry;
      this.nBasisVectorsPerContactPoint = momentumOptimizationSettings.getNumberOfBasisVectorsPerContactPoint();
      this.nContactPointsPerContactableBody = momentumOptimizationSettings.getNumberOfContactPointsPerContactableBody();
      this.nContactableBodies = momentumOptimizationSettings.getNumberOfContactableBodies();
      this.rhoSize = momentumOptimizationSettings.getRhoSize();

      if (contactablePlaneBodies != null)
      {
         this.planeContactWrenchProcessor = new PlaneContactWrenchProcessor(contactablePlaneBodies, yoGraphicsListRegistry, registry);
         this.wrenchVisualizer = WrenchVisualizer
               .createWrenchVisualizerWithContactableBodies("DesiredExternalWrench", contactablePlaneBodies, 1.0, yoGraphicsListRegistry, registry);
      }
      else
      {
         this.planeContactWrenchProcessor = null;
         this.wrenchVisualizer = null;
      }

      if (referenceFrames != null)
      {
         this.yoDesiredMomentumRateLinear = new YoFrameVector("desiredMomentumRateLinear", referenceFrames.getCenterOfMassFrame(), registry);
         this.yoAchievedMomentumRateLinear = new YoFrameVector("achievedMomentumRateLinear", referenceFrames.getCenterOfMassFrame(), registry);
         this.yoDesiredMomentumRateAngular = new YoFrameVector("desiredMomentumRateAngular", referenceFrames.getCenterOfMassFrame(), registry);
         this.yoAchievedMomentumRateAngular = new YoFrameVector("achievedMomentumRateAngular", referenceFrames.getCenterOfMassFrame(), registry);
      }
      else
      {
         yoDesiredMomentumRateLinear = null;
         yoAchievedMomentumRateLinear = null;
         yoDesiredMomentumRateAngular = null;
         yoAchievedMomentumRateAngular = null;
      }

      this.yoResidualRootJointForce = new YoFrameVector("residualRootJointForce", ReferenceFrame.getWorldFrame(), registry);
      this.yoResidualRootJointTorque = new YoFrameVector("residualRootJointTorque", ReferenceFrame.getWorldFrame(), registry);

      this.jointIndexHandler = new JointIndexHandler(controlledJoints);
      this.inverseDynamicsCalculator = new InverseDynamicsCalculator(twistCalculator, gravityZ);
      this.spatialAccelerationCalculator = inverseDynamicsCalculator.getSpatialAccelerationCalculator();

      if (fullRobotModel != null)
         totalRobotMass = TotalMassCalculator.computeSubTreeMass(fullRobotModel.getElevator());
      else
         totalRobotMass = TotalMassCalculator.computeSubTreeMass(ScrewTools.getRootBody(controlledJoints[0].getSuccessor()));
   }

   public static WholeBodyControlCoreToolbox createForInverseKinematicsOnly(FullRobotModel fullRobotModel, InverseDynamicsJoint[] controlledJoints,
         JointPrivilegedConfigurationParameters jointPrivilegedConfigurationParameters, CommonHumanoidReferenceFrames referenceFrames, double controlDT,
         GeometricJacobianHolder geometricJacobianHolder, TwistCalculator twistCalculator, MomentumOptimizationSettings momentumOptimizationSettings)
   {
      WholeBodyControlCoreToolbox ret = new WholeBodyControlCoreToolbox(fullRobotModel, null, controlledJoints, momentumOptimizationSettings,
            jointPrivilegedConfigurationParameters, referenceFrames, controlDT, Double.NaN, geometricJacobianHolder, twistCalculator, null, null, null);
      return ret;
   }

   public MomentumOptimizationSettings getMomentumOptimizationSettings()
   {
      return momentumOptimizationSettings;
   }

   public JointPrivilegedConfigurationParameters getJointPrivilegedConfigurationParameters()
   {
      return jointPrivilegedConfigurationParameters;
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

   public FloatingInverseDynamicsJoint getRobotRootJoint()
   {
      return fullRobotModel.getRootJoint();
   }

   public FullRobotModel getFullRobotModel()
   {
      return fullRobotModel;
   }

   public RigidBody[] getControlledBodies()
   {
      return controlledBodies;
   }

   public double getControlDT()
   {
      return controlDT;
   }

   public ReferenceFrame getCenterOfMassFrame()
   {
      return referenceFrames.getCenterOfMassFrame();
   }

   public double getGravityZ()
   {
      return gravityZ;
   }

   public double getTotalRobotMass()
   {
      return totalRobotMass;
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

   public YoFrameVector getYoDesiredMomentumRateAngular()
   {
      return yoDesiredMomentumRateAngular;
   }

   public YoFrameVector getYoAchievedMomentumRateAngular()
   {
      return yoAchievedMomentumRateAngular;
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

   public int getNumberOfBasisVectorsPerContactPoint()
   {
      return nBasisVectorsPerContactPoint;
   }

   public int getNumberOfContactPointsPerContactableBody()
   {
      return nContactPointsPerContactableBody;
   }

   public int getNumberOfContactableBodies()
   {
      return nContactableBodies;
   }

   public int getRhoSize()
   {
      return rhoSize;
   }

   public ReferenceFrames getReferenceFrames()
   {
      return referenceFrames;
   }
}
