package us.ihmc.commonWalkingControlModules.momentumBasedController.feedbackController;

import java.util.List;

import us.ihmc.SdfLoader.models.FullRobotModel;
import us.ihmc.commonWalkingControlModules.momentumBasedController.GeometricJacobianHolder;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.GeometricJacobian;
import us.ihmc.robotics.screwTheory.InverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.SixDoFJoint;
import us.ihmc.robotics.screwTheory.TwistCalculator;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicsListRegistry;

public class WholeBodyControlCoreToolbox
{
   private final GeometricJacobianHolder geometricJacobianHolder;
   private final TwistCalculator twistCalculator;
   private final double controlDT;
   private final FullRobotModel fullRobotModel;
   private final YoGraphicsListRegistry yoGraphicsListRegistry;
   private final List<? extends ContactablePlaneBody> contactablePlaneBodies;
   private final CommonHumanoidReferenceFrames referenceFrames;
   private final double gravityZ;

   public WholeBodyControlCoreToolbox(FullRobotModel fullRobotModel, CommonHumanoidReferenceFrames referenceFrames, double controlDT, double gravityZ,
         GeometricJacobianHolder geometricJacobianHolder, TwistCalculator twistCalculator, List<? extends ContactablePlaneBody> contactablePlaneBodies,
         YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this.fullRobotModel = fullRobotModel;
      this.referenceFrames = referenceFrames;
      this.controlDT = controlDT;
      this.gravityZ = gravityZ;
      this.twistCalculator = twistCalculator;
      this.geometricJacobianHolder = geometricJacobianHolder;
      this.contactablePlaneBodies = contactablePlaneBodies;
      this.yoGraphicsListRegistry = yoGraphicsListRegistry;
   }

   public TwistCalculator getTwistCalculator()
   {
      return twistCalculator;
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
}
