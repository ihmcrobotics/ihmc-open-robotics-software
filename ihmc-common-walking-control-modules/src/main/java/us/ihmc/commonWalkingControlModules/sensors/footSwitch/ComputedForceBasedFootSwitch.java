package us.ihmc.commonWalkingControlModules.sensors.footSwitch;

import org.ejml.data.DenseMatrix64F;
import org.ejml.factory.LinearSolverFactory;
import org.ejml.interfaces.linsol.LinearSolver;
import org.ejml.ops.CommonOps;

import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.Wrench;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotModels.FullLeggedRobotModel;
import us.ihmc.robotics.math.filters.GlitchFilteredYoBoolean;
import us.ihmc.robotics.robotSide.RobotSegment;
import us.ihmc.robotics.screwTheory.GeometricJacobian;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.robotics.sensors.FootSwitchInterface;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

/**
 * Computes the wrench at the sole frame using the Jacobian from Root to Sole and 
 * compares the linear Z force in World to the zForceThreshold to determine if the foot can be trusted
 * This class is useful if you don't have a contact sensor in the foot
 */
public class ComputedForceBasedFootSwitch<E extends Enum<E> & RobotSegment<E>> implements FootSwitchInterface
{
   private final String name = getClass().getSimpleName();
   private final YoVariableRegistry registry;
   private final GeometricJacobian jacobian;
   private final OneDoFJointBasics[] jointsFromRootToSole;

   private final DenseMatrix64F jacobianInverse;
   private final DenseMatrix64F footWrench = new DenseMatrix64F(6, 1);
   private final DenseMatrix64F jointTorques;
   
   private final GlitchFilteredYoBoolean isInContact;
   
   private final DoubleProvider contactThresholdForce;
   private final YoBoolean pastThreshold;
   private final YoDouble measuredZForce;
   private final FrameVector3D footForce = new FrameVector3D();
   private final Wrench wrench = new Wrench();
   
   private final LinearSolver<DenseMatrix64F> solver = LinearSolverFactory.pseudoInverse(true);
   private final double standingZForce;
   private MovingReferenceFrame soleFrame;
   
   public ComputedForceBasedFootSwitch(FullLeggedRobotModel<E> robotModel, E robotSegment, DoubleProvider contactThresholdForce, YoVariableRegistry parentRegistry)
   {
      String prefix = robotSegment.toString() + name;
      registry = new YoVariableRegistry(prefix);
      
      int filterWindowSize = 3;
      
      measuredZForce = new YoDouble(prefix + "measuredZForce", registry);
      pastThreshold = new YoBoolean(prefix + "PastFootswitchThreshold", registry);
      
      this.contactThresholdForce = contactThresholdForce;
      this.isInContact = new GlitchFilteredYoBoolean(prefix + "IsInContact", registry, pastThreshold, filterWindowSize);
      
      RigidBodyBasics body = robotModel.getRootBody();
      RigidBodyBasics foot = robotModel.getFoot(robotSegment);
      soleFrame = robotModel.getSoleFrame(robotSegment);
      jacobian = new GeometricJacobian(body, foot, soleFrame);
      
      jointsFromRootToSole = MultiBodySystemTools.createOneDoFJointPath(body, foot);
      
      jointTorques = new DenseMatrix64F(jointsFromRootToSole.length, 1);
      jacobianInverse = new DenseMatrix64F(jointsFromRootToSole.length, 3);
      
      parentRegistry.addChild(registry);
      
      double totalMass = robotModel.getTotalMass();
      standingZForce = totalMass * 9.81;
   }

   public void update()
   {
      for(int i = 0; i < jointsFromRootToSole.length; i++)
      {
         OneDoFJointBasics oneDoFJoint = jointsFromRootToSole[i];
         jointTorques.set(i, 0, oneDoFJoint.getTau());
      }
      
      jacobian.compute();

      solver.setA(jacobian.getJacobianMatrix());
      solver.invert(jacobianInverse);
      
      CommonOps.multTransA(jacobianInverse, jointTorques, footWrench);
      wrench.setIncludingFrame(jacobian.getJacobianFrame(), footWrench);
      
      footForce.setToZero(jacobian.getJacobianFrame());
      footForce.set(wrench.getLinearPart());
      footForce.changeFrame(ReferenceFrame.getWorldFrame());
      
      measuredZForce.set(footForce.getZ() * -1.0);
      pastThreshold.set(measuredZForce.getDoubleValue() > contactThresholdForce.getValue());
      isInContact.update();
   }

   @Override
   public boolean hasFootHitGround()
   {
      update();
      return isInContact.getBooleanValue();
   }

   @Override
   public double computeFootLoadPercentage()
   {
      return Math.min(1.0, measuredZForce.getDoubleValue() / standingZForce);
   }

   @Override
   public void computeAndPackCoP(FramePoint2D copToPack)
   {
      copToPack.setToZero(soleFrame);
   }

   @Override
   public void updateCoP()
   {
      
   }

   @Override
   public void computeAndPackFootWrench(Wrench footWrenchToPack)
   {
      footWrenchToPack.setIncludingFrame(wrench);
   }

   @Override
   public ReferenceFrame getMeasurementFrame()
   {
      return jacobian.getJacobianFrame();
   }

   @Override
   public void reset()
   {
      isInContact.set(false);
   }

   @Override
   public boolean getForceMagnitudePastThreshhold()
   {
      return isInContact.getBooleanValue();
   }

   @Override
   public void setFootContactState(boolean hasFootHitGround)
   {
      isInContact.set(hasFootHitGround);
   }

   @Override
   public void trustFootSwitchInSwing(boolean trustFootSwitch)
   {
   }

   @Override
   public void trustFootSwitchInSupport(boolean trustFootSwitch)
   {
   }
}