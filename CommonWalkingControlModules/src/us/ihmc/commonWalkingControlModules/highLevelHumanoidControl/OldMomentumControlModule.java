package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl;

import java.util.ArrayList;
import java.util.Collection;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

import javax.vecmath.Vector3d;

import org.ejml.data.DenseMatrix64F;
import org.ejml.factory.LinearSolver;

import us.ihmc.commonWalkingControlModules.WrenchDistributorTools;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.PlaneContactState;
import us.ihmc.commonWalkingControlModules.wrenchDistribution.GroundReactionWrenchDistributor;
import us.ihmc.commonWalkingControlModules.wrenchDistribution.GroundReactionWrenchDistributorInputData;
import us.ihmc.commonWalkingControlModules.wrenchDistribution.GroundReactionWrenchDistributorOutputData;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumRateOfChangeData;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumSolver;
import us.ihmc.commonWalkingControlModules.momentumBasedController.RootJointAccelerationData;
import us.ihmc.commonWalkingControlModules.momentumBasedController.TaskspaceConstraintData;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.utilities.math.MathTools;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.GeometricJacobian;
import us.ihmc.utilities.screwTheory.InverseDynamicsJoint;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.ScrewTools;
import us.ihmc.utilities.screwTheory.SixDoFJoint;
import us.ihmc.utilities.screwTheory.SpatialForceVector;
import us.ihmc.utilities.screwTheory.TotalMassCalculator;
import us.ihmc.utilities.screwTheory.TotalWrenchCalculator;
import us.ihmc.utilities.screwTheory.TwistCalculator;
import us.ihmc.utilities.screwTheory.Wrench;

import com.yobotics.simulationconstructionset.BooleanYoVariable;
import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.math.filter.AlphaFilteredYoFrameVector;
import com.yobotics.simulationconstructionset.util.math.filter.AlphaFilteredYoVariable;
import com.yobotics.simulationconstructionset.util.math.frames.YoFrameVector;

public class OldMomentumControlModule implements MomentumControlModule
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final MomentumSolver solver;
   private final GroundReactionWrenchDistributor groundReactionWrenchDistributor;

   private final YoFrameVector unfilteredDesiredGroundReactionTorque;
   private final YoFrameVector unfilteredDesiredGroundReactionForce;

   private final AlphaFilteredYoFrameVector desiredGroundReactionTorque;
   private final AlphaFilteredYoFrameVector desiredGroundReactionForce;

   private final SpatialForceVector gravitationalWrench;


   private final List<ContactablePlaneBody> contactablePlaneBodies;
   private final Map<ContactablePlaneBody, Wrench> externalWrenches = new LinkedHashMap<ContactablePlaneBody, Wrench>();

   protected final DoubleYoVariable alphaGroundReactionWrench = new DoubleYoVariable("alphaGroundReactionWrench", registry);
   private final BooleanYoVariable groundReactionWrenchFilterResetRequest = new BooleanYoVariable("groundReactionWrenchFilterResetRequest", registry);
   private final double controlDT;
   private final SpatialForceVector desiredCentroidalMomentumRate = new SpatialForceVector();


   public OldMomentumControlModule(SixDoFJoint rootJoint, Collection<? extends ContactablePlaneBody> contactablePlaneBodies, double gravityZ,
                                   GroundReactionWrenchDistributor groundReactionWrenchDistributor, ReferenceFrame centerOfMassFrame, double controlDT,
                                   TwistCalculator twistCalculator, LinearSolver<DenseMatrix64F> jacobianSolver, YoVariableRegistry parentRegistry)
   {
      MathTools.checkIfInRange(gravityZ, 0.0, Double.POSITIVE_INFINITY);

      this.solver = new MomentumSolver(rootJoint, rootJoint.getPredecessor(), centerOfMassFrame, twistCalculator, jacobianSolver, controlDT, registry);

      double totalMass = TotalMassCalculator.computeMass(ScrewTools.computeSupportAndSubtreeSuccessors(rootJoint.getSuccessor()));

      this.contactablePlaneBodies = new ArrayList<ContactablePlaneBody>(contactablePlaneBodies);
      this.groundReactionWrenchDistributor = groundReactionWrenchDistributor;

      this.unfilteredDesiredGroundReactionTorque = new YoFrameVector("unfilteredDesiredGroundReactionTorque", centerOfMassFrame, registry);
      this.unfilteredDesiredGroundReactionForce = new YoFrameVector("unfilteredDesiredGroundReactionForce", centerOfMassFrame, registry);

      this.controlDT = controlDT;

      this.desiredGroundReactionTorque = AlphaFilteredYoFrameVector.createAlphaFilteredYoFrameVector("desiredGroundReactionTorque", "", registry,
              alphaGroundReactionWrench, unfilteredDesiredGroundReactionTorque);
      this.desiredGroundReactionForce = AlphaFilteredYoFrameVector.createAlphaFilteredYoFrameVector("desiredGroundReactionForce", "", registry,
              alphaGroundReactionWrench, unfilteredDesiredGroundReactionForce);

      gravitationalWrench = new SpatialForceVector(centerOfMassFrame, new Vector3d(0.0, 0.0, totalMass * gravityZ), new Vector3d());

      parentRegistry.addChild(registry);
   }

   public void setGroundReactionWrenchBreakFrequencyHertz(double groundReactionWrenchBreakFrequencyHertz)
   {
      alphaGroundReactionWrench.set(AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(groundReactionWrenchBreakFrequencyHertz, controlDT));

   }

   public void initialize()
   {
      solver.initialize();
   }

   public void reset()
   {
      solver.reset();
      externalWrenches.clear();
   }

   public void compute(RootJointAccelerationData rootJointAccelerationData, MomentumRateOfChangeData momentumRateOfChangeData,
                       LinkedHashMap<ContactablePlaneBody, ? extends PlaneContactState> contactStates, RobotSide upcomingSupportLeg)
   {
      solver.compute();
      solver.solve(rootJointAccelerationData.getAccelerationSubspace(), rootJointAccelerationData.getAccelerationMultipliers(),
                   momentumRateOfChangeData.getMomentumSubspace(), momentumRateOfChangeData.getMomentumMultipliers());

      SpatialForceVector totalGroundReactionWrench = new SpatialForceVector();
      solver.getRateOfChangeOfMomentum(totalGroundReactionWrench);
      totalGroundReactionWrench.add(gravitationalWrench);

      unfilteredDesiredGroundReactionTorque.set(totalGroundReactionWrench.getAngularPartCopy());
      unfilteredDesiredGroundReactionForce.set(totalGroundReactionWrench.getLinearPartCopy());

      if (groundReactionWrenchFilterResetRequest.getBooleanValue())
      {
         desiredGroundReactionTorque.reset();
         desiredGroundReactionForce.reset();
         groundReactionWrenchFilterResetRequest.set(false);
      }

      desiredGroundReactionTorque.update();
      desiredGroundReactionForce.update();

      // TODO: copies:
      totalGroundReactionWrench.setAngularPart(desiredGroundReactionTorque.getFrameVectorCopy().getVector());
      totalGroundReactionWrench.setLinearPart(desiredGroundReactionForce.getFrameVectorCopy().getVector());

      GroundReactionWrenchDistributorInputData groundReactionWrenchDistributorInputData = new GroundReactionWrenchDistributorInputData();
      groundReactionWrenchDistributorInputData.reset();

      for (ContactablePlaneBody contactablePlaneBody : contactablePlaneBodies)
      {
         PlaneContactState contactState = contactStates.get(contactablePlaneBody);
         List<FramePoint> footContactPoints = contactState.getContactPoints();

         if (footContactPoints.size() > 0)
         {
            groundReactionWrenchDistributorInputData.addContact(contactState);
         }
      }

      groundReactionWrenchDistributorInputData.setSpatialForceVectorAndUpcomingSupportSide(totalGroundReactionWrench, upcomingSupportLeg);

      GroundReactionWrenchDistributorOutputData distributedWrenches = new GroundReactionWrenchDistributorOutputData();
      groundReactionWrenchDistributor.solve(distributedWrenches, groundReactionWrenchDistributorInputData);

      for (ContactablePlaneBody contactablePlaneBody : contactablePlaneBodies)
      {
         RigidBody rigidBody = contactablePlaneBody.getRigidBody();
         PlaneContactState contactState = contactStates.get(contactablePlaneBody);
         List<FramePoint> footContactPoints = contactState.getContactPoints();

         if (footContactPoints.size() > 0)
         {
            FrameVector force = distributedWrenches.getForce(contactState);
            FramePoint2d cop = distributedWrenches.getCenterOfPressure(contactState);
            double normalTorque = distributedWrenches.getNormalTorque(contactState);

            Wrench groundReactionWrench = new Wrench(rigidBody.getBodyFixedFrame(), contactState.getPlaneFrame());
            externalWrenches.put(contactablePlaneBody, groundReactionWrench);
            WrenchDistributorTools.computeWrench(groundReactionWrench, force, cop, normalTorque);
            groundReactionWrench.changeFrame(rigidBody.getBodyFixedFrame());
         }
      }

      Wrench admissibleGroundReactionWrench = TotalWrenchCalculator.computeTotalWrench(externalWrenches.values(),
                                                 totalGroundReactionWrench.getExpressedInFrame());
      desiredCentroidalMomentumRate.set(admissibleGroundReactionWrench);
      desiredCentroidalMomentumRate.sub(gravitationalWrench);

      solver.solve(desiredCentroidalMomentumRate);
   }

   public void resetGroundReactionWrenchFilter()
   {
      groundReactionWrenchFilterResetRequest.set(true);
   }

   public void setDesiredJointAcceleration(InverseDynamicsJoint joint, DenseMatrix64F jointAcceleration)
   {
      solver.setDesiredJointAcceleration(joint, jointAcceleration);
   }

   public void setDesiredSpatialAcceleration(GeometricJacobian jacobian, TaskspaceConstraintData taskspaceConstraintData)
   {
      solver.setDesiredSpatialAcceleration(jacobian, taskspaceConstraintData);
   }

   public SpatialForceVector getDesiredCentroidalMomentumRate()
   {
      return desiredCentroidalMomentumRate;
   }

   public Map<ContactablePlaneBody, Wrench> getExternalWrenches()
   {
      return externalWrenches;
   }
}
