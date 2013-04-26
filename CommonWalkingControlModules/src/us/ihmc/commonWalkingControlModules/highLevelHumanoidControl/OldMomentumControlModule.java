package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl;

import com.yobotics.simulationconstructionset.BooleanYoVariable;
import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicPosition;
import com.yobotics.simulationconstructionset.util.math.filter.AlphaFilteredYoFramePoint2d;
import com.yobotics.simulationconstructionset.util.math.filter.AlphaFilteredYoFrameVector;
import com.yobotics.simulationconstructionset.util.math.filter.AlphaFilteredYoVariable;
import com.yobotics.simulationconstructionset.util.math.frames.YoFramePoint;
import com.yobotics.simulationconstructionset.util.math.frames.YoFramePoint2d;
import com.yobotics.simulationconstructionset.util.math.frames.YoFrameVector;
import org.ejml.data.DenseMatrix64F;
import org.ejml.factory.LinearSolver;
import us.ihmc.commonWalkingControlModules.WrenchDistributorTools;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.PlaneContactState;
import us.ihmc.commonWalkingControlModules.controlModules.GroundReactionWrenchDistributor;
import us.ihmc.commonWalkingControlModules.controlModules.GroundReactionWrenchDistributorInputData;
import us.ihmc.commonWalkingControlModules.controlModules.GroundReactionWrenchDistributorOutputData;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumRateOfChangeData;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumSolver;
import us.ihmc.commonWalkingControlModules.momentumBasedController.RootJointAccelerationData;
import us.ihmc.commonWalkingControlModules.momentumBasedController.TaskspaceConstraintData;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.utilities.math.MathTools;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.*;

import javax.vecmath.Vector3d;
import java.util.*;

public class OldMomentumControlModule
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final MomentumSolver solver;
   private final GroundReactionWrenchDistributor groundReactionWrenchDistributor;

   private final YoFrameVector unfilteredDesiredGroundReactionTorque;
   private final YoFrameVector unfilteredDesiredGroundReactionForce;
   
   private final AlphaFilteredYoFrameVector desiredGroundReactionTorque;
   private final AlphaFilteredYoFrameVector desiredGroundReactionForce;


   private final LinkedHashMap<ContactablePlaneBody, YoFramePoint> filteredCentersOfPressureWorld = new LinkedHashMap<ContactablePlaneBody, YoFramePoint>();
   private final LinkedHashMap<ContactablePlaneBody, YoFramePoint2d> unfilteredCentersOfPressure2d = new LinkedHashMap<ContactablePlaneBody, YoFramePoint2d>();
   private final LinkedHashMap<ContactablePlaneBody, AlphaFilteredYoFramePoint2d> filteredCentersOfPressure2d = new LinkedHashMap<ContactablePlaneBody,
         AlphaFilteredYoFramePoint2d>();
   
   private final DoubleYoVariable alphaCoP = new DoubleYoVariable("alphaCoP", registry);
   private final LinkedHashMap<ContactablePlaneBody, BooleanYoVariable> copFilterResetRequests = new LinkedHashMap<ContactablePlaneBody, BooleanYoVariable>();

   private final SpatialForceVector gravitationalWrench;


   private final List<ContactablePlaneBody> contactablePlaneBodies;

   protected final DoubleYoVariable alphaGroundReactionWrench = new DoubleYoVariable("alphaGroundReactionWrench", registry);
   private final BooleanYoVariable groundReactionWrenchFilterResetRequest = new BooleanYoVariable("groundReactionWrenchFilterResetRequest", registry);
   private final double controlDT;


   public OldMomentumControlModule(SixDoFJoint rootJoint, Collection<? extends ContactablePlaneBody> contactablePlaneBodies, double gravityZ,
                                   GroundReactionWrenchDistributor groundReactionWrenchDistributor, ReferenceFrame centerOfMassFrame, double controlDT, TwistCalculator twistCalculator, LinearSolver<DenseMatrix64F> jacobianSolver,
                                   DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry, YoVariableRegistry parentRegistry)
   {
      MathTools.checkIfInRange(gravityZ, 0.0, Double.POSITIVE_INFINITY);

      this.solver = new MomentumSolver(rootJoint, rootJoint.getPredecessor(), centerOfMassFrame, twistCalculator, jacobianSolver, controlDT, registry);

      double totalMass = TotalMassCalculator.computeMass(ScrewTools.computeSupportAndSubtreeBodies(false, rootJoint.getSuccessor()));

      this.contactablePlaneBodies = new ArrayList<ContactablePlaneBody>(contactablePlaneBodies);
      this.groundReactionWrenchDistributor = groundReactionWrenchDistributor;

      this.unfilteredDesiredGroundReactionTorque = new YoFrameVector("unfilteredDesiredGroundReactionTorque", centerOfMassFrame, registry);
      this.unfilteredDesiredGroundReactionForce = new YoFrameVector("unfilteredDesiredGroundReactionForce", centerOfMassFrame, registry);

      this.controlDT = controlDT;

      this.desiredGroundReactionTorque = AlphaFilteredYoFrameVector.createAlphaFilteredYoFrameVector("desiredGroundReactionTorque", "", registry,
            alphaGroundReactionWrench, unfilteredDesiredGroundReactionTorque);
      this.desiredGroundReactionForce = AlphaFilteredYoFrameVector.createAlphaFilteredYoFrameVector("desiredGroundReactionForce", "", registry,
            alphaGroundReactionWrench, unfilteredDesiredGroundReactionForce);

      for (ContactablePlaneBody contactableBody : contactablePlaneBodies)
      {
         String copName = contactableBody.getRigidBody().getName() + "CoP";
         String listName = "cops";

         copFilterResetRequests.put(contactableBody, new BooleanYoVariable(copName + "FilterResetRequest", registry));

         YoFramePoint2d cop2d = new YoFramePoint2d(copName + "2d", "", contactableBody.getPlaneFrame(), registry);
         unfilteredCentersOfPressure2d.put(contactableBody, cop2d);

         AlphaFilteredYoFramePoint2d filteredCoP2d = AlphaFilteredYoFramePoint2d.createAlphaFilteredYoFramePoint2d(copName + "2dFilt", "", registry, alphaCoP,
               cop2d);
         filteredCentersOfPressure2d.put(contactableBody, filteredCoP2d);

         YoFramePoint cop = new YoFramePoint(copName, ReferenceFrame.getWorldFrame(), registry);
         filteredCentersOfPressureWorld.put(contactableBody, cop);

         if (dynamicGraphicObjectsListRegistry != null)
         {
            DynamicGraphicPosition copViz = cop.createDynamicGraphicPosition(copName, 0.005, YoAppearance.Navy(), DynamicGraphicPosition.GraphicType.BALL);
            dynamicGraphicObjectsListRegistry.registerDynamicGraphicObject(listName, copViz);
            dynamicGraphicObjectsListRegistry.registerArtifact(listName, copViz.createArtifact());
         }
      }

      gravitationalWrench = new SpatialForceVector(centerOfMassFrame, new Vector3d(0.0, 0.0, totalMass * gravityZ), new Vector3d());

      parentRegistry.addChild(registry);
   }

   public void setGroundReactionWrenchBreakFrequencyHertz(double groundReactionWrenchBreakFrequencyHertz)
   {
      alphaGroundReactionWrench.set(AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(groundReactionWrenchBreakFrequencyHertz, controlDT));

   }

   public SpatialForceVector computeDesiredAccelerationsAndExternalWrenches(RootJointAccelerationData rootJointAccelerationData, MomentumRateOfChangeData
         momentumRateOfChangeData, LinkedHashMap<ContactablePlaneBody, ? extends PlaneContactState> contactStates, Map<ContactablePlaneBody, Wrench> externalWrenches, RobotSide upcomingSupportLeg)
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

      List<Wrench> wrenches = new ArrayList<Wrench>();
      List<FramePoint2d> cops = new ArrayList<FramePoint2d>();

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

            unfilteredCentersOfPressure2d.get(contactablePlaneBody).set(cop);

            AlphaFilteredYoFramePoint2d filteredCoP2d = filteredCentersOfPressure2d.get(contactablePlaneBody);
            BooleanYoVariable copFilterResetRequest = copFilterResetRequests.get(contactablePlaneBody);
            if (copFilterResetRequest.getBooleanValue())
            {
               filteredCoP2d.reset();
               copFilterResetRequest.set(false);
            }

            filteredCoP2d.update();
            filteredCoP2d.getFramePoint2d(cop);

            cops.add(cop);
            FramePoint cop3d = cop.toFramePoint();
            cop3d.changeFrame(ReferenceFrame.getWorldFrame());
            filteredCentersOfPressureWorld.get(contactablePlaneBody).set(cop3d);

            Wrench groundReactionWrench = new Wrench(rigidBody.getBodyFixedFrame(), contactState.getPlaneFrame());
            WrenchDistributorTools.computeWrench(groundReactionWrench, force, cop, normalTorque);
            groundReactionWrench.changeFrame(rigidBody.getBodyFixedFrame());
            wrenches.add(groundReactionWrench);
            externalWrenches.put(contactablePlaneBody, groundReactionWrench);
         } else
         {
            resetCoPFilter(contactablePlaneBody);
            filteredCentersOfPressureWorld.get(contactablePlaneBody).setToNaN();
         }
      }

      Wrench admissibleGroundReactionWrench = TotalWrenchCalculator.computeTotalWrench(wrenches, totalGroundReactionWrench.getExpressedInFrame());
      SpatialForceVector desiredCentroidalMomentumRate = new SpatialForceVector();
      desiredCentroidalMomentumRate.set(admissibleGroundReactionWrench);
      desiredCentroidalMomentumRate.sub(gravitationalWrench);

      solver.solve(desiredCentroidalMomentumRate);
      return desiredCentroidalMomentumRate;
   }

   public void resetGroundReactionWrenchFilter()
   {
      groundReactionWrenchFilterResetRequest.set(true);
   }

   public void resetCoPFilter(ContactablePlaneBody contactableBody)
   {
      copFilterResetRequests.get(contactableBody).set(true);
   }

   public void reset()
   {
      solver.reset();
   }

   public void initialize()
   {
      solver.initialize();
   }

   public FramePoint2d getCoP(ContactablePlaneBody contactablePlaneBody)
   {
      return filteredCentersOfPressure2d.get(contactablePlaneBody).getFramePoint2dCopy();
   }

   public void setDesiredJointAcceleration(InverseDynamicsJoint joint, DenseMatrix64F jointAcceleration)
   {
      solver.setDesiredJointAcceleration(joint, jointAcceleration);
   }

   public void setDesiredSpatialAcceleration(GeometricJacobian spineJacobian, TaskspaceConstraintData taskspaceConstraintData)
   {
      solver.setDesiredSpatialAcceleration(spineJacobian, taskspaceConstraintData);
   }
}