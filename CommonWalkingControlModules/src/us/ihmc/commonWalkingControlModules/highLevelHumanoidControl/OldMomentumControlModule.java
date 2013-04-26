package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl;

import com.yobotics.simulationconstructionset.BooleanYoVariable;
import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicPosition;
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


   private final LinkedHashMap<ContactablePlaneBody, YoFramePoint> centersOfPressureWorld = new LinkedHashMap<ContactablePlaneBody, YoFramePoint>();
   private final LinkedHashMap<ContactablePlaneBody, YoFramePoint2d> centersOfPressure2d = new LinkedHashMap<ContactablePlaneBody, YoFramePoint2d>();

   private final SpatialForceVector gravitationalWrench;


   private final List<ContactablePlaneBody> contactablePlaneBodies;
   private final Map<ContactablePlaneBody, Wrench> externalWrenches = new HashMap<ContactablePlaneBody, Wrench>();

   protected final DoubleYoVariable alphaGroundReactionWrench = new DoubleYoVariable("alphaGroundReactionWrench", registry);
   private final BooleanYoVariable groundReactionWrenchFilterResetRequest = new BooleanYoVariable("groundReactionWrenchFilterResetRequest", registry);
   private final double controlDT;
   private final SpatialForceVector desiredCentroidalMomentumRate = new SpatialForceVector();


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

         YoFramePoint2d cop2d = new YoFramePoint2d(copName + "2d", "", contactableBody.getPlaneFrame(), registry);
         centersOfPressure2d.put(contactableBody, cop2d);

         YoFramePoint cop = new YoFramePoint(copName, ReferenceFrame.getWorldFrame(), registry);
         centersOfPressureWorld.put(contactableBody, cop);

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

   public void compute(RootJointAccelerationData rootJointAccelerationData, MomentumRateOfChangeData
         momentumRateOfChangeData, LinkedHashMap<ContactablePlaneBody, ? extends PlaneContactState> contactStates, RobotSide upcomingSupportLeg)
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

            centersOfPressure2d.get(contactablePlaneBody).set(cop);

            FramePoint cop3d = cop.toFramePoint();
            cop3d.changeFrame(ReferenceFrame.getWorldFrame());
            centersOfPressureWorld.get(contactablePlaneBody).set(cop3d);

            Wrench groundReactionWrench = new Wrench(rigidBody.getBodyFixedFrame(), contactState.getPlaneFrame());
            WrenchDistributorTools.computeWrench(groundReactionWrench, force, cop, normalTorque);
            groundReactionWrench.changeFrame(rigidBody.getBodyFixedFrame());
            externalWrenches.put(contactablePlaneBody, groundReactionWrench);
         } else
         {
            centersOfPressureWorld.get(contactablePlaneBody).setToNaN();
         }
      }

      Wrench admissibleGroundReactionWrench = TotalWrenchCalculator.computeTotalWrench(externalWrenches.values(), totalGroundReactionWrench.getExpressedInFrame());
      desiredCentroidalMomentumRate.set(admissibleGroundReactionWrench);
      desiredCentroidalMomentumRate.sub(gravitationalWrench);

      solver.solve(desiredCentroidalMomentumRate);
   }

   public void resetGroundReactionWrenchFilter()
   {
      groundReactionWrenchFilterResetRequest.set(true);
   }

   public void reset()
   {
      solver.reset();
      externalWrenches.clear();
   }

   public void initialize()
   {
      solver.initialize();
   }

   public FramePoint2d getCoP(ContactablePlaneBody contactablePlaneBody)
   {
      return centersOfPressure2d.get(contactablePlaneBody).getFramePoint2dCopy();
   }

   public void setDesiredJointAcceleration(InverseDynamicsJoint joint, DenseMatrix64F jointAcceleration)
   {
      solver.setDesiredJointAcceleration(joint, jointAcceleration);
   }

   public void setDesiredSpatialAcceleration(GeometricJacobian spineJacobian, TaskspaceConstraintData taskspaceConstraintData)
   {
      solver.setDesiredSpatialAcceleration(spineJacobian, taskspaceConstraintData);
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