package us.ihmc.commonWalkingControlModules.momentumBasedController;

import java.util.*;

import javax.vecmath.Vector3d;

import org.ejml.data.DenseMatrix64F;
import org.ejml.factory.LinearSolver;

import us.ihmc.commonWalkingControlModules.WrenchDistributorTools;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.PlaneContactState;
import us.ihmc.commonWalkingControlModules.wrenchDistribution.CylindricalContactState;
import us.ihmc.commonWalkingControlModules.wrenchDistribution.GroundReactionWrenchDistributor;
import us.ihmc.commonWalkingControlModules.wrenchDistribution.GroundReactionWrenchDistributorInputData;
import us.ihmc.commonWalkingControlModules.wrenchDistribution.GroundReactionWrenchDistributorOutputData;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.utilities.math.MathTools;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.*;

import com.yobotics.simulationconstructionset.BooleanYoVariable;
import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;
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
   private final ReferenceFrame centerOfMassFrame;

   private final Map<RigidBody, Wrench> externalWrenchesToCompensateFor = new LinkedHashMap<RigidBody, Wrench>();

   private final Map<RigidBody, Wrench> externalWrenches = new LinkedHashMap<RigidBody, Wrench>();
   protected final DoubleYoVariable alphaGroundReactionWrench = new DoubleYoVariable("alphaGroundReactionWrench", registry);
   private final BooleanYoVariable groundReactionWrenchFilterResetRequest = new BooleanYoVariable("groundReactionWrenchFilterResetRequest", registry);
   private final double controlDT;
   private final SpatialForceVector desiredCentroidalMomentumRate = new SpatialForceVector();
   private final RootJointAccelerationData rootJointAccelerationData;
   private final MomentumRateOfChangeData momentumRateOfChangeData;
   private final SixDoFJoint rootJoint;
   private final DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry;

   private GroundReactionWrenchDistributorInputData wrenchDistributorInput = new GroundReactionWrenchDistributorInputData();


   public OldMomentumControlModule(SixDoFJoint rootJoint, double gravityZ, GroundReactionWrenchDistributor groundReactionWrenchDistributor,
                                   ReferenceFrame centerOfMassFrame, double controlDT, TwistCalculator twistCalculator,
                                   LinearSolver<DenseMatrix64F> jacobianSolver, YoVariableRegistry parentRegistry, DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry)
   {
      MathTools.checkIfInRange(gravityZ, 0.0, Double.POSITIVE_INFINITY);
      this.dynamicGraphicObjectsListRegistry=dynamicGraphicObjectsListRegistry;
      this.solver = new MomentumSolver(rootJoint, rootJoint.getPredecessor(), centerOfMassFrame, twistCalculator, jacobianSolver, controlDT, registry);

      double totalMass = TotalMassCalculator.computeMass(ScrewTools.computeSupportAndSubtreeSuccessors(rootJoint.getSuccessor()));

      this.groundReactionWrenchDistributor = groundReactionWrenchDistributor;

      this.unfilteredDesiredGroundReactionTorque = new YoFrameVector("unfilteredDesiredGroundReactionTorque", centerOfMassFrame, registry);
      this.unfilteredDesiredGroundReactionForce = new YoFrameVector("unfilteredDesiredGroundReactionForce", centerOfMassFrame, registry);

      this.controlDT = controlDT;

      this.desiredGroundReactionTorque = AlphaFilteredYoFrameVector.createAlphaFilteredYoFrameVector("desiredGroundReactionTorque", "", registry,
              alphaGroundReactionWrench, unfilteredDesiredGroundReactionTorque);
      this.desiredGroundReactionForce = AlphaFilteredYoFrameVector.createAlphaFilteredYoFrameVector("desiredGroundReactionForce", "", registry,
              alphaGroundReactionWrench, unfilteredDesiredGroundReactionForce);

      this.gravitationalWrench = new SpatialForceVector(centerOfMassFrame, new Vector3d(0.0, 0.0, -totalMass * gravityZ), new Vector3d());

      this.rootJointAccelerationData = new RootJointAccelerationData(rootJoint.getFrameAfterJoint(), rootJoint.getFrameBeforeJoint(),
              rootJoint.getFrameAfterJoint());
      this.momentumRateOfChangeData = new MomentumRateOfChangeData(centerOfMassFrame);
      this.rootJoint = rootJoint;
      this.centerOfMassFrame = centerOfMassFrame;

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
      externalWrenchesToCompensateFor.clear();
   }

   public void compute(Map<ContactablePlaneBody, ? extends PlaneContactState> planeContactStates, RobotSide upcomingSupportLeg,
                       Map<RigidBody, ? extends CylindricalContactState> cylinderContactStates)
   {
      solver.compute();
      solver.solve(rootJointAccelerationData.getAccelerationSubspace(), rootJointAccelerationData.getAccelerationMultipliers(),
                   momentumRateOfChangeData.getMomentumSubspace(), momentumRateOfChangeData.getMomentumMultipliers());

      SpatialForceVector totalGroundReactionWrench = new SpatialForceVector();
      solver.getRateOfChangeOfMomentum(totalGroundReactionWrench);
      totalGroundReactionWrench.sub(gravitationalWrench);

      for (Wrench externalWrenchToCompensateFor : externalWrenchesToCompensateFor.values())
      {
         totalGroundReactionWrench.sub(externalWrenchToCompensateFor);
      }

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


      wrenchDistributorInput.reset();

      for (PlaneContactState contactState : planeContactStates.values())
      {
         List<FramePoint> footContactPoints = contactState.getContactPoints();

         if (footContactPoints.size() > 0)
         {
            wrenchDistributorInput.addPlaneContact(contactState);
         }
      }

      if (null != cylinderContactStates)
      {
         for (RigidBody body : cylinderContactStates.keySet())
         {
            CylindricalContactState cylinderContact = cylinderContactStates.get(body);
            wrenchDistributorInput.addCylinderContact(cylinderContact);
         }
      }

      wrenchDistributorInput.setSpatialForceVectorAndUpcomingSupportSide(totalGroundReactionWrench, upcomingSupportLeg);

      GroundReactionWrenchDistributorOutputData distributedWrenches = new GroundReactionWrenchDistributorOutputData();
      groundReactionWrenchDistributor.solve(distributedWrenches, wrenchDistributorInput);

      for (ContactablePlaneBody contactablePlaneBody : planeContactStates.keySet())
      {
         RigidBody rigidBody = contactablePlaneBody.getRigidBody();

         PlaneContactState contactState = planeContactStates.get(contactablePlaneBody);
         List<FramePoint> footContactPoints = contactState.getContactPoints();

         if (footContactPoints.size() > 0)
         {
            FrameVector force = distributedWrenches.getForce(contactState);
            FramePoint2d cop = distributedWrenches.getCenterOfPressure(contactState);
            double normalTorque = distributedWrenches.getNormalTorque(contactState);

            Wrench groundReactionWrench = new Wrench(rigidBody.getBodyFixedFrame(), contactState.getPlaneFrame());
            externalWrenches.put(rigidBody, groundReactionWrench);
            WrenchDistributorTools.computeWrench(groundReactionWrench, force, cop, normalTorque);
            groundReactionWrench.changeFrame(rigidBody.getBodyFixedFrame());
         }
      }

      
      if (cylinderContactStates != null)
      {
         for (RigidBody body : cylinderContactStates.keySet())
         {
            if (cylinderContactStates.get(body).isInContact())
            {
               Wrench bodyWrench = distributedWrenches.getWrenchOfNonPlaneContact(cylinderContactStates.get(body));
               if (bodyWrench == null)
               {
                  System.err.println("Wrench from cylinder not found! Crashing!");
                  throw new RuntimeException("Wrench not found. OptimizationBasedForceDistributor failed to provide it");
               }
               else
               {
                  //GrayTODO hack!
                  if (cylinderContactStates.get(body).isInContact())
                  {
//                     bodyWrench.changeFrame(body.getBodyFixedFrame());
                     bodyWrench.changeFrame(this.centerOfMassFrame);
                     bodyWrench.setAngularPartX(0);
                     bodyWrench.setAngularPartY(0);
                     bodyWrench.setAngularPartZ(-5);
                     bodyWrench.setLinearPartX(-10);
                     bodyWrench.setLinearPartY(0);
                     bodyWrench.setLinearPartZ(0);
                     bodyWrench.changeFrame(body.getBodyFixedFrame());
                     externalWrenches.put(body, bodyWrench);
                  }
                  else
                  {
                     bodyWrench.changeFrame(body.getBodyFixedFrame());
                     externalWrenches.put(body, bodyWrench);
                  }
               }
            }
         }
      }

      Wrench admissibleGroundReactionWrench = TotalWrenchCalculator.computeTotalWrench(externalWrenches.values(),
                                                 totalGroundReactionWrench.getExpressedInFrame());
      desiredCentroidalMomentumRate.set(admissibleGroundReactionWrench);
      desiredCentroidalMomentumRate.add(gravitationalWrench);

      for (RigidBody rigidBody : externalWrenchesToCompensateFor.keySet())
      {
         Wrench externalWrenchToCompensateFor = externalWrenchesToCompensateFor.get(rigidBody);
         totalGroundReactionWrench.add(externalWrenchToCompensateFor);
         Wrench wrench = externalWrenches.get(rigidBody);
         ReferenceFrame bodyFixedFrame = rigidBody.getBodyFixedFrame();
         externalWrenchToCompensateFor.changeBodyFrameAttachedToSameBody(bodyFixedFrame);
         externalWrenchToCompensateFor.changeFrame(bodyFixedFrame);
         if (wrench == null)
            externalWrenches.put(rigidBody, externalWrenchToCompensateFor);
         else
            wrench.add(externalWrenchToCompensateFor);
      }

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
      InverseDynamicsJoint[] jointsInOrder = jacobian.getJointsInOrder();
      if (Arrays.asList(jointsInOrder).contains(rootJoint))
      {
         DenseMatrix64F selectionMatrix = taskspaceConstraintData.getSelectionMatrix();
         SpatialAccelerationVector spatialAcceleration = taskspaceConstraintData.getSpatialAcceleration();

         rootJointAccelerationData.setUsingSelectionMatrix(selectionMatrix, spatialAcceleration);
      }
      else
      {
         solver.setDesiredSpatialAcceleration(jacobian, taskspaceConstraintData);
      }
   }

   public void setExternalWrenchToCompensateFor(RigidBody rigidBody, Wrench wrench)
   {
      Wrench copy = new Wrench(wrench);
      copy.changeFrame(centerOfMassFrame);
      externalWrenchesToCompensateFor.put(rigidBody, copy);
   }

   public void setDesiredRateOfChangeOfMomentum(MomentumRateOfChangeData momentumRateOfChangeData)
   {
      this.momentumRateOfChangeData.set(momentumRateOfChangeData);
   }

   public SpatialForceVector getDesiredCentroidalMomentumRate()
   {
      return desiredCentroidalMomentumRate;
   }

   public Map<RigidBody, Wrench> getExternalWrenches()
   {
      return externalWrenches;
   }
}
