package us.ihmc.commonWalkingControlModules.momentumBasedController;

import java.util.Arrays;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

import javax.vecmath.Vector3d;

import org.ejml.data.DenseMatrix64F;
import org.ejml.interfaces.linsol.LinearSolver;
import org.ejml.ops.CommonOps;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.PlaneContactState;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.DesiredJointAccelerationCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.DesiredPointAccelerationCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.DesiredRateOfChangeOfMomentumCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.DesiredSpatialAccelerationCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.MomentumModuleSolution;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.MomentumRateOfChangeData;
import us.ihmc.commonWalkingControlModules.wrenchDistribution.GroundReactionWrenchDistributor;
import us.ihmc.commonWalkingControlModules.wrenchDistribution.GroundReactionWrenchDistributorInputData;
import us.ihmc.commonWalkingControlModules.wrenchDistribution.GroundReactionWrenchDistributorOutputData;
import us.ihmc.commonWalkingControlModules.wrenchDistribution.WrenchDistributorTools;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.GeometricJacobian;
import us.ihmc.robotics.screwTheory.InverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.robotics.screwTheory.SixDoFJoint;
import us.ihmc.robotics.screwTheory.SpatialAccelerationVector;
import us.ihmc.robotics.screwTheory.SpatialForceVector;
import us.ihmc.robotics.screwTheory.TotalMassCalculator;
import us.ihmc.robotics.screwTheory.TotalWrenchCalculator;
import us.ihmc.robotics.screwTheory.TwistCalculator;
import us.ihmc.robotics.screwTheory.Wrench;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.graphics.YoGraphicsListRegistry;
import us.ihmc.robotics.humanoidRobot.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.yoUtilities.math.filters.AlphaFilteredYoFrameVector;
import us.ihmc.yoUtilities.math.filters.AlphaFilteredYoVariable;
import us.ihmc.yoUtilities.math.frames.YoFrameVector;


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
   private GroundReactionWrenchDistributorOutputData distributedWrenches = new GroundReactionWrenchDistributorOutputData();

   private GroundReactionWrenchDistributorInputData wrenchDistributorInput = new GroundReactionWrenchDistributorInputData();

   //   private final SpatialForceVector netGroundReactionWrench;
   //   private final SpatialForceVector desiredNetWrench;

   public OldMomentumControlModule(SixDoFJoint rootJoint, double gravityZ, GroundReactionWrenchDistributor groundReactionWrenchDistributor,
         ReferenceFrame centerOfMassFrame, double controlDT, TwistCalculator twistCalculator, LinearSolver<DenseMatrix64F> jacobianSolver,
         YoVariableRegistry parentRegistry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      MathTools.checkIfInRange(gravityZ, 0.0, Double.POSITIVE_INFINITY);
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
      rootJointAccelerationData.setEmpty();
      momentumRateOfChangeData.setEmpty();
      solver.reset();
      externalWrenches.clear();
      externalWrenchesToCompensateFor.clear();
   }

   private final TotalWrenchCalculator totalWrenchCalculator = new TotalWrenchCalculator();
   private final Wrench admissibleGroundReactionWrench = new Wrench();

   public MomentumModuleSolution compute(Map<ContactablePlaneBody, ? extends PlaneContactState> planeContactStates, RobotSide upcomingSupportLeg)
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
         List<FramePoint> footContactPoints = contactState.getContactFramePointsInContactCopy();

         if (footContactPoints.size() > 0)
         {
            wrenchDistributorInput.addPlaneContact(contactState);
         }
      }

      wrenchDistributorInput.setSpatialForceVectorAndUpcomingSupportSide(totalGroundReactionWrench, upcomingSupportLeg);

      groundReactionWrenchDistributor.solve(distributedWrenches, wrenchDistributorInput);

      for (ContactablePlaneBody contactablePlaneBody : planeContactStates.keySet())
      {
         RigidBody rigidBody = contactablePlaneBody.getRigidBody();

         PlaneContactState contactState = planeContactStates.get(contactablePlaneBody);
         List<FramePoint> footContactPoints = contactState.getContactFramePointsInContactCopy();

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

      totalWrenchCalculator.computeTotalWrench(admissibleGroundReactionWrench, externalWrenches.values(), totalGroundReactionWrench.getExpressedInFrame());
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
      //TODO: Add the joint accelerations to the solution, or not?
      MomentumModuleSolution solution = new MomentumModuleSolution(null, null, desiredCentroidalMomentumRate, externalWrenches);
      
      return solution;
   }

   public void resetGroundReactionWrenchFilter()
   {
      groundReactionWrenchFilterResetRequest.set(true);
   }

   public void setDesiredJointAcceleration(DesiredJointAccelerationCommand desiredJointAccelerationCommand)
   {
      solver.setDesiredJointAcceleration(desiredJointAccelerationCommand.getJoint(), desiredJointAccelerationCommand.getDesiredAcceleration());
   }

//   public void setDesiredJointAcceleration(InverseDynamicsJoint joint, DenseMatrix64F jointAcceleration, double weight)
//   {
//      throw new NoSuchMethodError();
//   }
   
   
   public void setDesiredSpatialAcceleration(DesiredSpatialAccelerationCommand desiredSpatialAccelerationCommand)
   {
      setDesiredSpatialAcceleration(desiredSpatialAccelerationCommand.getJacobian(), desiredSpatialAccelerationCommand.getTaskspaceConstraintData());
   }

   private void setDesiredSpatialAcceleration(GeometricJacobian jacobian, TaskspaceConstraintData taskspaceConstraintData)
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
   
//   public void setDesiredSpatialAcceleration(GeometricJacobian jacobian, TaskspaceConstraintData taskspaceConstraintData, double weight)
//   {
//      throw new NoSuchMethodError();
//   }

   public void setExternalWrenchToCompensateFor(RigidBody rigidBody, Wrench wrench)
   {
      Wrench copy = new Wrench(wrench);
      copy.changeFrame(centerOfMassFrame);
      externalWrenchesToCompensateFor.put(rigidBody, copy);
   }

   public void setDesiredRateOfChangeOfMomentum(DesiredRateOfChangeOfMomentumCommand desiredRateOfChangeOfMomentumCommand)
   {
      this.momentumRateOfChangeData.set(desiredRateOfChangeOfMomentumCommand.getMomentumRateOfChangeData());
   }


   public void setDesiredPointAcceleration(DesiredPointAccelerationCommand desiredPointAccelerationCommand)
   {
      throw new NoSuchMethodError();
   }


   private static class RootJointAccelerationData
   {
      private final ReferenceFrame bodyFrame;
      private final ReferenceFrame baseFrame;
      private final ReferenceFrame expressedInFrame;
      private final DenseMatrix64F accelerationSubspace = new DenseMatrix64F(SpatialAccelerationVector.SIZE, SpatialAccelerationVector.SIZE);
      private final DenseMatrix64F accelerationMultipliers = new DenseMatrix64F(SpatialAccelerationVector.SIZE, 1);

      public RootJointAccelerationData(ReferenceFrame bodyFrame, ReferenceFrame baseFrame, ReferenceFrame expressedInFrame)
      {
         this.bodyFrame = bodyFrame;
         this.baseFrame = baseFrame;
         this.expressedInFrame = expressedInFrame;
      }

      public ReferenceFrame getBodyFrame()
      {
         return bodyFrame;
      }

      public ReferenceFrame getBaseFrame()
      {
         return baseFrame;
      }

      public ReferenceFrame getExpressedInFrame()
      {
         return expressedInFrame;
      }

      public DenseMatrix64F getAccelerationSubspace()
      {
         return accelerationSubspace;
      }

      public DenseMatrix64F getAccelerationMultipliers()
      {
         return accelerationMultipliers;
      }

      public void setEmpty()
      {
         accelerationSubspace.reshape(SpatialForceVector.SIZE, 0);
         accelerationMultipliers.reshape(0, 1);
      }

      public void setUsingSelectionMatrix(DenseMatrix64F selectionMatrix, SpatialAccelerationVector spatialAcceleration)
      {
         // NOTE: doesn't work in all cases of selectionMatrix (only works when pseudo inverse == transpose)

         accelerationSubspace.reshape(selectionMatrix.getNumCols(), selectionMatrix.getNumRows());
         CommonOps.transpose(selectionMatrix, accelerationSubspace);

         accelerationMultipliers.reshape(selectionMatrix.getNumRows(), 1);
         DenseMatrix64F spatialAccelerationMatrix = new DenseMatrix64F(SpatialAccelerationVector.SIZE, 1);
         spatialAcceleration.packMatrix(spatialAccelerationMatrix, 0);
         CommonOps.mult(selectionMatrix, spatialAccelerationMatrix, accelerationMultipliers);
      }
   }

   public void setFootCoPControlData(RobotSide side, ReferenceFrame frame)
   {
      // TODO Auto-generated method stub 
   }

}
