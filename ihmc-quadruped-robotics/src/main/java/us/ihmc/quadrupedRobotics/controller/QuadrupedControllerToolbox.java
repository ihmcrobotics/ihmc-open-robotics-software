package us.ihmc.quadrupedRobotics.controller;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactPolygon;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.quadrupedRobotics.controlModules.foot.QuadrupedFootControlModuleParameters;
import us.ihmc.quadrupedRobotics.controller.toolbox.*;
import us.ihmc.quadrupedRobotics.estimator.GroundPlaneEstimator;
import us.ihmc.quadrupedRobotics.estimator.referenceFrames.QuadrupedReferenceFrames;
import us.ihmc.quadrupedRobotics.model.QuadrupedPhysicalProperties;
import us.ihmc.quadrupedRobotics.model.QuadrupedRuntimeEnvironment;
import us.ihmc.quadrupedRobotics.planning.ContactState;
import us.ihmc.robotModels.FullQuadrupedRobotModel;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.screwTheory.CenterOfMassJacobian;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoFrameConvexPolygon2D;
import us.ihmc.yoVariables.variable.YoFrameVector3D;

import java.awt.*;
import java.util.List;

public class QuadrupedControllerToolbox
{
   private final QuadrupedReferenceFrames referenceFrames;
   private final LinearInvertedPendulumModel linearInvertedPendulumModel;
   private final DivergentComponentOfMotionEstimator dcmPositionEstimator;
   private final GroundPlaneEstimator groundPlaneEstimator;
   private final QuadrupedSoleForceEstimator soleForceEstimator;
   private final QuadrupedFallDetector fallDetector;

   private final CenterOfMassJacobian comJacobian;

   private final QuadrupedRuntimeEnvironment runtimeEnvironment;
   private final QuadrupedFootControlModuleParameters footControlModuleParameters;

   private final FullQuadrupedRobotModel fullRobotModel;

   private final QuadrantDependentList<ContactState> contactStates = new QuadrantDependentList<>();
   private final QuadrantDependentList<YoPlaneContactState> footContactStates = new QuadrantDependentList<>();
   private final List<ContactablePlaneBody> contactablePlaneBodies;

   private final YoFrameConvexPolygon2D supportPolygon;
   private final YoArtifactPolygon supportPolygonVisualizer;

   private final FramePoint3D tempPoint = new FramePoint3D();
   private final FrameVector3D comVelocityEstimate = new FrameVector3D();

   private final YoFrameVector3D yoCoMVelocityEstimate;

   public QuadrupedControllerToolbox(QuadrupedRuntimeEnvironment runtimeEnvironment, QuadrupedPhysicalProperties physicalProperties,
                                     YoVariableRegistry registry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      double gravity = 9.81;
      double mass = runtimeEnvironment.getFullRobotModel().getTotalMass();

      yoCoMVelocityEstimate = new YoFrameVector3D("yoCoMVelocityEstimate", ReferenceFrame.getWorldFrame(), registry);

      this.runtimeEnvironment = runtimeEnvironment;

      fullRobotModel = runtimeEnvironment.getFullRobotModel();
      footControlModuleParameters = new QuadrupedFootControlModuleParameters();
      registry.addChild(footControlModuleParameters.getYoVariableRegistry());

      supportPolygon = new YoFrameConvexPolygon2D("supportPolygon", ReferenceFrame.getWorldFrame(), 4, registry);
      supportPolygonVisualizer = new YoArtifactPolygon("supportPolygonVisualizer", supportPolygon, Color.black, false, 1);
      yoGraphicsListRegistry.registerArtifact("supportPolygon", supportPolygonVisualizer);


      // create controllers and estimators
      referenceFrames = new QuadrupedReferenceFrames(runtimeEnvironment.getFullRobotModel(), physicalProperties);

      soleForceEstimator = new QuadrupedSoleForceEstimator(fullRobotModel, referenceFrames, registry);

      linearInvertedPendulumModel = new LinearInvertedPendulumModel(referenceFrames.getCenterOfMassFrame(), mass, gravity, 1.0, registry);
      groundPlaneEstimator = new GroundPlaneEstimator(registry, runtimeEnvironment.getGraphicsListRegistry());

      comJacobian = new CenterOfMassJacobian(fullRobotModel.getElevator());
      dcmPositionEstimator = new DivergentComponentOfMotionEstimator(referenceFrames.getCenterOfMassFrame(), linearInvertedPendulumModel, registry, yoGraphicsListRegistry);

      fallDetector = new QuadrupedFallDetector(referenceFrames.getBodyFrame(), referenceFrames.getSoleFrames(), dcmPositionEstimator, registry);

      contactablePlaneBodies = runtimeEnvironment.getContactablePlaneBodies();

      double coefficientOfFriction = 1.0; // TODO: magic number...
      QuadrantDependentList<ContactablePlaneBody> contactableFeet = runtimeEnvironment.getContactableFeet();

      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         ContactablePlaneBody contactableFoot = contactableFeet.get(robotQuadrant);
         RigidBody rigidBody = contactableFoot.getRigidBody();
         YoPlaneContactState contactState = new YoPlaneContactState(contactableFoot.getSoleFrame().getName(), rigidBody, contactableFoot.getSoleFrame(),
                                                                    contactableFoot.getContactPoints2d(), coefficientOfFriction, registry);

         footContactStates.put(robotQuadrant, contactState);
         contactStates.put(robotQuadrant, ContactState.IN_CONTACT);
      }

      update(); 
   }

   public void update()
   {
      referenceFrames.updateFrames();
      soleForceEstimator.compute();
      comJacobian.compute();

      updateSupportPolygon();

      comJacobian.getCenterOfMassVelocity(comVelocityEstimate);

      yoCoMVelocityEstimate.setMatchingFrame(comVelocityEstimate);

      dcmPositionEstimator.compute(comVelocityEstimate);
   }

   public void updateSupportPolygon()
   {
      supportPolygon.clear();

      for(RobotQuadrant quadrant : RobotQuadrant.values)
      {
         if(contactStates.get(quadrant) == ContactState.IN_CONTACT)
         {
            tempPoint.setToZero(referenceFrames.getSoleFrame(quadrant));
            supportPolygon.addVertexMatchingFrame(tempPoint);
         }
      }

      supportPolygon.update();
   }

   public FullQuadrupedRobotModel getFullRobotModel()
   {
      return fullRobotModel;
   }

   public QuadrupedRuntimeEnvironment getRuntimeEnvironment()
   {
      return runtimeEnvironment;
   }

   public QuadrupedFootControlModuleParameters getFootControlModuleParameters()
   {
      return footControlModuleParameters;
   }

   public QuadrupedReferenceFrames getReferenceFrames()
   {
      return referenceFrames;
   }

   public LinearInvertedPendulumModel getLinearInvertedPendulumModel()
   {
      return linearInvertedPendulumModel;
   }

   public GroundPlaneEstimator getGroundPlaneEstimator()
   {
      return groundPlaneEstimator;
   }

   public QuadrupedFallDetector getFallDetector()
   {
      return fallDetector;
   }

   public ReferenceFrame getSoleReferenceFrame(RobotQuadrant robotQuadrant)
   {
      return referenceFrames.getSoleFrame(robotQuadrant);
   }

   public void getDCMPositionEstimate(FramePoint3D dcmPositionEstimateToPack)
   {
      dcmPositionEstimator.getDCMPositionEstimate(dcmPositionEstimateToPack);
   }

   public YoPlaneContactState getFootContactState(RobotQuadrant robotQuadrant)
   {
      return footContactStates.get(robotQuadrant);
   }

   public QuadrantDependentList<YoPlaneContactState> getFootContactStates()
   {
      return footContactStates;
   }

   public ContactState getContactState(RobotQuadrant robotQuadrant)
   {
      return contactStates.get(robotQuadrant);
   }

   public QuadrantDependentList<ContactState> getContactStates()
   {
      return contactStates;
   }

   public List<ContactablePlaneBody> getContactablePlaneBodies()
   {
      return contactablePlaneBodies;
   }

   public FrameVector3DReadOnly getCoMVelocityEstimate()
   {
      return yoCoMVelocityEstimate;
   }

   public FrameVector3D getSoleContactForce(RobotQuadrant robotQuadrant)
   {
      return soleForceEstimator.getSoleContactForce(robotQuadrant);
   }
}
