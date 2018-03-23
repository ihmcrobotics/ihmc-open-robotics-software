package us.ihmc.quadrupedRobotics.controller.force;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.quadrupedRobotics.controlModules.foot.QuadrupedFootControlModuleParameters;
import us.ihmc.quadrupedRobotics.controller.force.toolbox.*;
import us.ihmc.quadrupedRobotics.estimator.GroundPlaneEstimator;
import us.ihmc.quadrupedRobotics.estimator.referenceFrames.QuadrupedReferenceFrames;
import us.ihmc.quadrupedRobotics.model.QuadrupedPhysicalProperties;
import us.ihmc.quadrupedRobotics.model.QuadrupedRuntimeEnvironment;
import us.ihmc.robotModels.FullQuadrupedRobotModel;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

import java.util.List;

public class QuadrupedForceControllerToolbox
{
   private final QuadrupedReferenceFrames referenceFrames;
   private final QuadrupedTaskSpaceEstimator taskSpaceEstimator;
   private final QuadrupedTaskSpaceController taskSpaceController;
   private final LinearInvertedPendulumModel linearInvertedPendulumModel;
   private final DivergentComponentOfMotionEstimator dcmPositionEstimator;
   private final GroundPlaneEstimator groundPlaneEstimator;
   private final QuadrupedFallDetector fallDetector;

   private final QuadrupedRuntimeEnvironment runtimeEnvironment;
   private final QuadrupedFootControlModuleParameters footControlModuleParameters;

   private final QuadrupedTaskSpaceEstimates taskSpaceEstimates = new QuadrupedTaskSpaceEstimates();
   private final FullQuadrupedRobotModel fullRobotModel;

   private final QuadrantDependentList<YoPlaneContactState> footContactStates = new QuadrantDependentList<>();
   private final List<ContactablePlaneBody> contactablePlaneBodies;

   public QuadrupedForceControllerToolbox(QuadrupedRuntimeEnvironment runtimeEnvironment, QuadrupedPhysicalProperties physicalProperties,
                                          YoVariableRegistry registry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      double gravity = 9.81;
      double mass = runtimeEnvironment.getFullRobotModel().getTotalMass();

      this.runtimeEnvironment = runtimeEnvironment;

      fullRobotModel = runtimeEnvironment.getFullRobotModel();
      footControlModuleParameters = new QuadrupedFootControlModuleParameters();
      registry.addChild(footControlModuleParameters.getYoVariableRegistry());

      // create controllers and estimators
      referenceFrames = new QuadrupedReferenceFrames(runtimeEnvironment.getFullRobotModel(), physicalProperties);
      taskSpaceEstimator = new QuadrupedTaskSpaceEstimator(runtimeEnvironment.getFullRobotModel(), referenceFrames, registry, runtimeEnvironment.getGraphicsListRegistry());
      taskSpaceController = new QuadrupedTaskSpaceController(runtimeEnvironment, referenceFrames, registry, runtimeEnvironment.getGraphicsListRegistry());
      linearInvertedPendulumModel = new LinearInvertedPendulumModel(referenceFrames.getCenterOfMassFrame(), mass, gravity, 1.0, registry);
      dcmPositionEstimator = new DivergentComponentOfMotionEstimator(referenceFrames.getCenterOfMassFrame(), linearInvertedPendulumModel, registry, yoGraphicsListRegistry);
      groundPlaneEstimator = new GroundPlaneEstimator(registry, runtimeEnvironment.getGraphicsListRegistry());
      fallDetector = new QuadrupedFallDetector(taskSpaceEstimator, dcmPositionEstimator, registry);

      contactablePlaneBodies = runtimeEnvironment.getContactablePlaneBodies();

      double coefficientOfFriction = 1.0; // TODO: magic number...
      QuadrantDependentList<ContactablePlaneBody> contactableFeet = runtimeEnvironment.getContactableFeet();

      for (RobotQuadrant robotSide : RobotQuadrant.values)
      {
         ContactablePlaneBody contactableFoot = contactableFeet.get(robotSide);
         RigidBody rigidBody = contactableFoot.getRigidBody();
         YoPlaneContactState contactState = new YoPlaneContactState(contactableFoot.getSoleFrame().getName(), rigidBody, contactableFoot.getSoleFrame(),
                                                                    contactableFoot.getContactPoints2d(), coefficientOfFriction, registry);

         footContactStates.put(robotSide, contactState);
      }
   }

   public void update()
   {
      taskSpaceEstimator.compute(taskSpaceEstimates);

      dcmPositionEstimator.compute(taskSpaceEstimates.getComVelocity());
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

   public QuadrupedTaskSpaceController getTaskSpaceController()
   {
      return taskSpaceController;
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

   public QuadrupedTaskSpaceEstimates getTaskSpaceEstimates()
   {
      return taskSpaceEstimates;
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

   public List<ContactablePlaneBody> getContactablePlaneBodies()
   {
      return contactablePlaneBodies;
   }
}
