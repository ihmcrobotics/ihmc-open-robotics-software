package us.ihmc.quadrupedRobotics.controller;

import static us.ihmc.humanoidRobotics.footstep.FootstepUtils.worldFrame;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.QuadrupedSupportPolygons;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.commonWalkingControlModules.referenceFrames.CommonQuadrupedReferenceFramesVisualizer;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.mecano.algorithms.CenterOfMassJacobian;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.quadrupedRobotics.controlModules.foot.QuadrupedFootControlModuleParameters;
import us.ihmc.quadrupedRobotics.controller.toolbox.DivergentComponentOfMotionEstimator;
import us.ihmc.quadrupedRobotics.controller.toolbox.LinearInvertedPendulumModel;
import us.ihmc.quadrupedRobotics.controller.toolbox.QuadrupedFallDetector;
import us.ihmc.robotics.controllers.ControllerFailureListener;
import us.ihmc.robotics.geometry.GroundPlaneEstimator;
import us.ihmc.robotics.geometry.YoGroundPlaneEstimator;
import us.ihmc.quadrupedBasics.referenceFrames.QuadrupedReferenceFrames;
import us.ihmc.quadrupedRobotics.model.QuadrupedPhysicalProperties;
import us.ihmc.quadrupedRobotics.model.QuadrupedRuntimeEnvironment;
import us.ihmc.quadrupedRobotics.planning.ContactState;
import us.ihmc.robotModels.FullQuadrupedRobotModel;
import us.ihmc.robotics.contactable.ContactablePlaneBody;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.sensors.CenterOfMassDataHolderReadOnly;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoFramePoint3D;
import us.ihmc.yoVariables.variable.YoFrameVector3D;

public class QuadrupedControllerToolbox
{
   private final QuadrupedReferenceFrames referenceFrames;
   private final CommonQuadrupedReferenceFramesVisualizer referenceFramesVisualizer;
   private final LinearInvertedPendulumModel linearInvertedPendulumModel;
   private final DivergentComponentOfMotionEstimator dcmPositionEstimator;

   private final GroundPlaneEstimator groundPlaneEstimator;
   private final GroundPlaneEstimator upcomingGroundPlaneEstimator;
   private final QuadrantDependentList<YoFramePoint3D> groundPlanePositions;
   private final QuadrantDependentList<YoFramePoint3D> upcomingGroundPlanePositions;

   private final QuadrupedFallDetector fallDetector;

   private final CenterOfMassJacobian comJacobian;

   private final QuadrupedRuntimeEnvironment runtimeEnvironment;
   private final QuadrupedFootControlModuleParameters footControlModuleParameters;

   private final FullQuadrupedRobotModel fullRobotModel;

   private final QuadrantDependentList<YoEnum<ContactState>> contactStates = new QuadrantDependentList<>();
   private final QuadrantDependentList<YoPlaneContactState> footContactStates = new QuadrantDependentList<>();
   private final List<ContactablePlaneBody> contactablePlaneBodies;

   private final QuadrupedSupportPolygons supportPolygon;

   private final FrameVector3D comVelocityEstimate = new FrameVector3D();

   private final YoFrameVector3D yoCoMVelocityEstimate;
   private final CenterOfMassDataHolderReadOnly centerOfMassDataHolder;

   private final ArrayList<ControllerFailureListener> controllerFailureListeners = new ArrayList<>();

   private final YoBoolean controllerFailed;

   public QuadrupedControllerToolbox(QuadrupedRuntimeEnvironment runtimeEnvironment, QuadrupedPhysicalProperties physicalProperties,
                                     YoVariableRegistry registry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      double gravity = 9.81;
      double mass = runtimeEnvironment.getFullRobotModel().getTotalMass();

      yoCoMVelocityEstimate = new YoFrameVector3D("yoCoMVelocityEstimate", ReferenceFrame.getWorldFrame(), registry);
      controllerFailed = new YoBoolean("controllerFailed", registry);

      this.runtimeEnvironment = runtimeEnvironment;

      fullRobotModel = runtimeEnvironment.getFullRobotModel();
      footControlModuleParameters = new QuadrupedFootControlModuleParameters();
      registry.addChild(footControlModuleParameters.getYoVariableRegistry());



      // create controllers and estimators
      referenceFrames = new QuadrupedReferenceFrames(runtimeEnvironment.getFullRobotModel());

      linearInvertedPendulumModel = new LinearInvertedPendulumModel(referenceFrames.getCenterOfMassFrame(), mass, gravity, physicalProperties.getNominalBodyHeight(), registry);
//      upcomingGroundPlaneEstimator = new YoGroundPlaneEstimator("upcoming", registry, runtimeEnvironment.getGraphicsListRegistry(), YoAppearance.PlaneMaterial());
      upcomingGroundPlaneEstimator = new GroundPlaneEstimator();
      groundPlaneEstimator = new YoGroundPlaneEstimator(registry, runtimeEnvironment.getGraphicsListRegistry());
      groundPlanePositions = new QuadrantDependentList<>();
      upcomingGroundPlanePositions = new QuadrantDependentList<>();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         groundPlanePositions.set(robotQuadrant, new YoFramePoint3D(robotQuadrant.getCamelCaseName() + "GroundPlanePosition", worldFrame, registry));
         upcomingGroundPlanePositions.set(robotQuadrant, new YoFramePoint3D(robotQuadrant.getCamelCaseName() + "UpcomingGroundPlanePosition", worldFrame, registry));
      }

      comJacobian = new CenterOfMassJacobian(fullRobotModel.getElevator(), worldFrame);
      dcmPositionEstimator = new DivergentComponentOfMotionEstimator(referenceFrames.getCenterOfMassFrame(), linearInvertedPendulumModel, registry, yoGraphicsListRegistry);

      fallDetector = new QuadrupedFallDetector(referenceFrames.getBodyFrame(), referenceFrames.getSoleFrames(), dcmPositionEstimator,
                                               runtimeEnvironment.getFallDetectionParameters(), registry);

      contactablePlaneBodies = runtimeEnvironment.getContactablePlaneBodies();
      centerOfMassDataHolder = runtimeEnvironment.getCenterOfMassDataHolder();

      double coefficientOfFriction = 1.0; // TODO: magic number...
      QuadrantDependentList<ContactablePlaneBody> contactableFeet = runtimeEnvironment.getContactableFeet();

      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         ContactablePlaneBody contactableFoot = contactableFeet.get(robotQuadrant);
         RigidBodyBasics rigidBody = contactableFoot.getRigidBody();
         String name = contactableFoot.getSoleFrame().getName();
         YoPlaneContactState planeContactState = new YoPlaneContactState(name, rigidBody, contactableFoot.getSoleFrame(),
                                                                         contactableFoot.getContactPoints2d(), coefficientOfFriction, registry);
         YoEnum<ContactState> contactState = new YoEnum<>(name, registry, ContactState.class);


         footContactStates.put(robotQuadrant, planeContactState);
         contactStates.put(robotQuadrant, contactState);
         planeContactState.attachContactChangeListener(changed ->
                                                       {
                                                          if (((YoBoolean) changed).getBooleanValue())
                                                             contactState.set(ContactState.IN_CONTACT);
                                                          else
                                                             contactState.set(ContactState.NO_CONTACT);
                                                       });

      }

      if (yoGraphicsListRegistry != null)
         referenceFramesVisualizer = new CommonQuadrupedReferenceFramesVisualizer(referenceFrames, yoGraphicsListRegistry, registry);
      else
         referenceFramesVisualizer = null;


      supportPolygon = new QuadrupedSupportPolygons(referenceFrames.getCenterOfFeetZUpFrameAveragingLowestZHeightsAcrossEnds(), footContactStates,
                                                    referenceFrames.getSoleZUpFrames(), registry, yoGraphicsListRegistry);

      attachControllerFailureListener(fallingDirection -> controllerFailed.set(true));

      update();
      updateSupportPolygon();
   }

   public void attachControllerFailureListener(ControllerFailureListener controllerFailureListener)
   {
      controllerFailureListeners.add(controllerFailureListener);
      fallDetector.attachControllerFailureListener(controllerFailureListener);
   }

   public void update()
   {
      referenceFrames.updateFrames();

      if (referenceFramesVisualizer != null)
         referenceFramesVisualizer.update();

      if(centerOfMassDataHolder == null)
      {
         comJacobian.reset();
         comVelocityEstimate.setIncludingFrame(comJacobian.getCenterOfMassVelocity());
      }
      else
      {
         centerOfMassDataHolder.getCenterOfMassVelocity(comVelocityEstimate);
      }

      yoCoMVelocityEstimate.setMatchingFrame(comVelocityEstimate);
      dcmPositionEstimator.compute(comVelocityEstimate);
   }

   public void updateSupportPolygon()
   {
      supportPolygon.updateUsingContactStates(footContactStates);
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

   public GroundPlaneEstimator getUpcomingGroundPlaneEstimator()
   {
      return upcomingGroundPlaneEstimator;
   }

   public QuadrantDependentList<YoFramePoint3D> getGroundPlanePositions()
   {
      return groundPlanePositions;
   }

   public QuadrantDependentList<YoFramePoint3D> getUpcomingGroundPlanePositions()
   {
      return upcomingGroundPlanePositions;
   }

   public QuadrupedFallDetector getFallDetector()
   {
      return fallDetector;
   }

   public YoBoolean getControllerFailedBoolean()
   {
      return controllerFailed;
   }

   public MovingReferenceFrame getSoleReferenceFrame(RobotQuadrant robotQuadrant)
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
      return contactStates.get(robotQuadrant).getEnumValue();
   }

   public QuadrantDependentList<YoEnum<ContactState>> getContactStates()
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

   public QuadrupedSupportPolygons getSupportPolygons()
   {
      return supportPolygon;
   }
}
