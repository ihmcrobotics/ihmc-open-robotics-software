package us.ihmc.quadrupedRobotics.stateEstimator.kinematicsBased;

import java.util.ArrayList;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.quadrupedRobotics.sensorProcessing.sensorProcessors.FootSwitchUpdater;
import us.ihmc.quadrupedRobotics.stateEstimator.QuadrupedStateEstimator;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.time.TimeTools;
import us.ihmc.sensorProcessing.sensorProcessors.SensorOutputMapReadOnly;
import us.ihmc.sensorProcessing.stateEstimation.evaluation.FullInverseDynamicsStructure;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicReferenceFrame;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicsListRegistry;
import us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation.JointStateUpdater;

public class QuadrupedKinematicsBasedStateEstimator implements QuadrupedStateEstimator
{
   private final String name = getClass().getSimpleName();
   private final YoVariableRegistry registry = new YoVariableRegistry(name);
   private final YoGraphicsListRegistry yoGraphicsListRegistry;

   private final SDFFullRobotModel sdfFullRobotModelForViz;

   private final JointStateUpdater jointStateUpdater;
   private final FootSwitchUpdater footSwitchUpdater;
   private final CenterOfMassLinearStateUpdater comLinearStateUpdater;
   private final CenterOfMassImuBasedRotationalStateUpdater comRotationalStateUpdater;

   private final SensorOutputMapReadOnly sensorOutputMapReadOnly;

   private final ArrayList<YoGraphicReferenceFrame> graphicReferenceFrames = new ArrayList<>();

   private final ArrayList<RobotQuadrant> feetInContact = new ArrayList<>();
   private final ArrayList<RobotQuadrant> feetNotInContact = new ArrayList<>();

   private final QuadrantDependentList<BooleanYoVariable> footContactBooleans = new QuadrantDependentList<>();

   private final BooleanYoVariable isEnabled = new BooleanYoVariable(name + "IsEnabled", registry);

   private boolean hasBeenInitialized = false;

   public QuadrupedKinematicsBasedStateEstimator(FullInverseDynamicsStructure inverseDynamicsStructure, SensorOutputMapReadOnly sensorOutputMapReadOnly,
         FootSwitchUpdater footSwitchUpdater, JointStateUpdater jointStateUpdater, CenterOfMassLinearStateUpdater comLinearStateUpdater,
         CenterOfMassImuBasedRotationalStateUpdater comRotationalStateUpdater, SDFFullRobotModel sdfFullRobotModelForViz, YoVariableRegistry parentRegistry,
         YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this.yoGraphicsListRegistry = yoGraphicsListRegistry;

      this.jointStateUpdater = jointStateUpdater;

      this.footSwitchUpdater = footSwitchUpdater;
      this.sensorOutputMapReadOnly = sensorOutputMapReadOnly;

      this.comLinearStateUpdater = comLinearStateUpdater;
      this.comRotationalStateUpdater = comRotationalStateUpdater;
      
      this.sdfFullRobotModelForViz = sdfFullRobotModelForViz;

      if (this.sdfFullRobotModelForViz != null)
         initializeVisualization();

      isEnabled.set(false); //TODO initialize to false

      for (RobotQuadrant quadrant : RobotQuadrant.values)
      {
         String name = quadrant.getCamelCaseNameForStartOfExpression() + "FootInContact";
         BooleanYoVariable footContactBoolean = new BooleanYoVariable(name, registry);
         footContactBooleans.set(quadrant, footContactBoolean);
      }

      parentRegistry.addChild(registry);
   }

   private void initializeVisualization()
   {
      OneDoFJoint[] oneDoFJoints = sdfFullRobotModelForViz.getOneDoFJoints();
      for (int i = 0; i < oneDoFJoints.length; i++)
      {
         String prefix = "StateEstimator" + oneDoFJoints[i].getName();
         ReferenceFrame referenceFrame = oneDoFJoints[i].getFrameBeforeJoint();
         YoGraphicReferenceFrame vizReferenceFrame = new YoGraphicReferenceFrame(prefix, referenceFrame, registry, 0.4, YoAppearance.AliceBlue());
         graphicReferenceFrames.add(vizReferenceFrame);
         yoGraphicsListRegistry.registerYoGraphic("KinematicsBasedStateEstimator", vizReferenceFrame);
      }
   }

   @Override
   public boolean isFootInContact(RobotQuadrant quadrant)
   {
      return footSwitchUpdater.isFootInContactWithGround(quadrant);
   }

   public void initialize()
   {
      updateFeetContactStatus();

      jointStateUpdater.initialize();
      sdfFullRobotModelForViz.updateFrames();
      comRotationalStateUpdater.initialize();
      comLinearStateUpdater.initialize();
   }

   @Override
   public void doControl()
   {
      if (isEnabled.getBooleanValue())
      {
         if (!hasBeenInitialized)
         {
            initialize();
            hasBeenInitialized = true;
         }

         updateFeetContactStatus();

         jointStateUpdater.updateJointState();
         
         comRotationalStateUpdater.updateRootJointOrientationAndAngularVelocity();
         
         comLinearStateUpdater.updateRootJointPositionAndLinearVelocity(feetInContact, feetNotInContact);

         sdfFullRobotModelForViz.updateFrames();
         updateViz();
      }
   }

   private void updateViz()
   {
      for (int i = 0; i < graphicReferenceFrames.size(); i++)
         graphicReferenceFrames.get(i).update();
   }

   @Override
   public double getCurrentTime()
   {
      return TimeTools.nanoSecondstoSeconds(sensorOutputMapReadOnly.getTimestamp());
   }

   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   private void updateFeetContactStatus()
   {
      feetInContact.clear();
      feetNotInContact.clear();
      for (RobotQuadrant quadrant : RobotQuadrant.values())
      {
         if (footSwitchUpdater.isFootInContactWithGround(quadrant))
         {
            feetInContact.add(quadrant);
            footContactBooleans.get(quadrant).set(true);
         }
         else
         {
            feetNotInContact.add(quadrant);
            footContactBooleans.get(quadrant).set(false);
         }
      }
   }

   @Override
   public void enable()
   {
      isEnabled.set(true);
   }
}
