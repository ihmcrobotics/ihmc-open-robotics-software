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

   private final SensorOutputMapReadOnly sensorOutputMapReadOnly;

   private final ArrayList<YoGraphicReferenceFrame> graphicReferenceFrames = new ArrayList<>();

   private final ArrayList<RobotQuadrant> feetInContact = new ArrayList<>();
   private final ArrayList<RobotQuadrant> feetNotInContact = new ArrayList<>();

   private final QuadrantDependentList<BooleanYoVariable> footContactBooleans = new QuadrantDependentList<>();
   
   public QuadrupedKinematicsBasedStateEstimator(FullInverseDynamicsStructure inverseDynamicsStructure, SensorOutputMapReadOnly sensorOutputMapReadOnly,
         FootSwitchUpdater footSwitchUpdater, JointStateUpdater jointStateUpdater, CenterOfMassLinearStateUpdater comLinearStateUpdater,
         SDFFullRobotModel sdfFullRobotModelForViz, YoVariableRegistry parentRegistry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this.yoGraphicsListRegistry = yoGraphicsListRegistry;

      this.jointStateUpdater = jointStateUpdater;

      this.footSwitchUpdater = footSwitchUpdater;
      this.sensorOutputMapReadOnly = sensorOutputMapReadOnly;

      this.comLinearStateUpdater = comLinearStateUpdater;

      this.sdfFullRobotModelForViz = sdfFullRobotModelForViz;

      if (this.sdfFullRobotModelForViz != null)
         initializeVisualization();

      for(RobotQuadrant quadrant : RobotQuadrant.values)
      {
         String name = quadrant.getCamelCaseNameForStartOfExpression() + "FootInContact";
         BooleanYoVariable footContactBoolean = new BooleanYoVariable(name, registry);
         footContactBooleans.put(quadrant, footContactBoolean);
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
      comLinearStateUpdater.initialize();
   }

   @Override
   public void doControl()
   {
      updateFeetContactStatus();

      jointStateUpdater.updateJointState();
      comLinearStateUpdater.updateCenterOfMassLinearState(feetInContact, feetNotInContact);

      sdfFullRobotModelForViz.updateFrames();
      updateViz();
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
}
