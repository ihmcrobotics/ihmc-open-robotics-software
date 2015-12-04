package us.ihmc.quadrupedRobotics.stateEstimator.kinematicsBased;

import java.util.ArrayList;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.quadrupedRobotics.sensorProcessing.sensorProcessors.FootSwitchUpdater;
import us.ihmc.quadrupedRobotics.stateEstimator.QuadrupedStateEstimator;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.SixDoFJoint;
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

   private final SDFFullRobotModel sdfFullRobotModelFromSensor; // hack for visualization
   private final SDFFullRobotModel sdfFullRobotModelForViz;
   
   private final JointStateUpdater jointStateUpdater;
   private final FootSwitchUpdater footSwitchUpdater;
   private final CenterOfMassLinearStateUpdater comLinearStateUpdater;
   
   private final SensorOutputMapReadOnly sensorOutputMapReadOnly;
   
   private final ArrayList<YoGraphicReferenceFrame> graphicReferenceFrames = new  ArrayList<>();
   
   //Hack constructor for visualization
   public QuadrupedKinematicsBasedStateEstimator(FullInverseDynamicsStructure inverseDynamicsStructure, SensorOutputMapReadOnly sensorOutputMapReadOnly,
         FootSwitchUpdater footSwitchUpdater, CenterOfMassLinearStateUpdater comLinearStateUpdater, SDFFullRobotModel sdfFullRobotModelFromSensor, SDFFullRobotModel sdfFullRobotModelForViz, YoVariableRegistry parentRegistry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this.yoGraphicsListRegistry = yoGraphicsListRegistry;
      
      jointStateUpdater = new JointStateUpdater(inverseDynamicsStructure, sensorOutputMapReadOnly, null, registry);
      
      this.footSwitchUpdater = footSwitchUpdater;
      this.sensorOutputMapReadOnly = sensorOutputMapReadOnly;
      
      this.comLinearStateUpdater = comLinearStateUpdater;
      
      this.sdfFullRobotModelForViz = sdfFullRobotModelForViz;
      this.sdfFullRobotModelFromSensor = sdfFullRobotModelFromSensor;
      
      if(this.sdfFullRobotModelForViz != null)
         initializeVisualization();
      
      parentRegistry.addChild(registry);
      
   }
   
   public QuadrupedKinematicsBasedStateEstimator(FullInverseDynamicsStructure inverseDynamicsStructure, SensorOutputMapReadOnly sensorOutputMapReadOnly,
         FootSwitchUpdater footSwitchUpdater, CenterOfMassLinearStateUpdater comLinearStateUpdater, SDFFullRobotModel sdfFullRobotModelForViz, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this.yoGraphicsListRegistry = yoGraphicsListRegistry;
      
      jointStateUpdater = new JointStateUpdater(inverseDynamicsStructure, sensorOutputMapReadOnly, null, registry);

      this.footSwitchUpdater = footSwitchUpdater;
      this.sensorOutputMapReadOnly = sensorOutputMapReadOnly;

      this.comLinearStateUpdater = comLinearStateUpdater;
      
      this.sdfFullRobotModelForViz = sdfFullRobotModelForViz;
      this.sdfFullRobotModelFromSensor = null;
      
      if(this.sdfFullRobotModelForViz != null)
         initializeVisualization();
   }

  private void initializeVisualization()
   {
     OneDoFJoint[] oneDoFJoints = sdfFullRobotModelForViz.getOneDoFJoints();
      for (int i = 0; i < oneDoFJoints.length; i++)
      {
         String prefix = "StateEstimator" + oneDoFJoints[i].getName();
         ReferenceFrame referenceFrame = oneDoFJoints[i].getFrameBeforeJoint();
         YoGraphicReferenceFrame vizReferenceFrame = new YoGraphicReferenceFrame(prefix, referenceFrame, registry, 0.2, YoAppearance.AliceBlue());
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
      jointStateUpdater.initialize();
      comLinearStateUpdater.initialize();
   }
   
   @Override
   public void doControl()
   {
      jointStateUpdater.updateJointState();
      comLinearStateUpdater.updateCenterOfMassLinearState();
      
      if(sdfFullRobotModelForViz != null)
         updateViz();
   }

   private void updateViz()
   {
      //very hackish, will need  to be deleted once the state estimator is working
      OneDoFJoint[] oneDoFJointsFromSensor = sdfFullRobotModelFromSensor.getOneDoFJoints();
      OneDoFJoint[] oneDoFJointsForViz = sdfFullRobotModelForViz.getOneDoFJoints();
     
     for (int i = 0; i < oneDoFJointsForViz.length; i++)
     {
        OneDoFJoint oneDoFJointFromSensor = oneDoFJointsFromSensor[i];
        OneDoFJoint oneDoFJointForViz = oneDoFJointsForViz[i];

        oneDoFJointForViz.setQ(oneDoFJointFromSensor.getQ());
        oneDoFJointForViz.setQd(oneDoFJointFromSensor.getQd());
        oneDoFJointForViz.setTau(oneDoFJointFromSensor.getTau());
        oneDoFJointForViz.setTauMeasured(oneDoFJointFromSensor.getTauMeasured());
        
     }
     
     SixDoFJoint rootJointFromSensor = sdfFullRobotModelFromSensor.getRootJoint();
     SixDoFJoint rootJointForViz = sdfFullRobotModelForViz.getRootJoint();
     
     rootJointForViz.setPositionAndRotation(rootJointFromSensor.getJointTransform3D());
     
     sdfFullRobotModelForViz.updateFrames();
     for(int i = 0; i < graphicReferenceFrames.size(); i++)
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
}
