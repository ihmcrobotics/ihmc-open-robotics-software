package us.ihmc.darpaRoboticsChallenge.controllers;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import javax.vecmath.Point2d;
import javax.vecmath.Vector3d;

import us.ihmc.atlas.parameters.AtlasJointPDGains;
import us.ihmc.atlas.visualization.CenterOfPressureVisualizer;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ListOfPointsContactablePlaneBody;
import us.ihmc.commonWalkingControlModules.controllers.ControllerFactory;
import us.ihmc.commonWalkingControlModules.controllers.HandControllerInterface;
import us.ihmc.commonWalkingControlModules.controllers.LidarControllerInterface;
import us.ihmc.commonWalkingControlModules.controllers.RobotControllerUpdatablesAdapter;
import us.ihmc.commonWalkingControlModules.controllers.Updatable;
import us.ihmc.commonWalkingControlModules.dynamics.FullRobotModel;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.HighLevelHumanoidControllerFactory;
import us.ihmc.commonWalkingControlModules.referenceFrames.CommonWalkingReferenceFrames;
import us.ihmc.commonWalkingControlModules.sensors.FootSwitchInterface;
import us.ihmc.commonWalkingControlModules.visualizer.ForceSensorDataVisualizer;
import us.ihmc.darpaRoboticsChallenge.DRCConfigParameters;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotParameters;
import us.ihmc.darpaRoboticsChallenge.sensors.AtlasWristFeetYoVariables;
import us.ihmc.darpaRoboticsChallenge.sensors.WrenchBasedFootSwitch;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.robotSide.SideDependentList;
import us.ihmc.sensorProcessing.AtlasWristFeetSensorServer;
import us.ihmc.sensorProcessing.sensors.ForceSensorData;
import us.ihmc.sensorProcessing.sensors.ForceSensorDataHolder;
import us.ihmc.sensorProcessing.stateEstimation.StateEstimationDataFromController;
import us.ihmc.utilities.io.streamingData.GlobalDataProducer;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.CenterOfMassJacobian;
import us.ihmc.utilities.screwTheory.OneDoFJoint;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.TotalMassCalculator;
import us.ihmc.utilities.screwTheory.TwistCalculator;

import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.gui.GUISetterUpperRegistry;
import com.yobotics.simulationconstructionset.robotController.RobotController;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;

public class DRCRobotMomentumBasedControllerFactory implements ControllerFactory
{
   private final HighLevelHumanoidControllerFactory highLevelHumanoidControllerFactory;
   private final boolean useGazeboPhysics;

   public DRCRobotMomentumBasedControllerFactory(HighLevelHumanoidControllerFactory highLevelHumanoidControllerFactory)
   {
      this(highLevelHumanoidControllerFactory, false);
   }

   public DRCRobotMomentumBasedControllerFactory(HighLevelHumanoidControllerFactory highLevelHumanoidControllerFactory, boolean useGazeboPhysics)
   {
      this.highLevelHumanoidControllerFactory = highLevelHumanoidControllerFactory;
      this.useGazeboPhysics = useGazeboPhysics;
   }

   @Override
   public RobotController getController(RigidBody estimationLink, ReferenceFrame estimationFrame, FullRobotModel fullRobotModel,
                                        CommonWalkingReferenceFrames referenceFrames, double controlDT, double gravity, DoubleYoVariable yoTime,
                                        DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry, GUISetterUpperRegistry guiSetterUpperRegistry, TwistCalculator twistCalculator,
                                        CenterOfMassJacobian centerOfMassJacobian, ForceSensorDataHolder forceSensorDataHolder, SideDependentList<HandControllerInterface> handControllers,
                                        LidarControllerInterface lidarControllerInterface, StateEstimationDataFromController stateEstimationDataFromControllerSink, GlobalDataProducer dataProducer)
   {

      ArrayList<Vector3d> contactPointsArrayList = DRCRobotParameters.DRC_ROBOT_GROUND_CONTACT_POINT_OFFSET_FROM_FOOT;

      YoVariableRegistry specificRegistry = new YoVariableRegistry("specific");


      SideDependentList<ContactablePlaneBody> bipedFeet = new SideDependentList<ContactablePlaneBody>();
      for (RobotSide robotSide : RobotSide.values)
      {
         RigidBody footBody = fullRobotModel.getFoot(robotSide);
         ReferenceFrame soleFrame = referenceFrames.getSoleFrame(robotSide);
         ListOfPointsContactablePlaneBody foot = new ListOfPointsContactablePlaneBody(footBody, soleFrame, this.createContactPoints(contactPointsArrayList));
         
         bipedFeet.put(robotSide, foot);
      }


      double gravityZ = Math.abs(gravity);
      double totalRobotWeight = TotalMassCalculator.computeSubTreeMass(fullRobotModel.getElevator()) * gravityZ;
      
      SideDependentList<FootSwitchInterface> footSwitches = createFootSwitches(bipedFeet, forceSensorDataHolder, totalRobotWeight, dynamicGraphicObjectsListRegistry, specificRegistry);
      



      Map<OneDoFJoint, Double> initialPositionControlKpGains = new HashMap<OneDoFJoint, Double>();
      Map<OneDoFJoint, Double> initialPositionControlKdGains = new HashMap<OneDoFJoint, Double>();
      
      AtlasJointPDGains.createMaps(fullRobotModel, initialPositionControlKpGains, initialPositionControlKdGains);

      RobotController highLevelHumanoidController = highLevelHumanoidControllerFactory.create(estimationLink, estimationFrame, fullRobotModel,
            initialPositionControlKpGains, initialPositionControlKdGains, 
            referenceFrames, null, yoTime, gravityZ, twistCalculator, centerOfMassJacobian, bipedFeet,
            controlDT, footSwitches, handControllers, lidarControllerInterface,
            stateEstimationDataFromControllerSink, dynamicGraphicObjectsListRegistry, specificRegistry,
            guiSetterUpperRegistry, null, forceSensorDataHolder);
      
      highLevelHumanoidController.getYoVariableRegistry().addChild(specificRegistry);


      if (dynamicGraphicObjectsListRegistry!=null)
      {
         RobotControllerUpdatablesAdapter  highLevelHumanoidControllerUpdatables = new RobotControllerUpdatablesAdapter(highLevelHumanoidController);

         final ForceSensorDataVisualizer forceSensorDataVisualizer=new ForceSensorDataVisualizer(fullRobotModel, forceSensorDataHolder, dynamicGraphicObjectsListRegistry, specificRegistry);
         final CenterOfPressureVisualizer centerOfPressureVisualizer=new CenterOfPressureVisualizer(fullRobotModel, forceSensorDataHolder, dynamicGraphicObjectsListRegistry, specificRegistry);
         final AtlasWristFeetYoVariables wristFeetVariables = new AtlasWristFeetYoVariables(forceSensorDataHolder, specificRegistry);
         final AtlasWristFeetSensorServer wristFeetNetwork = dataProducer == null ? null : new AtlasWristFeetSensorServer(forceSensorDataHolder,dataProducer);

         highLevelHumanoidControllerUpdatables.addUpdatable(
            new Updatable()
            {
               @Override
               public void update(double time)
               {
                    forceSensorDataVisualizer.visualize();
                    centerOfPressureVisualizer.visualize();
                    wristFeetVariables.update();
                    if( wristFeetNetwork != null)
                       wristFeetNetwork.update();
               }
            });
         
         highLevelHumanoidController = highLevelHumanoidControllerUpdatables;
      }
      return highLevelHumanoidController;      
   }

   private SideDependentList<FootSwitchInterface> createFootSwitches(SideDependentList<ContactablePlaneBody> bipedFeet,
         ForceSensorDataHolder forceSensorDataHolder, double totalRobotWeight, DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry, YoVariableRegistry registry)
   {
      SideDependentList<FootSwitchInterface> footSwitches = new SideDependentList<FootSwitchInterface>();
      
      for (RobotSide robotSide : RobotSide.values)
      {
         ForceSensorData footForceSensor = forceSensorDataHolder.getByName(robotSide.getShortLowerCaseName() + "_leg_akx");
         WrenchBasedFootSwitch wrenchBasedFootSwitch = new WrenchBasedFootSwitch(bipedFeet.get(robotSide).getName(), footForceSensor, 0.02, totalRobotWeight, bipedFeet.get(robotSide),
                                                          dynamicGraphicObjectsListRegistry, useGazeboPhysics, registry);
         footSwitches.put(robotSide, wrenchBasedFootSwitch);
      }

      return footSwitches;
   }

   @Override
   public double getControlDT()
   {
      return DRCConfigParameters.CONTROL_DT;
   }
   
   private List<Point2d> createContactPoints(ArrayList<Vector3d> list)
   {
      ArrayList<Point2d> contactPoints = new ArrayList<Point2d>();
      for(int i = 0; i<list.size(); i++)
      {
         contactPoints.add(new Point2d(list.get(i).x, list.get(i).y));
      }
      return contactPoints;
   }

}
