package us.ihmc.darpaRoboticsChallenge.controllers;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

import javax.vecmath.Point2d;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ListOfPointsContactablePlaneBody;
import us.ihmc.commonWalkingControlModules.controllers.ControllerFactory;
import us.ihmc.commonWalkingControlModules.controllers.LidarControllerInterface;
import us.ihmc.commonWalkingControlModules.controllers.RobotControllerUpdatablesAdapter;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.HighLevelHumanoidControllerFactory;
import us.ihmc.commonWalkingControlModules.referenceFrames.CommonWalkingReferenceFrames;
import us.ihmc.commonWalkingControlModules.sensors.FootSwitchInterface;
import us.ihmc.commonWalkingControlModules.sensors.WrenchBasedFootSwitch;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.robotSide.SideDependentList;
import us.ihmc.sensorProcessing.sensors.ForceSensorData;
import us.ihmc.sensorProcessing.sensors.ForceSensorDataHolder;
import us.ihmc.utilities.humanoidRobot.model.FullRobotModel;
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
   private final double contactTresholdForce;
   private final SideDependentList<String> footSensorNames;

    public DRCRobotMomentumBasedControllerFactory(HighLevelHumanoidControllerFactory highLevelHumanoidControllerFactory, double contactTresholdForce, SideDependentList<String> footSensorNames)
   {
      this.highLevelHumanoidControllerFactory = highLevelHumanoidControllerFactory;
      this.contactTresholdForce = contactTresholdForce;
      this.footSensorNames = footSensorNames;
   }

   @Override
   public RobotController getController(FullRobotModel fullRobotModel, CommonWalkingReferenceFrames referenceFrames, double controlDT, double gravity, DoubleYoVariable yoTime,
                                        DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry, GUISetterUpperRegistry guiSetterUpperRegistry, TwistCalculator twistCalculator,
                                        CenterOfMassJacobian centerOfMassJacobian, ForceSensorDataHolder forceSensorDataHolder, LidarControllerInterface lidarControllerInterface,
                                        GlobalDataProducer dataProducer, SideDependentList<ArrayList<Point2d>> contactPointsArrayList)
   {
      YoVariableRegistry specificRegistry = new YoVariableRegistry("specific");

      SideDependentList<ContactablePlaneBody> bipedFeet = new SideDependentList<ContactablePlaneBody>();
      for (RobotSide robotSide : RobotSide.values)
      {
         RigidBody footBody = fullRobotModel.getFoot(robotSide);
         ReferenceFrame soleFrame = referenceFrames.getSoleFrame(robotSide);
         ListOfPointsContactablePlaneBody foot = new ListOfPointsContactablePlaneBody(footBody, soleFrame, contactPointsArrayList.get(robotSide));

         bipedFeet.put(robotSide, foot);
      }


      double gravityZ = Math.abs(gravity);
      double totalRobotWeight = TotalMassCalculator.computeSubTreeMass(fullRobotModel.getElevator()) * gravityZ;

      SideDependentList<FootSwitchInterface> footSwitches = createFootSwitches(bipedFeet, forceSensorDataHolder, totalRobotWeight, dynamicGraphicObjectsListRegistry, specificRegistry);




      Map<OneDoFJoint, Double> initialPositionControlKpGains = new HashMap<OneDoFJoint, Double>();
      Map<OneDoFJoint, Double> initialPositionControlKdGains = new HashMap<OneDoFJoint, Double>();

      // TODO: Generalize setting up the gain maps.  No knowledge of the model at this stage in the game but this shouldn't be Atlas specific
//      AtlasJointPDGains.createMaps(fullRobotModel, initialPositionControlKpGains, initialPositionControlKdGains);

      RobotController highLevelHumanoidController = highLevelHumanoidControllerFactory.create(fullRobotModel,
            initialPositionControlKpGains, initialPositionControlKdGains, referenceFrames, null, yoTime, gravityZ, twistCalculator, centerOfMassJacobian,
            bipedFeet, controlDT, footSwitches, lidarControllerInterface, dynamicGraphicObjectsListRegistry, specificRegistry, guiSetterUpperRegistry, forceSensorDataHolder);

      highLevelHumanoidController.getYoVariableRegistry().addChild(specificRegistry);


      if (dynamicGraphicObjectsListRegistry!=null)
      {
         RobotControllerUpdatablesAdapter  highLevelHumanoidControllerUpdatables = new RobotControllerUpdatablesAdapter(highLevelHumanoidController);


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
         ForceSensorData footForceSensor = forceSensorDataHolder.getByName(footSensorNames.get(robotSide));
         WrenchBasedFootSwitch wrenchBasedFootSwitch = new WrenchBasedFootSwitch(bipedFeet.get(robotSide).getName(), footForceSensor, 0.02, totalRobotWeight, bipedFeet.get(robotSide),
                                                          dynamicGraphicObjectsListRegistry, contactTresholdForce, registry);
         footSwitches.put(robotSide, wrenchBasedFootSwitch);
      }

      return footSwitches;
   }
}
