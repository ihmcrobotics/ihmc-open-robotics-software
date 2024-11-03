package us.ihmc.avatar.logProcessor;

import static us.ihmc.yoVariables.tools.YoGeometryNameTools.createQsName;
import static us.ihmc.yoVariables.tools.YoGeometryNameTools.createQxName;
import static us.ihmc.yoVariables.tools.YoGeometryNameTools.createQyName;
import static us.ihmc.yoVariables.tools.YoGeometryNameTools.createQzName;
import static us.ihmc.yoVariables.tools.YoGeometryNameTools.createXName;
import static us.ihmc.yoVariables.tools.YoGeometryNameTools.createYName;
import static us.ihmc.yoVariables.tools.YoGeometryNameTools.createZName;

import java.util.ArrayList;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FootControlModule;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FootControlModule.ConstraintType;
import us.ihmc.commonWalkingControlModules.controllers.Updatable;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ContactableBodiesFactory;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.commonWalkingControlModules.momentumBasedController.PlaneContactWrenchProcessor;
import us.ihmc.commonWalkingControlModules.sensors.footSwitch.WrenchBasedFootSwitch;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactableFoot;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.humanoidRobotics.model.CenterOfMassStateProvider;
import us.ihmc.mecano.spatial.interfaces.WrenchReadOnly;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.contactable.ContactablePlaneBody;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.sensors.FootSwitchInterface;
import us.ihmc.sensorProcessing.simulatedSensors.SDFPerfectSimulatedSensorReader;
import us.ihmc.sensorProcessing.stateEstimation.evaluation.FullInverseDynamicsStructure;
import us.ihmc.simulationconstructionset.FloatingRootJointRobot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.wholeBodyController.RobotContactPointParameters;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint2D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameQuaternion;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.registry.YoVariableHolder;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;

public class LogDataProcessorHelper
{
   private final FullHumanoidRobotModel fullRobotModel;
   private final FullInverseDynamicsStructure inverseDynamicsStructure;
   private final CenterOfMassStateProvider centerOfMassStateProvider;
   private final HumanoidReferenceFrames referenceFrames;
   private final SDFPerfectSimulatedSensorReader sensorReader;
   private final LogDataRawSensorMap rawSensorMap;
   private final SideDependentList<ContactableFoot> contactableFeet;

   private final SideDependentList<YoFramePoint2D> cops = new SideDependentList<>();
   private final SideDependentList<YoFramePoint2D> desiredCoPs = new SideDependentList<>();
   private final SideDependentList<YoEnum<?>> footStates = new SideDependentList<>();

   private final double controllerDT;
   private final WalkingControllerParameters walkingControllerParameters;

   private final SideDependentList<FootSwitchInterface> stateEstimatorFootSwitches;

   private final SimulationConstructionSet scs;

   private final UpdatableHighLevelHumanoidControllerToolbox controllerToolbox;
   private final ArrayList<Updatable> updatables = new ArrayList<>();
   private final YoDouble yoTime;

   public LogDataProcessorHelper(DRCRobotModel model, SimulationConstructionSet scs, FloatingRootJointRobot sdfRobot)
   {
      this.scs = scs;
      fullRobotModel = model.createFullRobotModel();
      centerOfMassStateProvider = CenterOfMassStateProvider.createJacobianBasedStateCalculator(fullRobotModel.getElevator(), ReferenceFrame.getWorldFrame());
      referenceFrames = new HumanoidReferenceFrames(fullRobotModel);
      sensorReader = new SDFPerfectSimulatedSensorReader(sdfRobot, fullRobotModel, referenceFrames);
      rawSensorMap = new LogDataRawSensorMap(fullRobotModel, scs);

      inverseDynamicsStructure = new FullInverseDynamicsStructure(fullRobotModel.getElevator(), fullRobotModel.getPelvis(), fullRobotModel.getRootJoint());

      controllerDT = model.getControllerDT();
      this.walkingControllerParameters = model.getWalkingControllerParameters();

      RobotContactPointParameters<RobotSide> contactPointParameters = model.getContactPointParameters();

      ContactableBodiesFactory<RobotSide> contactableBodiesFactory = new ContactableBodiesFactory<>();
      contactableBodiesFactory.setFootContactPoints(contactPointParameters.getFootContactPoints());
      contactableBodiesFactory.setToeContactParameters(contactPointParameters.getControllerToeContactPoints(),
                                                       contactPointParameters.getControllerToeContactLines());
      contactableBodiesFactory.setFullRobotModel(fullRobotModel);
      contactableBodiesFactory.setReferenceFrames(referenceFrames);

      contactableFeet = new SideDependentList<>(contactableBodiesFactory.createFootContactableFeet());
      contactableBodiesFactory.disposeFactory();

      for (RobotSide robotSide : RobotSide.values)
      {
         ReferenceFrame soleFrame = referenceFrames.getSoleFrame(robotSide);

         String side = robotSide.getCamelCaseNameForMiddleOfExpression();
         String bodyName = contactableFeet.get(robotSide).getName();
         String copNamePrefix = bodyName + "StateEstimator";
         String copNamespace = copNamePrefix + WrenchBasedFootSwitch.class.getSimpleName();
         String copName = copNamePrefix + "ResolvedCoP";
         YoDouble copx = (YoDouble) scs.findVariable(copNamespace, copName + "X");
         YoDouble copy = (YoDouble) scs.findVariable(copNamespace, copName + "Y");
         if (copx != null && copy != null)
         {
            YoFramePoint2D cop = new YoFramePoint2D(copx, copy, soleFrame);
            cops.put(robotSide, cop);
         }

         String desiredCoPNamespace = PlaneContactWrenchProcessor.class.getSimpleName();
         String desiredCoPName = side + "SoleCoP2d";
         YoDouble desiredCoPx = (YoDouble) scs.findVariable(desiredCoPNamespace, desiredCoPName + "X");
         YoDouble desiredCoPy = (YoDouble) scs.findVariable(desiredCoPNamespace, desiredCoPName + "Y");
         YoFramePoint2D desiredCoP = new YoFramePoint2D(desiredCoPx, desiredCoPy, soleFrame);
         desiredCoPs.put(robotSide, desiredCoP);

         String sidePrefix = robotSide.getCamelCaseNameForStartOfExpression();
         String namePrefix = sidePrefix + "Foot";
         String footStateNamespace = sidePrefix + FootControlModule.class.getSimpleName();
         String footStateName = namePrefix + "State";
         @SuppressWarnings("unchecked")
         YoEnum<?> footState = (YoEnum<ConstraintType>) scs.findVariable(footStateNamespace, footStateName);
         footStates.put(robotSide, footState);
      }

      stateEstimatorFootSwitches = createStateEstimatorFootSwitches(scs);

      double omega0 = walkingControllerParameters.getOmega0();
      double gravityZ = 9.81;
      String controllerTimeNamespace = null; // FIXME DRCControllerThread.class.getSimpleName();
      yoTime = (YoDouble) scs.findVariable(controllerTimeNamespace, "controllerTime");

      controllerToolbox = new UpdatableHighLevelHumanoidControllerToolbox(scs,
                                                                          fullRobotModel,
                                                                          centerOfMassStateProvider,
                                                                          referenceFrames,
                                                                          stateEstimatorFootSwitches,
                                                                          null,
                                                                          yoTime,
                                                                          gravityZ,
                                                                          omega0,
                                                                          contactableFeet,
                                                                          controllerDT,
                                                                          updatables,
                                                                          null,
                                                                          null);
   }

   private SideDependentList<FootSwitchInterface> createStateEstimatorFootSwitches(YoVariableHolder yoVariableHolder)
   {
      SideDependentList<FootSwitchInterface> footSwitches = new SideDependentList<FootSwitchInterface>();

      for (final RobotSide robotSide : RobotSide.values)
      {
         String namePrefix = contactableFeet.get(robotSide).getName() + "StateEstimator";
         String namespaceEnding = namePrefix + WrenchBasedFootSwitch.class.getSimpleName();
         final YoBoolean hasFootHitGround = (YoBoolean) yoVariableHolder.findVariable(namespaceEnding, namePrefix + "FilteredFootHitGround");
         final YoBoolean forceMagnitudePastThreshold = (YoBoolean) yoVariableHolder.findVariable(namespaceEnding, namePrefix + "ForcePastThresh");
         final YoDouble footLoadPercentage = (YoDouble) yoVariableHolder.findVariable(namespaceEnding, namePrefix + "FootLoadPercentage");

         FootSwitchInterface footSwitch = new FootSwitchInterface()
         {

            @Override
            public void reset()
            {
            }

            @Override
            public boolean hasFootHitGroundSensitive()
            {
               return forceMagnitudePastThreshold.getBooleanValue();
            }

            @Override
            public boolean hasFootHitGroundFiltered()
            {
               return hasFootHitGround.getBooleanValue();
            }

            @Override
            public ReferenceFrame getMeasurementFrame()
            {
               return null;
            }

            @Override
            public double getFootLoadPercentage()
            {
               return footLoadPercentage.getDoubleValue();
            }

            @Override
            public WrenchReadOnly getMeasuredWrench()
            {
               return null;
            }

            @Override
            public FramePoint2DReadOnly getCenterOfPressure()
            {
               return cops.get(robotSide);
            }
         };

         footSwitches.put(robotSide, footSwitch);
      }

      return footSwitches;
   }

   public void update()
   {
      sensorReader.read();
      controllerToolbox.update();
   }

   public FullHumanoidRobotModel getFullRobotModel()
   {
      return fullRobotModel;
   }

   public FullInverseDynamicsStructure getInverseDynamicsStructure()
   {
      return inverseDynamicsStructure;
   }

   public HumanoidReferenceFrames getReferenceFrames()
   {
      return referenceFrames;
   }

   public LogDataRawSensorMap getRawSensorMap()
   {
      return rawSensorMap;
   }

   public SideDependentList<? extends ContactablePlaneBody> getContactableFeet()
   {
      return contactableFeet;
   }

   public SideDependentList<FootSwitchInterface> getStateEstimatorFootSwitches()
   {
      return stateEstimatorFootSwitches;
   }

   public ConstraintType getCurrenFootState(RobotSide robotSide)
   {
      return ConstraintType.valueOf(footStates.get(robotSide).getStringValue());
   }

   public void getMeasuredCoP(RobotSide robotSide, FramePoint2D copToPack)
   {
      copToPack.setIncludingFrame(cops.get(robotSide));
   }

   public void getDesiredCoP(RobotSide robotSide, FramePoint2D desiredCoPToPack)
   {
      desiredCoPToPack.setIncludingFrame(desiredCoPs.get(robotSide));
   }

   public double getControllerDT()
   {
      return controllerDT;
   }

   public WalkingControllerParameters getWalkingControllerParameters()
   {
      return walkingControllerParameters;
   }

   public YoVariableHolder getLogYoVariableHolder()
   {
      return scs;
   }

   public HighLevelHumanoidControllerToolbox getHighLevelHumanoidControllerToolbox()
   {
      return controllerToolbox;
   }

   public YoFramePoint3D findYoFramePoint(String pointPrefix, ReferenceFrame pointFrame)
   {
      return findYoFramePoint(pointPrefix, "", pointFrame);
   }

   public YoFramePoint3D findYoFramePoint(String pointPrefix, String pointSuffix, ReferenceFrame pointFrame)
   {
      YoDouble x = (YoDouble) scs.findVariable(createXName(pointPrefix, pointSuffix));
      YoDouble y = (YoDouble) scs.findVariable(createYName(pointPrefix, pointSuffix));
      YoDouble z = (YoDouble) scs.findVariable(createZName(pointPrefix, pointSuffix));
      if (x == null || y == null || z == null)
         return null;
      else
         return new YoFramePoint3D(x, y, z, pointFrame);
   }

   public YoFrameVector3D findYoFrameVector(String vectorPrefix, ReferenceFrame vectorFrame)
   {
      return findYoFrameVector(vectorPrefix, "", vectorFrame);
   }

   public YoFrameVector3D findYoFrameVector(String vectorPrefix, String vectorSuffix, ReferenceFrame vectorFrame)
   {
      YoDouble x = (YoDouble) scs.findVariable(createXName(vectorPrefix, vectorSuffix));
      YoDouble y = (YoDouble) scs.findVariable(createYName(vectorPrefix, vectorSuffix));
      YoDouble z = (YoDouble) scs.findVariable(createZName(vectorPrefix, vectorSuffix));
      if (x == null || y == null || z == null)
         return null;
      else
         return new YoFrameVector3D(x, y, z, vectorFrame);
   }

   public YoFrameQuaternion findYoFrameQuaternion(String quaternionPrefix, ReferenceFrame quaternionFrame)
   {
      return findYoFrameQuaternion(quaternionPrefix, "", quaternionFrame);
   }

   public YoFrameQuaternion findYoFrameQuaternion(String quaternionPrefix, String quaternionSuffix, ReferenceFrame quaternionFrame)
   {
      YoDouble qx = (YoDouble) scs.findVariable(createQxName(quaternionPrefix, quaternionSuffix));
      YoDouble qy = (YoDouble) scs.findVariable(createQyName(quaternionPrefix, quaternionSuffix));
      YoDouble qz = (YoDouble) scs.findVariable(createQzName(quaternionPrefix, quaternionSuffix));
      YoDouble qs = (YoDouble) scs.findVariable(createQsName(quaternionPrefix, quaternionSuffix));

      if (qx == null || qy == null || qz == null || qs == null)
         return null;
      else
         return new YoFrameQuaternion(qx, qy, qz, qs, quaternionFrame);
   }
}
