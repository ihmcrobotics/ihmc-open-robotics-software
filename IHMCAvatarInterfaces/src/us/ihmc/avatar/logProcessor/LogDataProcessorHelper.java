package us.ihmc.avatar.logProcessor;

import static us.ihmc.robotics.math.frames.YoFrameVariableNameTools.createQsName;
import static us.ihmc.robotics.math.frames.YoFrameVariableNameTools.createQxName;
import static us.ihmc.robotics.math.frames.YoFrameVariableNameTools.createQyName;
import static us.ihmc.robotics.math.frames.YoFrameVariableNameTools.createQzName;
import static us.ihmc.robotics.math.frames.YoFrameVariableNameTools.createXName;
import static us.ihmc.robotics.math.frames.YoFrameVariableNameTools.createYName;
import static us.ihmc.robotics.math.frames.YoFrameVariableNameTools.createZName;

import java.util.ArrayList;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FootControlModule;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FootControlModule.ConstraintType;
import us.ihmc.commonWalkingControlModules.controllers.Updatable;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ContactableBodiesFactory;
import us.ihmc.commonWalkingControlModules.momentumBasedController.GeometricJacobianHolder;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.commonWalkingControlModules.momentumBasedController.PlaneContactWrenchProcessor;
import us.ihmc.commonWalkingControlModules.sensors.footSwitch.WrenchBasedFootSwitch;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactableFoot;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.dataStructures.YoVariableHolder;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFramePoint2d;
import us.ihmc.robotics.math.frames.YoFrameQuaternion;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.TwistCalculator;
import us.ihmc.robotics.screwTheory.Wrench;
import us.ihmc.robotics.sensors.FootSwitchInterface;
import us.ihmc.sensorProcessing.simulatedSensors.SDFPerfectSimulatedSensorReader;
import us.ihmc.sensorProcessing.stateEstimation.evaluation.FullInverseDynamicsStructure;
import us.ihmc.simulationconstructionset.FloatingRootJointRobot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.wholeBodyController.DRCControllerThread;

public class LogDataProcessorHelper
{
   private final FullHumanoidRobotModel fullRobotModel;
   private final FullInverseDynamicsStructure inverseDynamicsStructure;
   private final HumanoidReferenceFrames referenceFrames;
   private final SDFPerfectSimulatedSensorReader sensorReader;
   private final LogDataRawSensorMap rawSensorMap;
   private final TwistCalculator twistCalculator;
   private final SideDependentList<ContactableFoot> contactableFeet;

   private final SideDependentList<YoFramePoint2d> cops = new SideDependentList<>();
   private final SideDependentList<YoFramePoint2d> desiredCoPs = new SideDependentList<>();
   private final SideDependentList<EnumYoVariable<?>> footStates = new SideDependentList<>();

   private final double controllerDT;
   private final WalkingControllerParameters walkingControllerParameters;

   private final SideDependentList<FootSwitchInterface> stateEstimatorFootSwitches;

   private final SimulationConstructionSet scs;

   private final UpdatableHighLevelHumanoidControllerToolbox controllerToolbox;
   private final ArrayList<Updatable> updatables = new ArrayList<>();
   private final DoubleYoVariable yoTime;

   public LogDataProcessorHelper(DRCRobotModel model, SimulationConstructionSet scs, FloatingRootJointRobot sdfRobot)
   {
      this.scs = scs;
      fullRobotModel = model.createFullRobotModel();
      referenceFrames = new HumanoidReferenceFrames(fullRobotModel);
      sensorReader = new SDFPerfectSimulatedSensorReader(sdfRobot, fullRobotModel, referenceFrames);
      rawSensorMap = new LogDataRawSensorMap(fullRobotModel, scs);
      twistCalculator = new TwistCalculator(fullRobotModel.getElevatorFrame(), fullRobotModel.getElevator());

      inverseDynamicsStructure = new FullInverseDynamicsStructure(fullRobotModel.getElevator(), fullRobotModel.getPelvis(), fullRobotModel.getRootJoint());

      controllerDT = model.getControllerDT();
      this.walkingControllerParameters = model.getWalkingControllerParameters();

      ContactableBodiesFactory contactableBodiesFactory = model.getContactPointParameters().getContactableBodiesFactory();
      contactableFeet = contactableBodiesFactory.createFootContactableBodies(fullRobotModel, referenceFrames);

      for (RobotSide robotSide : RobotSide.values)
      {
         ReferenceFrame soleFrame = referenceFrames.getSoleFrame(robotSide);

         String side = robotSide.getCamelCaseNameForMiddleOfExpression();
         String bodyName = contactableFeet.get(robotSide).getName();
         String copNamePrefix = bodyName + "StateEstimator";
         String copNameSpace = copNamePrefix + WrenchBasedFootSwitch.class.getSimpleName();
         String copName = copNamePrefix + "ResolvedCoP";
         DoubleYoVariable copx = (DoubleYoVariable) scs.getVariable(copNameSpace, copName + "X");
         DoubleYoVariable copy = (DoubleYoVariable) scs.getVariable(copNameSpace, copName + "Y");
         if (copx != null && copy != null)
         {
            YoFramePoint2d cop = new YoFramePoint2d(copx, copy, soleFrame);
            cops.put(robotSide, cop);
         }

         String desiredCoPNameSpace = PlaneContactWrenchProcessor.class.getSimpleName();
         String desiredCoPName = side + "SoleCoP2d";
         DoubleYoVariable desiredCoPx = (DoubleYoVariable) scs.getVariable(desiredCoPNameSpace, desiredCoPName + "X");
         DoubleYoVariable desiredCoPy = (DoubleYoVariable) scs.getVariable(desiredCoPNameSpace, desiredCoPName + "Y");
         YoFramePoint2d desiredCoP = new YoFramePoint2d(desiredCoPx, desiredCoPy, soleFrame);
         desiredCoPs.put(robotSide, desiredCoP);

         String sidePrefix = robotSide.getCamelCaseNameForStartOfExpression();
         String namePrefix = sidePrefix + "Foot";
         String footStateNameSpace = sidePrefix + FootControlModule.class.getSimpleName();
         String footStateName = namePrefix + "State";
         @SuppressWarnings("unchecked")
         EnumYoVariable<?> footState = (EnumYoVariable<ConstraintType>) scs.getVariable(footStateNameSpace, footStateName);
         footStates.put(robotSide, footState);
      }

      stateEstimatorFootSwitches = createStateEstimatorFootSwitches(scs);

      double omega0 = walkingControllerParameters.getOmega0();
      double gravityZ = 9.81;
      GeometricJacobianHolder robotJacobianHolder = new GeometricJacobianHolder();
      String controllerTimeNamespace = DRCControllerThread.class.getSimpleName();
      yoTime = (DoubleYoVariable) scs.getVariable(controllerTimeNamespace, "controllerTime");

      controllerToolbox = new UpdatableHighLevelHumanoidControllerToolbox(scs, fullRobotModel, robotJacobianHolder, referenceFrames, stateEstimatorFootSwitches,
            null, null, yoTime, gravityZ, omega0, twistCalculator, contactableFeet, controllerDT, updatables, null);

   }

   private SideDependentList<FootSwitchInterface> createStateEstimatorFootSwitches(YoVariableHolder yoVariableHolder)
   {
      SideDependentList<FootSwitchInterface> footSwitches = new SideDependentList<FootSwitchInterface>();

      for (final RobotSide robotSide : RobotSide.values)
      {
         String namePrefix = contactableFeet.get(robotSide).getName() + "StateEstimator";
         String nameSpaceEnding = namePrefix + WrenchBasedFootSwitch.class.getSimpleName();
         final BooleanYoVariable hasFootHitGround = (BooleanYoVariable) yoVariableHolder.getVariable(nameSpaceEnding, namePrefix + "FilteredFootHitGround");
         final BooleanYoVariable forceMagnitudePastThreshhold = (BooleanYoVariable) yoVariableHolder.getVariable(nameSpaceEnding, namePrefix +  "ForcePastThresh");
         final DoubleYoVariable footLoadPercentage = (DoubleYoVariable) yoVariableHolder.getVariable(nameSpaceEnding, namePrefix + "FootLoadPercentage");

         FootSwitchInterface footSwitch = new FootSwitchInterface()
         {

            @Override
            public void reset()
            {
            }

            @Override
            public boolean hasFootHitGround()
            {
               return hasFootHitGround.getBooleanValue();
            }

            @Override
            public ReferenceFrame getMeasurementFrame()
            {
               return null;
            }

            @Override
            public boolean getForceMagnitudePastThreshhold()
            {
               return forceMagnitudePastThreshhold.getBooleanValue();
            }

            @Override
            public double computeFootLoadPercentage()
            {
               return footLoadPercentage.getDoubleValue();
            }

            @Override
            public void computeAndPackFootWrench(Wrench footWrenchToPack)
            {
            }

            @Override
            public void computeAndPackCoP(FramePoint2d copToPack)
            {
               cops.get(robotSide).getFrameTuple2dIncludingFrame(copToPack);
            }

            @Override
            public void updateCoP()
            {
            }

            @Override
            @Deprecated
            public void setFootContactState(boolean hasFootHitGround)
            {
            }

            @Override
            public void trustFootSwitch(boolean trustFootSwitch)
            {
            }
         };

         footSwitches.put(robotSide, footSwitch);
      }

      return footSwitches;
   }

   public void update()
   {
      sensorReader.read();
      twistCalculator.compute();
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

   public TwistCalculator getTwistCalculator()
   {
      return twistCalculator;
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

   public void getMeasuredCoP(RobotSide robotSide, FramePoint2d copToPack)
   {
      cops.get(robotSide).getFrameTuple2dIncludingFrame(copToPack);
   }

   public void getDesiredCoP(RobotSide robotSide, FramePoint2d desiredCoPToPack)
   {
      desiredCoPs.get(robotSide).getFrameTuple2dIncludingFrame(desiredCoPToPack);
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

   public YoFramePoint findYoFramePoint(String pointPrefix, ReferenceFrame pointFrame)
   {
      return findYoFramePoint(pointPrefix, "", pointFrame);
   }

   public YoFramePoint findYoFramePoint(String pointPrefix, String pointSuffix, ReferenceFrame pointFrame)
   {
      DoubleYoVariable x = (DoubleYoVariable) scs.getVariable(createXName(pointPrefix, pointSuffix));
      DoubleYoVariable y = (DoubleYoVariable) scs.getVariable(createYName(pointPrefix, pointSuffix));
      DoubleYoVariable z = (DoubleYoVariable) scs.getVariable(createZName(pointPrefix, pointSuffix));
      if (x == null || y == null || z == null)
         return null;
      else
         return new YoFramePoint(x, y, z, pointFrame);
   }

   public YoFrameVector findYoFrameVector(String vectorPrefix, ReferenceFrame vectorFrame)
   {
      return findYoFrameVector(vectorPrefix, "", vectorFrame);
   }

   public YoFrameVector findYoFrameVector(String vectorPrefix, String vectorSuffix, ReferenceFrame vectorFrame)
   {
      DoubleYoVariable x = (DoubleYoVariable) scs.getVariable(createXName(vectorPrefix, vectorSuffix));
      DoubleYoVariable y = (DoubleYoVariable) scs.getVariable(createYName(vectorPrefix, vectorSuffix));
      DoubleYoVariable z = (DoubleYoVariable) scs.getVariable(createZName(vectorPrefix, vectorSuffix));
      if (x == null || y == null || z == null)
         return null;
      else
         return new YoFrameVector(x, y, z, vectorFrame);
   }

   public YoFrameQuaternion findYoFrameQuaternion(String quaternionPrefix, ReferenceFrame quaternionFrame)
   {
      return findYoFrameQuaternion(quaternionPrefix, "", quaternionFrame);
   }

   public YoFrameQuaternion findYoFrameQuaternion(String quaternionPrefix, String quaternionSuffix, ReferenceFrame quaternionFrame)
   {
      DoubleYoVariable qx = (DoubleYoVariable) scs.getVariable(createQxName(quaternionPrefix, quaternionSuffix));
      DoubleYoVariable qy = (DoubleYoVariable) scs.getVariable(createQyName(quaternionPrefix, quaternionSuffix));
      DoubleYoVariable qz = (DoubleYoVariable) scs.getVariable(createQzName(quaternionPrefix, quaternionSuffix));
      DoubleYoVariable qs = (DoubleYoVariable) scs.getVariable(createQsName(quaternionPrefix, quaternionSuffix));

      if (qx == null || qy == null || qz == null || qs == null)
         return null;
      else
         return new YoFrameQuaternion(qx, qy, qz, qs, quaternionFrame);
   }
}
