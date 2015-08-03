package us.ihmc.darpaRoboticsChallenge.logProcessor;

import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FootControlModule.ConstraintType;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ContactableBodiesFactory;
import us.ihmc.commonWalkingControlModules.momentumBasedController.PlaneContactWrenchProcessor;
import us.ihmc.commonWalkingControlModules.sensors.footSwitch.FootSwitchInterface;
import us.ihmc.commonWalkingControlModules.sensors.footSwitch.WrenchBasedFootSwitch;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.sensorProcessing.simulatedSensors.SDFPerfectSimulatedSensorReader;
import us.ihmc.sensorProcessing.stateEstimation.evaluation.FullInverseDynamicsStructure;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.robotics.humanoidRobot.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.robotics.humanoidRobot.frames.ReferenceFrames;
import us.ihmc.robotics.humanoidRobot.model.FullRobotModel;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.TwistCalculator;
import us.ihmc.robotics.screwTheory.Wrench;
import us.ihmc.yoUtilities.dataStructure.YoVariableHolder;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.EnumYoVariable;
import us.ihmc.yoUtilities.math.frames.YoFramePoint2d;

public class LogDataProcessorHelper
{
   private final FullRobotModel fullRobotModel;
   private final FullInverseDynamicsStructure inverseDynamicsStructure;
   private final ReferenceFrames referenceFrames;
   private final SDFPerfectSimulatedSensorReader sensorReader;
   private final LogDataRawSensorMap rawSensorMap;
   private final TwistCalculator twistCalculator;
   private final SideDependentList<ContactablePlaneBody> contactableFeet;

   private final SideDependentList<YoFramePoint2d> cops = new SideDependentList<>();
   private final SideDependentList<YoFramePoint2d> desiredCoPs = new SideDependentList<>();
   private final SideDependentList<EnumYoVariable<?>> footStates = new SideDependentList<>();

   private final double controllerDT;
   private final WalkingControllerParameters walkingControllerParameters;

   private final SideDependentList<FootSwitchInterface> stateEstimatorFootSwitches;
   
   private final SimulationConstructionSet scs;

   public LogDataProcessorHelper(DRCRobotModel model, SimulationConstructionSet scs, SDFRobot sdfRobot)
   {
      this.scs = scs;
      fullRobotModel = model.createFullRobotModel();
      referenceFrames = new ReferenceFrames(fullRobotModel);
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
         YoFramePoint2d cop = new YoFramePoint2d(copx, copy, soleFrame);
         cops.put(robotSide, cop);

         String desiredCoPNameSpace = PlaneContactWrenchProcessor.class.getSimpleName();
         String desiredCoPName = side + "SoleCoP2d";
         DoubleYoVariable desiredCoPx = (DoubleYoVariable) scs.getVariable(desiredCoPNameSpace, desiredCoPName + "X");
         DoubleYoVariable desiredCoPy = (DoubleYoVariable) scs.getVariable(desiredCoPNameSpace, desiredCoPName + "Y");
         YoFramePoint2d desiredCoP = new YoFramePoint2d(desiredCoPx, desiredCoPy, soleFrame);
         desiredCoPs.put(robotSide, desiredCoP);

         String namePrefix = robotSide.getShortLowerCaseName() + "_foot";
         String footStateNameSpace = "DRCControllerThread.DRCMomentumBasedController.HighLevelHumanoidControllerManager.MomentumBasedControllerFactory.FeetManager."
               + namePrefix + "FootControlModule";
         String footStateName = namePrefix + "State";
         @SuppressWarnings("unchecked")
         EnumYoVariable<?> footState = (EnumYoVariable<ConstraintType>) scs.getVariable(footStateNameSpace, footStateName);
         footStates.put(robotSide, footState);
      }

      stateEstimatorFootSwitches = createStateEstimatorFootSwitches(scs);
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
         };
         
         footSwitches.put(robotSide, footSwitch);
      }
      
      return footSwitches;
   }

   public void update()
   {
      sensorReader.read();
      twistCalculator.compute();
   }

   public FullRobotModel getFullRobotModel()
   {
      return fullRobotModel;
   }

   public FullInverseDynamicsStructure getInverseDynamicsStructure()
   {
      return inverseDynamicsStructure;
   }

   public ReferenceFrames getReferenceFrames()
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

   public SideDependentList<ContactablePlaneBody> getContactableFeet()
   {
      return contactableFeet;
   }

   public SideDependentList<FootSwitchInterface> getStateEstimatorFootSwitches()
   {
      return stateEstimatorFootSwitches;
   }

   public ConstraintType getCurrenFootState(RobotSide robotSide)
   {
      return ConstraintType.valueOf(footStates.get(robotSide).getEnumValue().toString());
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
}
