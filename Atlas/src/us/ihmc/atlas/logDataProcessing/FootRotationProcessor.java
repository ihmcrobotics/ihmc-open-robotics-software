package us.ihmc.atlas.logDataProcessing;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FootControlModule.ConstraintType;
import us.ihmc.commonWalkingControlModules.controlModules.foot.PartialFootholdControlModule;
import us.ihmc.commonWalkingControlModules.momentumBasedController.PlaneContactWrenchProcessor;
import us.ihmc.commonWalkingControlModules.sensors.WrenchBasedFootSwitch;
import us.ihmc.darpaRoboticsChallenge.DRCControllerThread;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.utilities.humanoidRobot.frames.ReferenceFrames;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.utilities.robotSide.SideDependentList;
import us.ihmc.utilities.screwTheory.TwistCalculator;
import us.ihmc.yoUtilities.dataStructure.listener.VariableChangedListener;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.EnumYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.LongYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.YoVariable;
import us.ihmc.yoUtilities.graphics.YoGraphicsListRegistry;
import us.ihmc.yoUtilities.humanoidRobot.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.yoUtilities.math.frames.YoFramePoint2d;

import com.yobotics.simulationconstructionset.DataProcessingFunction;
import com.yobotics.simulationconstructionset.SimulationConstructionSet;

public class FootRotationProcessor implements DataProcessingFunction
{
   private final String name = getClass().getSimpleName();
   private final YoVariableRegistry registry = new YoVariableRegistry(name);
   private final YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();
   private final FullRobotModelUpdater fullRobotModelUpdater;
   private final TwistCalculator twistCalculator;
   private final SideDependentList<YoFramePoint2d> cops = new SideDependentList<>();
   private final SideDependentList<YoFramePoint2d> desiredCoPs = new SideDependentList<>();
   private final SideDependentList<PartialFootholdControlModule> partialFootholdControlModules = new SideDependentList<>();
   private final SideDependentList<ReferenceFrame> soleFrames = new SideDependentList<>();
   private final SideDependentList<EnumYoVariable<?>> footStates = new SideDependentList<>();
   private final LongYoVariable controllerTimerCount;
   private boolean nextControllerTick = true;

   private final double[] saveVarAsDoubles;
   
   public FootRotationProcessor(DRCRobotModel model, SimulationConstructionSet scs)
   {
      SDFFullRobotModel fullRobotModel = model.createFullRobotModel();
      ReferenceFrames referenceFrames = new ReferenceFrames(fullRobotModel);
      fullRobotModelUpdater = new FullRobotModelUpdater(scs.getRobots()[0], fullRobotModel, referenceFrames);
      SideDependentList<ContactablePlaneBody> contactableFeet = model.getContactPointParameters().getContactableBodiesFactory()
            .createFootContactableBodies(fullRobotModel, referenceFrames);
      twistCalculator = new TwistCalculator(fullRobotModel.getElevatorFrame(), fullRobotModel.getElevator());

      for (RobotSide robotSide : RobotSide.values)
      {
         String namePrefix = robotSide.getCamelCaseNameForStartOfExpression() + "Foot";
         PartialFootholdControlModule partialFootholdControlModule = new PartialFootholdControlModule(namePrefix, model.getControllerDT(), contactableFeet.get(robotSide), twistCalculator, registry, yoGraphicsListRegistry);
         partialFootholdControlModules.put(robotSide, partialFootholdControlModule);

         ReferenceFrame soleFrame = contactableFeet.get(robotSide).getSoleFrame();
         soleFrames.put(robotSide, soleFrame);
      }

      for (RobotSide robotSide : RobotSide.values)
      {
         String side = robotSide.getCamelCaseNameForMiddleOfExpression();
         String bodyName = contactableFeet.get(robotSide).getName();
         String copNamePrefix = bodyName + "StateEstimator";
         String copNameSpace = copNamePrefix + WrenchBasedFootSwitch.class.getSimpleName();
         String copName = copNamePrefix + "ResolvedCoP";
         DoubleYoVariable copx = (DoubleYoVariable) scs.getVariable(copNameSpace, copName + "X");
         DoubleYoVariable copy = (DoubleYoVariable) scs.getVariable(copNameSpace, copName + "Y");
         YoFramePoint2d cop = new YoFramePoint2d(copx, copy, soleFrames.get(robotSide));
         cops.put(robotSide, cop);
         
         String desiredCoPNameSpace = PlaneContactWrenchProcessor.class.getSimpleName();
         String desiredCoPName = side + "SoleCoP2d";
         DoubleYoVariable desiredCoPx = (DoubleYoVariable) scs.getVariable(desiredCoPNameSpace, desiredCoPName + "X");
         DoubleYoVariable desiredCoPy = (DoubleYoVariable) scs.getVariable(desiredCoPNameSpace, desiredCoPName + "Y");
         YoFramePoint2d desiredCoP = new YoFramePoint2d(desiredCoPx, desiredCoPy, soleFrames.get(robotSide));
         desiredCoPs.put(robotSide, desiredCoP);
         
         String namePrefix = robotSide.getShortLowerCaseName() + "_foot";
         String footStateNameSpace = "DRCControllerThread.DRCMomentumBasedController.HighLevelHumanoidControllerManager.MomentumBasedControllerFactory.FeetManager." + namePrefix + "FootControlModule";
         String footStateName = namePrefix + "State";
         EnumYoVariable<?> footState = (EnumYoVariable<ConstraintType>) scs.getVariable(footStateNameSpace, footStateName);
         footStates.put(robotSide, footState);
      }

      controllerTimerCount = (LongYoVariable) scs.getVariable(DRCControllerThread.class.getSimpleName(), "controllerTimerCount");
      controllerTimerCount.addVariableChangedListener(new VariableChangedListener()
      {
         @Override
         public void variableChanged(YoVariable<?> v)
         {
            nextControllerTick = true;
         }
      });
      
      scs.addYoGraphicsListRegistry(yoGraphicsListRegistry, true);
      scs.addYoVariableRegistry(registry);
      
      saveVarAsDoubles = new double[registry.getAllVariablesArray().length];
   }


   @Override
   public void initializeProcessing()
   {
      for (int i = 0; i < saveVarAsDoubles.length; i++)
      {
         saveVarAsDoubles[i] = registry.getAllVariablesArray()[i].getValueAsDouble();
      }
   }
   
   private final FramePoint2d tempCoP2d = new FramePoint2d();
   private final FramePoint2d tempDesiredCoP2d = new FramePoint2d();
   private final FramePoint tempCoP = new FramePoint();
   
   @Override
   public void processData()
   {
      for (int i = 0; i < saveVarAsDoubles.length; i++)
      {
         registry.getAllVariablesArray()[i].setValueFromDouble(saveVarAsDoubles[i], false);
      }

      if (!nextControllerTick)
      {
         return;
      }

      nextControllerTick = false;
      
      fullRobotModelUpdater.read();
      twistCalculator.compute();

      for (RobotSide robotSide : RobotSide.values)
      {
         if (footStates.get(robotSide).getEnumValue().toString().equals(ConstraintType.FULL.toString()))
         {
            cops.get(robotSide).getFrameTuple2dIncludingFrame(tempCoP2d);
            desiredCoPs.get(robotSide).getFrameTuple2dIncludingFrame(tempDesiredCoP2d);
            partialFootholdControlModules.get(robotSide).compute(tempDesiredCoP2d, tempCoP2d);
         }
         else
         {
            partialFootholdControlModules.get(robotSide).reset();
         }
      }

      for (int i = 0; i < saveVarAsDoubles.length; i++)
      {
         saveVarAsDoubles[i] = registry.getAllVariablesArray()[i].getValueAsDouble();
      }
   }

   public YoVariableRegistry getRegistry()
   {
      return registry;
   }

   public YoGraphicsListRegistry getYoGraphicsListRegistry()
   {
      return yoGraphicsListRegistry;
   }

}
