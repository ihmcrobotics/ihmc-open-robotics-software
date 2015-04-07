package us.ihmc.commonWalkingControlModules.controlModules;

import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;
import us.ihmc.commonWalkingControlModules.packetConsumers.PelvisPoseProvider;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.FrameVector2d;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.math.trajectories.providers.DoubleProvider;
import us.ihmc.utilities.math.trajectories.providers.PositionProvider;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.utilities.robotSide.SideDependentList;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.Twist;
import us.ihmc.utilities.screwTheory.TwistCalculator;
import us.ihmc.yoUtilities.dataStructure.listener.VariableChangedListener;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.YoVariable;
import us.ihmc.yoUtilities.math.frames.YoFramePoint;
import us.ihmc.yoUtilities.math.frames.YoFramePoint2d;
import us.ihmc.yoUtilities.math.frames.YoFrameVector2d;
import us.ihmc.yoUtilities.math.trajectories.MultipleWaypointsPositionTrajectoryGenerator;
import us.ihmc.yoUtilities.math.trajectories.PositionTrajectoryGenerator;
import us.ihmc.yoUtilities.math.trajectories.StraightLinePositionTrajectoryGenerator;
import us.ihmc.yoUtilities.math.trajectories.WaypointPositionTrajectoryData;
import us.ihmc.yoUtilities.math.trajectories.providers.YoPositionProvider;
import us.ihmc.yoUtilities.math.trajectories.providers.YoVariableDoubleProvider;

public class PelvisICPBasedTranslationManager
{
   private static final double minTrajectoryTime = 0.1;

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final YoFramePoint2d desiredPelvisPosition = new YoFramePoint2d("desiredPelvis", worldFrame, registry);

   private final DoubleYoVariable initialPelvisPositionTime = new DoubleYoVariable("initialPelvisPositionTime", registry);
   private final DoubleYoVariable pelvisPositionTrajectoryTime = new DoubleYoVariable("pelvisPositionTrajectoryTime", registry);
   private final YoFramePoint initialPelvisPosition = new YoFramePoint("initialPelvis", worldFrame, registry);
   private final YoFramePoint finalPelvisPosition = new YoFramePoint("finalPelvis", worldFrame, registry);

   private final BooleanYoVariable isUsingWaypointTrajectory;
   private PositionTrajectoryGenerator activeTrajectoryGenerator;
   private final StraightLinePositionTrajectoryGenerator pelvisPositionTrajectoryGenerator;
   private final MultipleWaypointsPositionTrajectoryGenerator pelvisWaypointsPositionTrajectoryGenerator;

   private final YoFrameVector2d pelvisPositionError = new YoFrameVector2d("pelvisPositionError", worldFrame, registry);
   private final DoubleYoVariable proportionalGain = new DoubleYoVariable("pelvisPositionProportionalGain", registry);
   private final YoFrameVector2d proportionalTerm = new YoFrameVector2d("pelvisPositionProportionalTerm", worldFrame, registry);

   private final DoubleYoVariable dampingGain = new DoubleYoVariable("pelvisPositionDampingGain", registry);
   private final YoFrameVector2d dampingTerm = new YoFrameVector2d("pelvisPositionDampingTerm", worldFrame, registry);

   private final YoFrameVector2d desiredICPOffset = new YoFrameVector2d("desiredICPOffset", worldFrame, registry);
   private final YoFramePoint2d yoDesiredICPForPelvisXYPositionControl = new YoFramePoint2d("desiredICPForPelvisXYPositionControl", worldFrame, registry);

   private final BooleanYoVariable isEnabled = new BooleanYoVariable("isPelvisTranslationManagerEnabled", registry);
   private final BooleanYoVariable isRunning = new BooleanYoVariable("isPelvisTranslationManagerRunning", registry);

   private final BooleanYoVariable manualMode = new BooleanYoVariable("manualModeICPOffset", registry);

   private final DoubleYoVariable yoTime;

   private final PelvisPoseProvider desiredPelvisPoseProvider;

   private ReferenceFrame supportFrame;
   private final ReferenceFrame pelvisZUpFrame;
   private final ReferenceFrame pelvisFrame;
   private final ReferenceFrame midFeetZUpFrame;
   private final SideDependentList<ReferenceFrame> ankleZUpFrames;

   private final FramePoint tempPosition = new FramePoint();
   private final FrameVector tempVelocity = new FrameVector();
   private final FrameVector tempAcceleration = new FrameVector();
   private final FrameVector2d tempICPOffset = new FrameVector2d();
   private final Twist tempPelvisTwist = new Twist();

   private final FramePoint2d pelvisPosition2d = new FramePoint2d();
   private final FrameVector pelvisVelocity = new FrameVector();
   private final FrameVector2d pelvisVelocity2d = new FrameVector2d();
   private final FrameVector desiredPelvisVelocity = new FrameVector();
   private final FrameVector2d desiredPelvisVelocity2d = new FrameVector2d();
   
   private final FramePoint2d desiredICPForPelvisXYPositionControl = new FramePoint2d();

   private final RigidBody pelvis;

   private final TwistCalculator twistCalculator;

   public PelvisICPBasedTranslationManager(MomentumBasedController momentumBasedController, PelvisPoseProvider desiredPelvisPoseProvider,
         YoVariableRegistry parentRegistry)
   {
      yoTime = momentumBasedController.getYoTime();
      pelvisZUpFrame = momentumBasedController.getPelvisZUpFrame();
      midFeetZUpFrame = momentumBasedController.getReferenceFrames().getMidFeetZUpFrame();
      ankleZUpFrames = momentumBasedController.getReferenceFrames().getAnkleZUpReferenceFrames();
      pelvisFrame = momentumBasedController.getFullRobotModel().getRootJoint().getFrameAfterJoint();

      this.desiredPelvisPoseProvider = desiredPelvisPoseProvider;
      this.twistCalculator = momentumBasedController.getTwistCalculator();
      this.pelvis = momentumBasedController.getFullRobotModel().getPelvis();

      isUsingWaypointTrajectory = new BooleanYoVariable(getClass().getSimpleName() + "IsUsingWaypointTrajectory", registry);
      isUsingWaypointTrajectory.set(false);

      DoubleProvider trajectoryTimeProvider = new YoVariableDoubleProvider(pelvisPositionTrajectoryTime);
      PositionProvider initialPositionProvider = new YoPositionProvider(initialPelvisPosition);
      PositionProvider finalPositionProvider = new YoPositionProvider(finalPelvisPosition);
      pelvisPositionTrajectoryGenerator = new StraightLinePositionTrajectoryGenerator("pelvis", worldFrame, trajectoryTimeProvider, initialPositionProvider,
            finalPositionProvider, registry);
      pelvisPositionTrajectoryGenerator.initialize();
      activeTrajectoryGenerator = pelvisPositionTrajectoryGenerator;

      pelvisWaypointsPositionTrajectoryGenerator = new MultipleWaypointsPositionTrajectoryGenerator("pelvisWaypoints", worldFrame, initialPositionProvider, registry);

      proportionalGain.set(4.0);
      dampingGain.set(1.2);

      manualMode.addVariableChangedListener(new VariableChangedListener()
      {
         @Override
         public void variableChanged(YoVariable<?> v)
         {
            initialize();
         }
      });

      parentRegistry.addChild(registry);
   }

   public void compute(RobotSide supportLeg, FramePoint2d actualICP)
   {
      if (isUsingWaypointTrajectory != null)
      {
         if (isUsingWaypointTrajectory.getBooleanValue())
            activeTrajectoryGenerator = pelvisWaypointsPositionTrajectoryGenerator;
         else
            activeTrajectoryGenerator = pelvisPositionTrajectoryGenerator;
      }

      supportFrame = supportLeg == null ? midFeetZUpFrame : ankleZUpFrames.get(supportLeg);

      if (!isEnabled.getBooleanValue())
      {
         desiredICPOffset.setToZero();
         yoDesiredICPForPelvisXYPositionControl.setToZero();
         return;
      }

      if (manualMode.getBooleanValue())
         return;

      updateDesireds();

      if (!isRunning.getBooleanValue())
      {
         desiredICPOffset.setToZero();
         yoDesiredICPForPelvisXYPositionControl.setToZero();
         return;
      }

      computeDesiredICPForPelvisXYPositionControl(actualICP);
   }

   private void updateDesireds()
   {
      if (desiredPelvisPoseProvider != null)
      {
         if (desiredPelvisPoseProvider.checkForHomePosition())
         {
            disable();
            enable();
         }
         else if (desiredPelvisPoseProvider.checkForNewPosition())
         {
            initialPelvisPositionTime.set(yoTime.getDoubleValue());
            if (desiredPelvisPoseProvider.getTrajectoryTime() < minTrajectoryTime)
               pelvisPositionTrajectoryTime.set(minTrajectoryTime);
            else
               pelvisPositionTrajectoryTime.set(desiredPelvisPoseProvider.getTrajectoryTime());
            tempPosition.setToZero(pelvisZUpFrame);
            initialPelvisPosition.setAndMatchFrame(tempPosition);
            finalPelvisPosition.setAndMatchFrame(desiredPelvisPoseProvider.getDesiredPelvisPosition(supportFrame));
            pelvisPositionTrajectoryGenerator.initialize();
            isUsingWaypointTrajectory.set(false);
            activeTrajectoryGenerator = pelvisPositionTrajectoryGenerator;
            isRunning.set(true);
         }
         else if (desiredPelvisPoseProvider.checkForNewPositionWithWaypoints())
         {
            initialPelvisPositionTime.set(yoTime.getDoubleValue());
            pelvisWaypointsPositionTrajectoryGenerator.clear();     
            
            tempPosition.setToZero(pelvisZUpFrame);
            tempPosition.changeFrame(worldFrame);
            tempVelocity.setToZero(worldFrame);     
            
            WaypointPositionTrajectoryData desiredPelvisPositionWithWaypoints = desiredPelvisPoseProvider.getDesiredPelvisPositionWithWaypoints();
            desiredPelvisPositionWithWaypoints.changeFrame(worldFrame);
            pelvisWaypointsPositionTrajectoryGenerator.appendWaypoints(desiredPelvisPositionWithWaypoints);
           
            pelvisWaypointsPositionTrajectoryGenerator.initialize( tempPosition, tempVelocity );
           
            isUsingWaypointTrajectory.set(true);
            activeTrajectoryGenerator = pelvisWaypointsPositionTrajectoryGenerator;
            isRunning.set(true);
         }
      }

      if (isRunning.getBooleanValue())
      {
         double deltaTime = yoTime.getDoubleValue() - initialPelvisPositionTime.getDoubleValue();
         activeTrajectoryGenerator.compute(deltaTime);
         activeTrajectoryGenerator.get(tempPosition);
         activeTrajectoryGenerator.packVelocity(desiredPelvisVelocity);
         activeTrajectoryGenerator.packAcceleration(tempAcceleration);

         desiredPelvisPosition.setByProjectionOntoXYPlane(tempPosition);
      }
   }

   public void computeDesiredICPForPelvisXYPositionControl(FramePoint2d actualICP)
   {
      desiredICPForPelvisXYPositionControl.setIncludingFrame(actualICP);

      pelvisPositionError.set(desiredPelvisPosition);
      pelvisPosition2d.setToZero(pelvisZUpFrame);
      pelvisPosition2d.changeFrame(worldFrame);
      pelvisPositionError.sub(pelvisPosition2d);

      proportionalTerm.set(pelvisPositionError);
      proportionalTerm.scale(proportionalGain.getDoubleValue());
      proportionalTerm.getFrameTuple2dIncludingFrame(tempICPOffset);
      desiredICPForPelvisXYPositionControl.add(tempICPOffset);

      twistCalculator.packTwistOfBody(tempPelvisTwist, pelvis);
      tempPelvisTwist.changeFrame(pelvisFrame);
      tempPelvisTwist.packLinearPart(pelvisVelocity);
      pelvisVelocity.changeFrame(worldFrame);
      pelvisVelocity2d.setByProjectionOntoXYPlaneIncludingFrame(pelvisVelocity);

      dampingTerm.set(pelvisVelocity2d);
      dampingTerm.scale(- dampingGain.getDoubleValue());
      dampingTerm.getFrameTuple2d(tempICPOffset);
      desiredICPForPelvisXYPositionControl.add(tempICPOffset);

      desiredPelvisVelocity2d.setByProjectionOntoXYPlaneIncludingFrame(desiredPelvisVelocity);
      desiredICPForPelvisXYPositionControl.add(desiredPelvisVelocity2d);
      yoDesiredICPForPelvisXYPositionControl.setAndMatchFrame(desiredICPForPelvisXYPositionControl);
   }

   public void addICPOffset(FramePoint2d desiredICPToModify, FrameVector2d desiredICPVelocityToModify)
   {
      if (!isEnabled.getBooleanValue())
      {
         desiredICPOffset.setToZero();
         yoDesiredICPForPelvisXYPositionControl.setToZero();
         return;
      }

      if (manualMode.getBooleanValue())
      {
         // Ignore the desiredICPOffset frame assuming the user wants to control the ICP in the supportFrame
         tempICPOffset.setIncludingFrame(supportFrame, desiredICPOffset.getX(), desiredICPOffset.getY());
         desiredICPToModify.add(tempICPOffset);
         return;
      }
      else if (!isRunning.getBooleanValue())
      {
         desiredICPOffset.setToZero();
         yoDesiredICPForPelvisXYPositionControl.setToZero();
         return;
      }
      else
      {
         desiredICPForPelvisXYPositionControl.changeFrame(supportFrame);
         desiredICPToModify.set(desiredICPForPelvisXYPositionControl);

         desiredPelvisVelocity.changeFrame(supportFrame);
         desiredPelvisVelocity2d.setByProjectionOntoXYPlaneIncludingFrame(desiredPelvisVelocity);
         desiredICPVelocityToModify.set(desiredPelvisVelocity2d);
      }
   }

   public void disable()
   {
      isEnabled.set(false);
      isRunning.set(false);

      pelvisPositionError.setToZero();

      proportionalTerm.setToZero();
      dampingTerm.setToZero();

      desiredICPOffset.setToZero();
      yoDesiredICPForPelvisXYPositionControl.setToZero();
   }

   public void enable()
   {
      if (isEnabled.getBooleanValue())
         return;
      isEnabled.set(true);
      initialize();
   }

   private void initialize()
   {
      initialPelvisPositionTime.set(yoTime.getDoubleValue());
      pelvisPositionTrajectoryTime.set(0.0);
      tempPosition.setToZero(pelvisZUpFrame);
      initialPelvisPosition.setAndMatchFrame(tempPosition);
      finalPelvisPosition.setAndMatchFrame(tempPosition);
      pelvisPositionTrajectoryGenerator.initialize();
      activeTrajectoryGenerator = pelvisPositionTrajectoryGenerator;
   }
}
