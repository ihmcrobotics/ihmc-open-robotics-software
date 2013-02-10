package us.ihmc.commonWalkingControlModules.desiredFootStep;

import java.util.List;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.robotSide.SideDependentList;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.RigidBody;

import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.math.frames.YoFrameOrientation;
import com.yobotics.simulationconstructionset.util.math.frames.YoFramePoint;

public abstract class AbstractAdjustableDesiredFootstepCalculator implements DesiredFootstepCalculator
{
   protected final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   protected final SideDependentList<YoFramePoint> footstepPositions = new SideDependentList<YoFramePoint>();
   protected final SideDependentList<YoFrameOrientation> footstepOrientations = new SideDependentList<YoFrameOrientation>();
   protected final SideDependentList<? extends ContactablePlaneBody> contactableBodies;

   private DesiredFootstepAdjustor desiredFootstepAdjustor;

   public AbstractAdjustableDesiredFootstepCalculator(SideDependentList<? extends ContactablePlaneBody> contactableBodies,
           SideDependentList<ReferenceFrame> framesToSaveFootstepIn, YoVariableRegistry parentRegistry)
   {
      this.contactableBodies = contactableBodies;

      for (RobotSide robotSide : RobotSide.values())
      {
         String namePrefix = robotSide.getCamelCaseNameForMiddleOfExpression() + "Footstep";

         ReferenceFrame frameToSaveFootstepIn = framesToSaveFootstepIn.get(robotSide);
         YoFramePoint footstepPosition = new YoFramePoint(namePrefix + "Position", frameToSaveFootstepIn, registry);
         footstepPositions.put(robotSide, footstepPosition);

         YoFrameOrientation footstepOrientation = new YoFrameOrientation(namePrefix + "Orientation", "", frameToSaveFootstepIn, registry);
         footstepOrientations.put(robotSide, footstepOrientation);
      }

      parentRegistry.addChild(registry);
   }

   public Footstep updateAndGetDesiredFootstep(RobotSide supportLegSide)
   {
      RobotSide swingLegSide = supportLegSide.getOppositeSide();

      FramePose footstepPose = new FramePose(footstepPositions.get(swingLegSide).getFramePointCopy(),
                                  footstepOrientations.get(swingLegSide).getFrameOrientationCopy());
      ContactablePlaneBody foot = contactableBodies.get(swingLegSide);
      
      boolean trustHeight = false;
      Footstep desiredFootstep = new Footstep(foot.getRigidBody(), footstepPose, getContactPoints(swingLegSide), trustHeight);

      if (desiredFootstepAdjustor != null)
      {
         ContactablePlaneBody stanceFoot = contactableBodies.get(supportLegSide);
         RigidBody stanceFootBody = stanceFoot.getRigidBody();
         FramePose stanceFootPose = new FramePose(stanceFootBody.getBodyFixedFrame());
         Footstep stanceFootstep = new Footstep(stanceFootBody, stanceFootPose, stanceFoot.getContactPoints(), trustHeight);
         desiredFootstep = desiredFootstepAdjustor.adjustDesiredFootstep(stanceFootstep, desiredFootstep);

         desiredFootstep.getPose(footstepPose);
         footstepPositions.get(swingLegSide).set(footstepPose.getPosition());
         footstepOrientations.get(swingLegSide).set(footstepPose.getOrientation());
      }

      return desiredFootstep;
   }

   public void setDesiredFootstepAdjustor(DesiredFootstepAdjustor desiredFootstepAdjustor)
   {
      this.desiredFootstepAdjustor = desiredFootstepAdjustor;
   }

   protected abstract List<FramePoint> getContactPoints(RobotSide swingSide);


}
