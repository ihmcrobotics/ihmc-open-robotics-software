package us.ihmc.avatar;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoFramePoint3D;

public abstract class AvatarTestYoVariables
{
   private final YoDouble yoTime;

   private final YoDouble pelvisX;
   private final YoDouble pelvisY;
   private final YoDouble pelvisZ;
   private final YoDouble pelvisYaw;

   private final YoDouble midFeetZUpZ;
   private final YoDouble desiredCOMHeight;

   private final YoDouble angularMomentumX;
   private final YoDouble angularMomentumY;
   private final YoDouble angularMomentumZ;

   private final YoDouble icpPlannerDesiredCentroidalAngularMomentumX;
   private final YoDouble icpPlannerDesiredCentroidalAngularMomentumY;
   private final YoDouble icpPlannerDesiredCentroidalAngularMomentumZ;

   private final SideDependentList<YoFramePoint3D> solePositions = new SideDependentList<>();

   private final YoBoolean controllerIsInDoubleSupport;

   public AvatarTestYoVariables(SimulationConstructionSet scs)
   {
      yoTime = (YoDouble) scs.getVariable("t");

      pelvisX = (YoDouble) scs.getVariable("q_x");
      pelvisY = (YoDouble) scs.getVariable("q_y");
      pelvisZ = (YoDouble) scs.getVariable("q_z");
      pelvisYaw = (YoDouble) scs.getVariable("q_yaw");

      midFeetZUpZ = (YoDouble) scs.getVariable("midFeetZUpZ");

      if (scs.getVariable("desiredCOMHeight") == null)
      {
         // We might be controlling the desired pelvis height in this case.
         desiredCOMHeight = (YoDouble) scs.getVariable("pelvisDesiredPositionZ");
      }
      else
      {
         desiredCOMHeight = (YoDouble) scs.getVariable("desiredCOMHeight");
      }

      solePositions.set(RobotSide.LEFT, new YoFramePoint3D((YoDouble) scs.getVariable("leftSoleX"), (YoDouble) scs.getVariable("leftSoleY"),
                                                         (YoDouble) scs.getVariable("leftSoleZ"), ReferenceFrame.getWorldFrame()));
      solePositions.set(RobotSide.RIGHT, new YoFramePoint3D((YoDouble) scs.getVariable("rightSoleX"), (YoDouble) scs.getVariable("rightSoleY"),
                                                         (YoDouble) scs.getVariable("rightSoleZ"), ReferenceFrame.getWorldFrame()));

      angularMomentumX = (YoDouble) scs.getVariable("AngularMomentumX");
      angularMomentumY = (YoDouble) scs.getVariable("AngularMomentumY");
      angularMomentumZ = (YoDouble) scs.getVariable("AngularMomentumZ");

      icpPlannerDesiredCentroidalAngularMomentumX = (YoDouble) scs.getVariable("icpPlannerDesiredCentroidalAngularMomentumX");
      icpPlannerDesiredCentroidalAngularMomentumY = (YoDouble) scs.getVariable("icpPlannerDesiredCentroidalAngularMomentumY");
      icpPlannerDesiredCentroidalAngularMomentumZ = (YoDouble) scs.getVariable("icpPlannerDesiredCentroidalAngularMomentumZ");

      controllerIsInDoubleSupport = (YoBoolean) scs.getVariable("controllerIsInDoubleSupport");
   }

   public YoDouble getYoTime()
   {
      return yoTime;
   }

   public YoDouble getPelvisX()
   {
      return pelvisX;
   }

   public YoDouble getPelvisY()
   {
      return pelvisY;
   }

   public YoDouble getPelvisZ()
   {
      return pelvisZ;
   }

   public YoDouble getPelvisYaw()
   {
      return pelvisYaw;
   }

   public YoDouble getMidFeetZUpZ()
   {
      return midFeetZUpZ;
   }

   public YoDouble getDesiredCOMHeight()
   {
      return desiredCOMHeight;
   }

   public YoFramePoint3D getSolePosition(RobotSide robotSide)
   {
      return solePositions.get(robotSide);
   }

   public YoDouble getAngularMomentumX()
   {
      return angularMomentumX;
   }

   public YoDouble getAngularMomentumY()
   {
      return angularMomentumY;
   }

   public YoDouble getAngularMomentumZ()
   {
      return angularMomentumZ;
   }

   public YoDouble getIcpPlannerDesiredCentroidalAngularMomentumX()
   {
      return icpPlannerDesiredCentroidalAngularMomentumX;
   }

   public YoDouble getIcpPlannerDesiredCentroidalAngularMomentumY()
   {
      return icpPlannerDesiredCentroidalAngularMomentumY;
   }

   public YoDouble getIcpPlannerDesiredCentroidalAngularMomentumZ()
   {
      return icpPlannerDesiredCentroidalAngularMomentumZ;
   }

   public YoBoolean getControllerIsInDoubleSupport()
   {
      return controllerIsInDoubleSupport;
   }
}
