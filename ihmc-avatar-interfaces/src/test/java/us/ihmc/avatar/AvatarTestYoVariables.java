package us.ihmc.avatar;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

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
      yoTime = (YoDouble) scs.findVariable("t");

      pelvisX = (YoDouble) scs.findVariable("q_x");
      pelvisY = (YoDouble) scs.findVariable("q_y");
      pelvisZ = (YoDouble) scs.findVariable("q_z");
      pelvisYaw = (YoDouble) scs.findVariable("q_yaw");

      midFeetZUpZ = (YoDouble) scs.findVariable("midFeetZUpZ");

      if (scs.findVariable("desiredCOMHeight") == null)
      {
         // We might be controlling the desired pelvis height in this case.
         desiredCOMHeight = (YoDouble) scs.findVariable("pelvisDesiredPositionZ");
      }
      else
      {
         desiredCOMHeight = (YoDouble) scs.findVariable("desiredCOMHeight");
      }

      solePositions.set(RobotSide.LEFT, new YoFramePoint3D((YoDouble) scs.findVariable("leftSoleX"), (YoDouble) scs.findVariable("leftSoleY"),
                                                         (YoDouble) scs.findVariable("leftSoleZ"), ReferenceFrame.getWorldFrame()));
      solePositions.set(RobotSide.RIGHT, new YoFramePoint3D((YoDouble) scs.findVariable("rightSoleX"), (YoDouble) scs.findVariable("rightSoleY"),
                                                         (YoDouble) scs.findVariable("rightSoleZ"), ReferenceFrame.getWorldFrame()));

      angularMomentumX = (YoDouble) scs.findVariable("AngularMomentumX");
      angularMomentumY = (YoDouble) scs.findVariable("AngularMomentumY");
      angularMomentumZ = (YoDouble) scs.findVariable("AngularMomentumZ");

      icpPlannerDesiredCentroidalAngularMomentumX = (YoDouble) scs.findVariable("icpPlannerDesiredCentroidalAngularMomentumX");
      icpPlannerDesiredCentroidalAngularMomentumY = (YoDouble) scs.findVariable("icpPlannerDesiredCentroidalAngularMomentumY");
      icpPlannerDesiredCentroidalAngularMomentumZ = (YoDouble) scs.findVariable("icpPlannerDesiredCentroidalAngularMomentumZ");

      controllerIsInDoubleSupport = (YoBoolean) scs.findVariable("controllerIsInDoubleSupport");
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
