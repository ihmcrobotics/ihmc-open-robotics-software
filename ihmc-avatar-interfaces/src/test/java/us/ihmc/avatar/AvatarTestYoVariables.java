package us.ihmc.avatar;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.dataBuffer.YoVariableHolder;
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

   public AvatarTestYoVariables(YoVariableHolder yoVariableHolder)
   {
      yoTime = (YoDouble) yoVariableHolder.getVariable("t");

      pelvisX = (YoDouble) yoVariableHolder.getVariable("q_x");
      pelvisY = (YoDouble) yoVariableHolder.getVariable("q_y");
      pelvisZ = (YoDouble) yoVariableHolder.getVariable("q_z");
      pelvisYaw = (YoDouble) yoVariableHolder.getVariable("q_yaw");

      midFeetZUpZ = (YoDouble) yoVariableHolder.getVariable("midFeetZUpZ");

      if (yoVariableHolder.getVariable("desiredCOMHeight") == null)
      {
         // We might be controlling the desired pelvis height in this case.
         desiredCOMHeight = (YoDouble) yoVariableHolder.getVariable("pelvisDesiredPositionZ");
      }
      else
      {
         desiredCOMHeight = (YoDouble) yoVariableHolder.getVariable("desiredCOMHeight");
      }

      solePositions.set(RobotSide.LEFT, new YoFramePoint3D((YoDouble) yoVariableHolder.getVariable("leftSoleX"), (YoDouble) yoVariableHolder.getVariable("leftSoleY"),
                                                         (YoDouble) yoVariableHolder.getVariable("leftSoleZ"), ReferenceFrame.getWorldFrame()));
      solePositions.set(RobotSide.RIGHT, new YoFramePoint3D((YoDouble) yoVariableHolder.getVariable("rightSoleX"), (YoDouble) yoVariableHolder.getVariable("rightSoleY"),
                                                         (YoDouble) yoVariableHolder.getVariable("rightSoleZ"), ReferenceFrame.getWorldFrame()));

      angularMomentumX = (YoDouble) yoVariableHolder.getVariable("AngularMomentumX");
      angularMomentumY = (YoDouble) yoVariableHolder.getVariable("AngularMomentumY");
      angularMomentumZ = (YoDouble) yoVariableHolder.getVariable("AngularMomentumZ");

      icpPlannerDesiredCentroidalAngularMomentumX = (YoDouble) yoVariableHolder.getVariable("icpPlannerDesiredCentroidalAngularMomentumX");
      icpPlannerDesiredCentroidalAngularMomentumY = (YoDouble) yoVariableHolder.getVariable("icpPlannerDesiredCentroidalAngularMomentumY");
      icpPlannerDesiredCentroidalAngularMomentumZ = (YoDouble) yoVariableHolder.getVariable("icpPlannerDesiredCentroidalAngularMomentumZ");

      controllerIsInDoubleSupport = (YoBoolean) yoVariableHolder.getVariable("controllerIsInDoubleSupport");
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
