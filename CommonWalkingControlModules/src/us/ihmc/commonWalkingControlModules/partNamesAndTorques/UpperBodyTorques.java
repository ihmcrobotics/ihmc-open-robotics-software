package us.ihmc.commonWalkingControlModules.partNamesAndTorques;

import us.ihmc.robotics.robotSide.RobotSide;

public class UpperBodyTorques
{
   private SpineTorques spineTorques = new SpineTorques();
   private ArmTorques[] armTorquesArray = new ArmTorques[RobotSide.values.length];
   private NeckTorques neckTorques = new NeckTorques();

   public UpperBodyTorques()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         armTorquesArray[robotSide.ordinal()] = new ArmTorques(robotSide);
      }
   }


// private UpperBodyTorques(UpperBodyTorques upperBodyTorques)
// {
//    this.spineTorques = upperBodyTorques.getSpineTorquesCopy();
//    this.headTorques = upperBodyTorques.getHeadTorquesCopy();
//    this.armTorquesArray = upperBodyTorques.getArmTorquesArrayCopy();
// }

   public ArmTorques[] getArmTorques()
   {
      return armTorquesArray;
   }
   
   public ArmTorques getArmTorques(RobotSide robotSide)
   {
      return armTorquesArray[robotSide.ordinal()];
   }
   
   public ArmTorques getArmTorquesCopy(RobotSide robotSide)
   {
      return armTorquesArray[robotSide.ordinal()].getArmTorquesCopy();
   }

   public ArmTorques[] getArmTorquesArrayCopy()
   {
      ArmTorques[] ret = new ArmTorques[armTorquesArray.length];

      for (int i = 0; i < armTorquesArray.length; i++)
      {
         ret[i] = armTorquesArray[i].getArmTorquesCopy();
      }

      return ret;
   }

   public SpineTorques getSpineTorques()
   {
      return spineTorques;
   }
   
   public SpineTorques getSpineTorquesCopy()
   {
      return spineTorques.getSpineTorquesCopy();
   }

   public NeckTorques getNeckTorques()
   {
      return neckTorques;
   }

   public NeckTorques getNeckTorquesCopy()
   {
      return neckTorques.getNeckTorquesCopy();
   }

   public void setArmTorques(ArmTorques armTorques)
   {
      armTorquesArray[armTorques.getRobotSide().ordinal()] = armTorques.getArmTorquesCopy();
   }

   public void setSpineTorques(SpineTorques spineTorques)
   {
      this.spineTorques = spineTorques.getSpineTorquesCopy();
   }


   public void setNeckTorques(NeckTorques neckTorques)
   {
      this.neckTorques = neckTorques.getNeckTorquesCopy();
   }

   public void setTorquesToZero()
   {
      neckTorques.setTorquesToZero();
      spineTorques.setTorquesToZero();

      for (ArmTorques armTorques : armTorquesArray)
      {
         armTorques.setTorquesToZero();
      }
   }
}
