package us.ihmc.aware.controller.force.taskSpaceController;

import us.ihmc.aware.util.ContactState;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;

public class QuadrupedTaskSpaceControllerSettings
{
   private final QuadrantDependentList<ContactState> contactState;
   private final double[] comTorqueCommandWeights;
   private final double[] comForceCommandWeights;
   private final QuadrantDependentList<double[]> soleForceCommandWeights;

   public QuadrupedTaskSpaceControllerSettings()
   {
      contactState = new QuadrantDependentList<>();
      comTorqueCommandWeights = new double[3];
      comForceCommandWeights = new double[3];
      soleForceCommandWeights = new QuadrantDependentList<>();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         soleForceCommandWeights.set(robotQuadrant, new double[3]);
      }
      setDefaults();
   }

   public void setDefaults()
   {
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         contactState.set(robotQuadrant, ContactState.IN_CONTACT);
      }
      for (int i = 0; i < 3; i++)
      {
         comTorqueCommandWeights[i] = 1.0;
         comForceCommandWeights[i] = 1.0;
         for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
         {
            soleForceCommandWeights.get(robotQuadrant)[i] = 0.0;
         }
      }
   }

   public void setContactState(RobotQuadrant robotQuadrant, ContactState contactState)
   {
      this.contactState.set(robotQuadrant, contactState);
   }

   public void setComTorqueCommandWeights(double[] weights)
   {
      for (int i = 0; i < 3; i++)
      {
         comTorqueCommandWeights[i] = Math.max(weights[i], 0.0);
      }
   }

   public void setComForceCommandWeights(double[] weights)
   {
      for (int i = 0; i < 3; i++)
      {
         comForceCommandWeights[i] = weights[i];
      }
   }

   public void setSoleForceCommandWeights(RobotQuadrant robotQuadrant, double[] weights)
   {
      for (int i = 0; i < 3; i++)
      {
         soleForceCommandWeights.get(robotQuadrant)[i] = weights[i];
      }
   }

   public void setComTorqueCommandWeights(double weightX, double weightY, double weightZ)
   {
      comTorqueCommandWeights[0] = weightX;
      comTorqueCommandWeights[1] = weightY;
      comTorqueCommandWeights[2] = weightZ;
   }

   public void setComForceCommandWeights(double weightX, double weightY, double weightZ)
   {
      comForceCommandWeights[0] = weightX;
      comForceCommandWeights[1] = weightY;
      comForceCommandWeights[2] = weightZ;
   }

   public void setSoleForceCommandWeights(RobotQuadrant robotQuadrant, double weightX, double weightY, double weightZ)
   {
      soleForceCommandWeights.get(robotQuadrant)[0] = weightX;
      soleForceCommandWeights.get(robotQuadrant)[1] = weightY;
      soleForceCommandWeights.get(robotQuadrant)[2] = weightZ;
   }

   public ContactState getContactState(RobotQuadrant robotQuadrant)
   {
      return contactState.get(robotQuadrant);
   }

   public /* const */ double[] getComTorqueCommandWeights()
   {
      return comTorqueCommandWeights;
   }

   public /* const */ double[] getComForceCommandWeights()
   {
      return comForceCommandWeights;
   }

   public /* const */ double[] getSoleForceCommandWeights(RobotQuadrant robotQuadrant)
   {
      return soleForceCommandWeights.get(robotQuadrant);
   }
}

