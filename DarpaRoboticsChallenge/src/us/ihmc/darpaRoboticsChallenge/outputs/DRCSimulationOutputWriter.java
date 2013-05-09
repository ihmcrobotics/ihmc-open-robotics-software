package us.ihmc.darpaRoboticsChallenge.outputs;

import com.yobotics.simulationconstructionset.OneDegreeOfFreedomJoint;
import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.SdfLoader.SDFPerfectSimulatedOutputWriter;
import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.utilities.Pair;
import us.ihmc.utilities.screwTheory.OneDoFJoint;

public class DRCSimulationOutputWriter extends SDFPerfectSimulatedOutputWriter implements DRCOutputWriter
{
//   private final double controlDT;
   public DRCSimulationOutputWriter(SDFRobot robot, double controlDT)
   {
      super(robot);
//      this.controlDT = controlDT;

   }

   public void writeAfterController(long timestamp)
   {
      // Do not write here, because it will set the robot's torques while the simulation is running
   }

   public void writeAfterEstimator()
   {
      // Nothing to do here
   }

   public void writeAfterSimulationTick()
   {
      for (Pair<OneDegreeOfFreedomJoint, OneDoFJoint> jointPair : revoluteJoints)
      {
         OneDegreeOfFreedomJoint pinJoint = jointPair.first();
         OneDoFJoint revoluteJoint = jointPair.second();


         boolean useFeedForward = false;
         double dampingComp;

         if (useFeedForward)
         {
         //This is just velocity based
         dampingComp = - revoluteJoint.getTauDamping();
         //This is using velocity and acceleration
//         double dampingComp = 0.9 * (revoluteJoint.getDampingParameter() * (revoluteJoint.getQd() + revoluteJoint.getQddDesired() * controlDT));
         }
         else
            dampingComp = 0.0;

         pinJoint.setTau(revoluteJoint.getTau() + dampingComp);
      }
   }

   public void setFullRobotModel(SDFFullRobotModel fullRobotModel)
   {
      super.setFullRobotModel(fullRobotModel);
   }

   public void setEstimatorModel(SDFFullRobotModel estimatorModel)
   {
      // TODO Auto-generated method stub
      
   }

}
