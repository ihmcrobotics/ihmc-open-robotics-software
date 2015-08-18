package us.ihmc.communication.packets.walking;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.Random;

import javax.vecmath.Quat4d;

import us.ihmc.communication.packetAnnotations.ClassDocumentation;
import us.ihmc.communication.packetAnnotations.FieldDocumentation;
import us.ihmc.communication.packets.IHMCRosApiPacket;
import us.ihmc.communication.packets.VisualizablePacket;
import us.ihmc.tools.ArrayTools;
import us.ihmc.tools.random.RandomTools;
import us.ihmc.communication.TransformableDataObject;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.robotSide.RobotSide;

@ClassDocumentation(documentation = "This message commands the controller to execute a list of footsteps. See FootstepDataMessage\n"
                                  + "for information about defining a footstep.")
public class FootstepDataList extends IHMCRosApiPacket<FootstepDataList> implements TransformableDataObject<FootstepDataList>, Iterable<FootstepData>, VisualizablePacket
{
   public ArrayList<FootstepData> footstepDataList = new ArrayList<FootstepData>();

   @FieldDocumentation(documentation = "swingTime is the time spent in single-support when stepping")
   public double swingTime = 0.0;
   @FieldDocumentation(documentation = "transferTime is the time spent in double-support between steps")
   public double transferTime = 0.0;

   public FootstepDataList()
   {
      // Must have null constructor for efficient serialization
   }

   public FootstepDataList(ArrayList<FootstepData> footstepDataList, double swingTime, double transferTime)
   {
      if(footstepDataList != null)
      {
         this.footstepDataList = footstepDataList;
      }
      this.swingTime = swingTime;
      this.transferTime = transferTime;
   }


   public FootstepDataList(double swingTime, double transferTime)
   {
      this.swingTime = swingTime;
      this.transferTime = transferTime;
   }

   public ArrayList<FootstepData> getDataList()
   {
      return footstepDataList;
   }

   public void add(FootstepData footstepData)
   {
      this.footstepDataList.add(footstepData);
   }

   public void clear()
   {
      this.footstepDataList.clear();
   }

   public FootstepData get(int i)
   {
      return this.footstepDataList.get(i);
   }

   public int size()
   {
      return this.footstepDataList.size();
   }


   public boolean epsilonEquals(FootstepDataList otherList, double epsilon)
   {
      for (int i = 0; i < size(); i++)
      {
         if (!otherList.get(i).epsilonEquals(get(i), epsilon))
         {
            return false;
         }
      }

      if (Math.abs(swingTime - otherList.swingTime) > epsilon)
      {
         return false;
      }

      if (Math.abs(transferTime - otherList.transferTime) > epsilon)
      {
         return false;
      }


      return true;
   }

   public String toString()
   {
      String startingFootstep = "";
      int numberOfSteps = this.size();
      if (numberOfSteps > 0)
      {
         startingFootstep = this.get(0).getLocation().toString();
         Quat4d quat4d = this.get(0).getOrientation();

         FrameOrientation frameOrientation = new FrameOrientation(ReferenceFrame.getWorldFrame(), quat4d);
         startingFootstep = startingFootstep + ", ypr= " + ArrayTools.arrayToString(frameOrientation.getYawPitchRoll());
      }

      System.out.println(this.size());

      if (this.size() == 1)
      {
         RobotSide footSide = footstepDataList.get(0).getRobotSide();
         String side = "Right Step";
         if (footSide.getSideNameFirstLetter().equals("L"))
            side = "Left Step";

         return (side);
      }
      else
      {
         return (this.size() + " Footsteps");
      }


   }

   public Iterator<FootstepData> iterator()
   {
      return footstepDataList.iterator();
   }

   public FootstepDataList transform(RigidBodyTransform transform)
   {
      FootstepDataList ret = new FootstepDataList(swingTime, transferTime);

      for (FootstepData footstepData : (FootstepDataList) this)
      {
         FootstepData transformedFootstepData = footstepData.transform(transform);
         ret.add(transformedFootstepData);
      }

      return ret;
   }

   public FootstepDataList(Random random)
   {

      int footstepListSize = random.nextInt(20);
      for (int i = 0; i < footstepListSize; i++)
      {
         footstepDataList.add(new FootstepData(random));
      }

      this.swingTime = RandomTools.generateRandomDoubleWithEdgeCases(random, 0.1);
      this.transferTime = RandomTools.generateRandomDoubleWithEdgeCases(random, 0.1);
   }
}
