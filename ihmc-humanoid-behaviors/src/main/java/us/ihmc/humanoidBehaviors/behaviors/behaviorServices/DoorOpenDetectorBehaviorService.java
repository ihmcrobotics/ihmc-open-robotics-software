package us.ihmc.humanoidBehaviors.behaviors.behaviorServices;

import java.util.ArrayList;

import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.ros2.Ros2Node;

public class DoorOpenDetectorBehaviorService extends FiducialDetectorBehaviorService
{

   private int numberToAverage = 5;
   private ArrayList<FramePose3D> originPoses;
   private ArrayList<FramePose3D> doorPoses;

   public FramePose3D averageOrigin;
   private FramePose3D averageCurrentDoorLocation;
   public FramePose3D newPose = null;
   private boolean doorOpen = false;
   private float openDistance = 0.0127f;
   private boolean run = false;

   public DoorOpenDetectorBehaviorService(String robotName, String ThreadName, Ros2Node ros2Node, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      super(robotName, ThreadName, ros2Node, yoGraphicsListRegistry);
      initialize();
   }
   public void run(boolean run)
   {
      this.run = run;
   }
   @Override
   public void initialize()
   {
      super.initialize();
      originPoses = new ArrayList<>();
      doorPoses = new ArrayList<>();
      averageOrigin = null;
      averageCurrentDoorLocation = null;
      doorOpen = false;
      run = false;
   }

   public boolean isDoorOpen()
   {
      return doorOpen;
   }

   public boolean doorDetected()
   {
      return averageOrigin != null;
   }

   public void reset()
   {
      doorOpen = false;
      averageOrigin = null;
      run = false;

   }

   @Override
   public void doThreadAction()
   {
      
      super.doThreadAction();
      if (run)
      {
         if (getGoalHasBeenLocated())
         {
            newPose = new FramePose3D(ReferenceFrame.getWorldFrame());
            getReportedGoalPoseWorldFrame(newPose);
            if (averageOrigin == null)
            {

               originPoses.add(newPose);

               if (originPoses.size() >= numberToAverage)
               {
                  averageOrigin = averageFramePoses(originPoses);
               }

            }
            //track the door for movements
            else
            {

               doorPoses.add(newPose);
               if (doorPoses.size() > numberToAverage)
                  doorPoses.remove(0);

               if (doorPoses.size() >= numberToAverage)
               {
                  averageCurrentDoorLocation = averageFramePoses(doorPoses);

                  if (averageCurrentDoorLocation != null && averageOrigin.getPositionDistance(averageCurrentDoorLocation) > openDistance)
                  {
                     doorOpen = true;
                  }
                  else
                  {
                     doorOpen = false;
                  }
               }
            }

            super.initialize();
         }
      }
   }

   private FramePose3D averageFramePoses(ArrayList<FramePose3D> poses)
   {
      if (poses.size() > 0)
      {
         float numberOfPoses = poses.size();
         FramePose3D aveagedPose = new FramePose3D(ReferenceFrame.getWorldFrame());
         for (FramePose3D pose : poses)
         {
            aveagedPose.setX(aveagedPose.getX() + pose.getX());
            aveagedPose.setY(aveagedPose.getY() + pose.getY());
            aveagedPose.setZ(aveagedPose.getZ() + pose.getZ());
         }
         aveagedPose.setX(aveagedPose.getX() / numberOfPoses);
         aveagedPose.setY(aveagedPose.getY() / numberOfPoses);
         aveagedPose.setZ(aveagedPose.getZ() / numberOfPoses);
         return aveagedPose;
      }
      else
         return null;

   }

}
