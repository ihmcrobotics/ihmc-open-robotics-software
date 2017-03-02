package us.ihmc.humanoidRobotics.footstep;

import java.util.ArrayList;

import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;


public class BagOfSingleFootstepVisualizers
{
   private final SideDependentList<ArrayList<SingleFootstepVisualizer>> singleFootstepVisualizers;
   private final SideDependentList<Integer> indices = new SideDependentList<Integer>(0, 0);


   public BagOfSingleFootstepVisualizers(int maxNumberOfFootstepsPerSide, int maxContactPoints, YoVariableRegistry registry,
           YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      singleFootstepVisualizers = new SideDependentList<ArrayList<SingleFootstepVisualizer>>(new ArrayList<SingleFootstepVisualizer>(),
              new ArrayList<SingleFootstepVisualizer>());

      for (RobotSide robotSide : RobotSide.values)
      {
         for (int footstepIndex = 0; footstepIndex < maxNumberOfFootstepsPerSide; footstepIndex++)
         {
            SingleFootstepVisualizer singleFootstepVisualizer = new SingleFootstepVisualizer(robotSide, maxContactPoints, registry, yoGraphicsListRegistry);
            singleFootstepVisualizers.get(robotSide).add(singleFootstepVisualizer);
         }
      }
   }

   public void visualizeFootstep(Footstep footstep, ContactablePlaneBody bipedFoot)
   {
      RobotSide robotSide = footstep.getRobotSide();
      int index = indices.get(robotSide);

      SingleFootstepVisualizer singleFootstepVisualizer = singleFootstepVisualizers.get(robotSide).get(index);

      singleFootstepVisualizer.visualizeFootstep(footstep, bipedFoot);
      index++;

      if (index >= singleFootstepVisualizers.get(robotSide).size())
      {
         index = 0;
      }

      indices.set(robotSide, index);
   }
}
