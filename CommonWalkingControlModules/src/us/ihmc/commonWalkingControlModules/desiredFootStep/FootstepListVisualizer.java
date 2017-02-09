package us.ihmc.commonWalkingControlModules.desiredFootStep;

import java.awt.Color;
import java.util.ArrayList;
import java.util.List;

import org.apache.commons.lang3.mutable.MutableInt;

import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearanceRGBColor;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public class FootstepListVisualizer
{
   private static final int maxNumberOfFootstepsToVisualizePerSide = 2;
   public static final Color defaultLeftColor = new Color(0.85f, 0.35f, 0.65f, 1.0f);
   public static final Color defaultRightColor = new Color(0.15f, 0.8f, 0.15f, 1.0f);
   public static final SideDependentList<Color> defaultFeetColors = new SideDependentList<Color>(defaultLeftColor, defaultRightColor);

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final SideDependentList<List<FootstepVisualizer>> footstepVisualizers = new SideDependentList<>();

   public FootstepListVisualizer(SideDependentList<? extends ContactablePlaneBody> contactableFeet, YoGraphicsListRegistry yoGraphicsListRegistry,
         YoVariableRegistry parentRegistry)
   {
      String graphicListName = "FootstepVisualizer";

      for (RobotSide robotSide : RobotSide.values)
      {
         ContactablePlaneBody contactableFoot = contactableFeet.get(robotSide);
         footstepVisualizers.put(robotSide, new ArrayList<FootstepVisualizer>());

         for (int i = 0; i < maxNumberOfFootstepsToVisualizePerSide; i++)
         {
            String name = robotSide.getCamelCaseNameForStartOfExpression() + "Footstep" + i;
            AppearanceDefinition footstepColor = new YoAppearanceRGBColor(defaultFeetColors.get(robotSide).darker(), 0.0);
            FootstepVisualizer footstepVisualizer = new FootstepVisualizer(name, graphicListName, robotSide, contactableFoot, footstepColor, yoGraphicsListRegistry, registry);
            footstepVisualizers.get(robotSide).add(footstepVisualizer);
         }
      }

      parentRegistry.addChild(registry);
   }

   private final SideDependentList<MutableInt> counters = new SideDependentList<MutableInt>(new MutableInt(0), new MutableInt(0));

   public void updateFirstFootstep(Footstep firstFootstep)
   {
      RobotSide robotSide = firstFootstep.getRobotSide();
      if (counters.get(robotSide).intValue() < 1)
         return;

      footstepVisualizers.get(robotSide).get(0).update(firstFootstep);
   }

   public void update(List<Footstep> footsteps)
   {
      for (RobotSide robotSide : RobotSide.values)
         counters.get(robotSide).setValue(0);

      for (int i = 0; i < footsteps.size(); i++)
      {
         Footstep footstep = footsteps.get(i);
         RobotSide robotSide = footstep.getRobotSide();
         MutableInt counter = counters.get(robotSide);
         if (counter.intValue() < maxNumberOfFootstepsToVisualizePerSide)
         {
            FootstepVisualizer footstepVisualizer = footstepVisualizers.get(robotSide).get(counter.intValue());
            footstepVisualizer.update(footstep);
            counter.increment();
         }
      }

      for (RobotSide robotside : RobotSide.values)
      {
         for (int i = counters.get(robotside).intValue(); i < maxNumberOfFootstepsToVisualizePerSide; i++)
            footstepVisualizers.get(robotside).get(i).hide();
      }
   }
}
