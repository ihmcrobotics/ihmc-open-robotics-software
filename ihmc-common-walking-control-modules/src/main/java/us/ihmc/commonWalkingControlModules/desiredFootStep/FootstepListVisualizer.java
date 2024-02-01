package us.ihmc.commonWalkingControlModules.desiredFootStep;

import java.awt.Color;
import java.util.ArrayList;
import java.util.List;

import org.apache.commons.lang3.mutable.MutableInt;

import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.robotics.SCS2YoGraphicHolder;
import us.ihmc.robotics.contactable.ContactablePlaneBody;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicGroupDefinition;
import us.ihmc.yoVariables.registry.YoRegistry;

public class FootstepListVisualizer implements SCS2YoGraphicHolder
{
   private static final int maxNumberOfFootstepsToVisualizePerSide = 2;
   public static final Color defaultLeftColor = new Color(0.85f, 0.35f, 0.65f, 1.0f);
   public static final Color defaultRightColor = new Color(0.15f, 0.8f, 0.15f, 1.0f);
   public static final SideDependentList<Color> defaultFeetColors = new SideDependentList<>(defaultLeftColor, defaultRightColor);

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final SideDependentList<List<FootstepVisualizer>> footstepVisualizers = new SideDependentList<>();

   public FootstepListVisualizer(SideDependentList<? extends ContactablePlaneBody> contactableFeet,
                                 YoGraphicsListRegistry yoGraphicsListRegistry,
                                 YoRegistry parentRegistry)
   {
      String graphicListName = "FootstepVisualizer";

      for (RobotSide robotSide : RobotSide.values)
      {
         ContactablePlaneBody contactableFoot = contactableFeet.get(robotSide);
         footstepVisualizers.put(robotSide, new ArrayList<FootstepVisualizer>());

         for (int i = 0; i < maxNumberOfFootstepsToVisualizePerSide; i++)
         {
            String name = robotSide.getCamelCaseNameForStartOfExpression() + "Footstep" + i;
            FootstepVisualizer footstepVisualizer = new FootstepVisualizer(name,
                                                                           graphicListName,
                                                                           robotSide,
                                                                           contactableFoot,
                                                                           defaultFeetColors.get(robotSide).darker(),
                                                                           yoGraphicsListRegistry,
                                                                           registry);
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

   @Override
   public YoGraphicDefinition getSCS2YoGraphics()
   {
      YoGraphicGroupDefinition group = new YoGraphicGroupDefinition(getClass().getSimpleName());

      for (RobotSide robotSide : RobotSide.values)
      {
         for (int i = 0; i < maxNumberOfFootstepsToVisualizePerSide; i++)
         {
            group.addChild(footstepVisualizers.get(robotSide).get(i).getSCS2YoGraphics());
         }
      }

      return group;
   }
}
