package us.ihmc.commonWalkingControlModules.bipedSupportPolygons;

import java.awt.Color;
import java.util.ArrayList;
import java.util.Collection;
import java.util.HashMap;
import java.util.List;

import us.ihmc.commonWalkingControlModules.controllers.regularWalkingGait.Updatable;
import us.ihmc.commonWalkingControlModules.desiredFootStep.DesiredFootstepCalculatorTools;
import us.ihmc.utilities.math.geometry.FrameConvexPolygon2d;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.ReferenceFrame;

import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.plotting.DynamicGraphicYoPolygonArtifact;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;
import com.yobotics.simulationconstructionset.util.math.frames.YoFrameConvexPolygon2d;

public class FootPolygonVisualizer implements Updatable
{
   private final String name = getClass().getSimpleName();
   private final YoVariableRegistry registry = new YoVariableRegistry(name);
   private final HashMap<ContactState, YoFrameConvexPolygon2d> yoFootPolygons = new HashMap<ContactState, YoFrameConvexPolygon2d>();
   private static final List<Color> colors = new ArrayList<Color>();
   static
   {
      colors.add(new Color(53, 184, 144));
      colors.add(new Color(202, 119, 11));
   }

   public FootPolygonVisualizer(Collection<? extends ContactState> contactStates, DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry,
                                YoVariableRegistry parentRegistry)
   {
      if (dynamicGraphicObjectsListRegistry != null)
      {
         int colorIndex = 0;
         for (ContactState contactState : contactStates)
         {
            String contactStateName = contactState.getBodyFrame().getName(); // TODO: give contactStates a name
            YoFrameConvexPolygon2d yoFootPolygon = new YoFrameConvexPolygon2d(contactStateName, "", ReferenceFrame.getWorldFrame(), 30, registry);
            yoFootPolygons.put(contactState, yoFootPolygon);
            Color color = colors.get(colorIndex++);

            DynamicGraphicYoPolygonArtifact dynamicGraphicYoPolygonArtifact = new DynamicGraphicYoPolygonArtifact(contactStateName, yoFootPolygon, color,
                                                                                 false);
            dynamicGraphicObjectsListRegistry.registerArtifact(contactStateName, dynamicGraphicYoPolygonArtifact);
         }
      }

      parentRegistry.addChild(registry);
   }

   public void update(double time)
   {
      for (ContactState contactState : yoFootPolygons.keySet())
      {
         YoFrameConvexPolygon2d yoFootPolygon = yoFootPolygons.get(contactState);
         if (yoFootPolygon != null)
         {
            List<FramePoint> contactPoints = contactState.getContactPoints();
            contactPoints = DesiredFootstepCalculatorTools.fixTwoPointsAndCopy(contactPoints); // TODO: terrible
            if (contactPoints.size() > 0)
               yoFootPolygon.setFrameConvexPolygon2d(FrameConvexPolygon2d.constructByProjectionOntoXYPlane(contactPoints, yoFootPolygon.getReferenceFrame()));
            else
               yoFootPolygon.hide();
         }
      }
   }
}
