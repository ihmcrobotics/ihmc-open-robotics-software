package us.ihmc.commonWalkingControlModules.bipedSupportPolygons;

import java.awt.Color;
import java.util.ArrayList;
import java.util.List;

import us.ihmc.commonWalkingControlModules.controllers.Updatable;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.FrameConvexPolygon2d;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.yoUtilities.graphics.YoGraphicsListRegistry;
import us.ihmc.yoUtilities.graphics.plotting.YoArtifactPolygon;
import us.ihmc.yoUtilities.math.frames.YoFrameConvexPolygon2d;


public class FootPolygonVisualizer implements Updatable
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final String name = getClass().getSimpleName();
   private final YoVariableRegistry registry = new YoVariableRegistry(name);
   private final List<YoFrameConvexPolygon2d> yoFootPolygons = new ArrayList<YoFrameConvexPolygon2d>();
   private final List<FrameConvexPolygon2d> footPolygons = new ArrayList<FrameConvexPolygon2d>();
   private final List<? extends PlaneContactState> contactStates;
   private static final List<Color> colors = new ArrayList<Color>();

   public static final Color defaultLeftColor = new Color(0.85f, 0.35f, 0.65f, 1.0f);
   public static final Color defaultRightColor = new Color(0.15f, 0.8f, 0.15f, 1.0f);

   static
   {
      colors.add(defaultLeftColor);
      colors.add(defaultRightColor);
   }

   public FootPolygonVisualizer(List<? extends PlaneContactState> contactStates, YoGraphicsListRegistry yoGraphicsListRegistry,
         YoVariableRegistry parentRegistry)
   {
      this.contactStates = new ArrayList<PlaneContactState>(contactStates);

      if (yoGraphicsListRegistry != null)
      {
         int colorIndex = 0;
         for (int i = 0; i < contactStates.size(); i++)
         {
            PlaneContactState contactState = contactStates.get(i);
            String contactStateName = contactState.getFrameAfterParentJoint().getName(); // TODO: give contactStates a name
            YoFrameConvexPolygon2d yoFootPolygon = new YoFrameConvexPolygon2d(contactStateName, "", worldFrame, 30, registry);
            yoFootPolygons.add(yoFootPolygon);

            FrameConvexPolygon2d footPolygon = new FrameConvexPolygon2d(worldFrame);
            footPolygons.add(footPolygon);

            Color color = colors.get(colorIndex++);

            YoArtifactPolygon dynamicGraphicYoPolygonArtifact = new YoArtifactPolygon(contactStateName, yoFootPolygon, color, false);
            yoGraphicsListRegistry.registerArtifact(contactStateName, dynamicGraphicYoPolygonArtifact);
         }
      }

      parentRegistry.addChild(registry);
   }

   private final FramePoint tempFramePoint = new FramePoint();
   public void update(double time)
   {
      for (int i = 0; i < contactStates.size(); i++)
      {
         PlaneContactState contactState = contactStates.get(i);
         YoFrameConvexPolygon2d yoFootPolygon = yoFootPolygons.get(i);
         FrameConvexPolygon2d footPolygon = footPolygons.get(i);

         footPolygon.clear(worldFrame);
         if (yoFootPolygon == null)
            continue;

         if (!contactState.inContact())
         {
            yoFootPolygon.hide();
            continue;
         }

         for (int j = 0; j < contactState.getTotalNumberOfContactPoints(); j++)
         {
            ContactPointInterface contactPoint = contactState.getContactPoints().get(j);
            if (contactPoint.isInContact())
            {
               contactPoint.getPosition(tempFramePoint);
               footPolygon.addVertexByProjectionOntoXYPlane(tempFramePoint);
            }
         }
         footPolygon.update();
         yoFootPolygon.setFrameConvexPolygon2d(footPolygon);
      }
   }
}
