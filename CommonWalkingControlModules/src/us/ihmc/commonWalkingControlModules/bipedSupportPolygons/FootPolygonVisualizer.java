package us.ihmc.commonWalkingControlModules.bipedSupportPolygons;

import java.awt.Color;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

import us.ihmc.commonWalkingControlModules.controllers.Updatable;
import us.ihmc.utilities.math.geometry.FrameConvexPolygon2d;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.ReferenceFrame;

import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.plotting.DynamicGraphicYoPolygonArtifact;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;
import com.yobotics.simulationconstructionset.util.math.frames.YoFrameConvexPolygon2d;

public class FootPolygonVisualizer implements Updatable
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   
   private final String name = getClass().getSimpleName();
   private final YoVariableRegistry registry = new YoVariableRegistry(name);
   private final List<YoFrameConvexPolygon2d> yoFootPolygons = new ArrayList<YoFrameConvexPolygon2d>();
   private final List<FrameConvexPolygon2d> footPolygons = new ArrayList<FrameConvexPolygon2d>();
   private final List<? extends PlaneContactState> contactStates;
   private final Map<PlaneContactState, List<FramePoint>> contactPoints = new LinkedHashMap<PlaneContactState, List<FramePoint>>();
   private static final List<Color> colors = new ArrayList<Color>();
   
   public static final Color defaultLeftColor = new Color(0.85f, 0.35f, 0.65f, 1.0f);
   public static final Color defaultRightColor = new Color(0.15f, 0.8f, 0.15f, 1.0f);
   
   static
   {
      colors.add(defaultLeftColor);
      colors.add(defaultRightColor);
   }

   public FootPolygonVisualizer(List<? extends PlaneContactState> contactStates, DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry,
                                YoVariableRegistry parentRegistry)
   {
      this.contactStates = new ArrayList<PlaneContactState>(contactStates);
      
      if (dynamicGraphicObjectsListRegistry != null)
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
            
            contactPoints.put(contactState, contactState.getContactFramePointsInContactCopy());
            
            Color color = colors.get(colorIndex++);

            DynamicGraphicYoPolygonArtifact dynamicGraphicYoPolygonArtifact = new DynamicGraphicYoPolygonArtifact(contactStateName, yoFootPolygon, color, false);
            dynamicGraphicObjectsListRegistry.registerArtifact(contactStateName, dynamicGraphicYoPolygonArtifact);
         }
      }

      parentRegistry.addChild(registry);
   }

   public void update(double time)
   {
      for (int i = 0; i < contactStates.size(); i++)
      {
         PlaneContactState contactState = contactStates.get(i);
         YoFrameConvexPolygon2d yoFootPolygon = yoFootPolygons.get(i);
         FrameConvexPolygon2d footPolygon = footPolygons.get(i);
         
         if (yoFootPolygon != null)
         {
            contactState.getContactFramePointsInContact(contactPoints.get(contactState));
            if (contactPoints.get(contactState).size() > 0)
            {
               footPolygon.setIncludingFrameByProjectionOntoXYPlane(worldFrame, contactPoints.get(contactState));
               yoFootPolygon.setFrameConvexPolygon2d(footPolygon);
            }
            else
               yoFootPolygon.hide();
         }
      }
   }
}
