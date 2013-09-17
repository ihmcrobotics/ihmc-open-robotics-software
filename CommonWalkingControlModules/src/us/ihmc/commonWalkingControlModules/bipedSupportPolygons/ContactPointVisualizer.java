package us.ihmc.commonWalkingControlModules.bipedSupportPolygons;

import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicPosition;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicVector;
import com.yobotics.simulationconstructionset.util.math.frames.YoFramePoint;
import com.yobotics.simulationconstructionset.util.math.frames.YoFrameVector;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;

/**
 * @author twan
 *         Date: 5/28/13
 */
public class ContactPointVisualizer
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final List<YoFramePoint> contactPointsWorld = new ArrayList<YoFramePoint>();
   private final List<DynamicGraphicPosition> dynamicGraphicPositions = new ArrayList<DynamicGraphicPosition>();
   private final List<DynamicGraphicVector> dynamicGraphicVectors = new ArrayList<DynamicGraphicVector>();
   private final List<YoFrameVector> normalVectors = new ArrayList<YoFrameVector>();
   private final double normalVectorScale = 0.1;

   public ContactPointVisualizer(int maxNumberOfDynamicGraphicPositions, DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry,
                                 YoVariableRegistry parentRegistry)
   {
      for (int i = 0; i < maxNumberOfDynamicGraphicPositions; i++)
      {
         YoFramePoint contactPointWorld = new YoFramePoint("contactPoint" + i, ReferenceFrame.getWorldFrame(), this.registry);
         contactPointsWorld.add(contactPointWorld);
         DynamicGraphicPosition dynamicGraphicPosition = contactPointWorld.createDynamicGraphicPosition("contactViz" + i, 0.01,
               YoAppearance.Crimson());
         dynamicGraphicPositions.add(dynamicGraphicPosition);
         dynamicGraphicObjectsListRegistry.registerDynamicGraphicObject("contactPoints", dynamicGraphicPosition);

         YoFrameVector normalVector = new YoFrameVector("contactNormal" + i, ReferenceFrame.getWorldFrame(), registry);
         normalVectors.add(normalVector);
         DynamicGraphicVector dynamicGraphicVector = new DynamicGraphicVector("contactNormalViz" + i, contactPointWorld, normalVector, YoAppearance.Crimson());
         dynamicGraphicVectors.add(dynamicGraphicVector);
         dynamicGraphicObjectsListRegistry.registerDynamicGraphicObject("contactPoints", dynamicGraphicVector);
      }
      parentRegistry.addChild(registry);
   }

   public void update(Collection<? extends PlaneContactState> contactStates)
   {
      int i = 0;
      for (PlaneContactState contactState : contactStates)
      {
         for (FramePoint framePoint : contactState.getContactFramePoints())
         {
            framePoint = new FramePoint(framePoint);
            framePoint.changeFrame(ReferenceFrame.getWorldFrame());
            contactPointsWorld.get(i).set(framePoint);
            FrameVector normalVector = new FrameVector(contactState.getContactNormalFrameVector());
            normalVector.changeFrame(ReferenceFrame.getWorldFrame());
            normalVector.scale(normalVectorScale);
            normalVectors.get(i).set(normalVector);

            dynamicGraphicPositions.get(i).showGraphicObject();
            dynamicGraphicVectors.get(i).showGraphicObject();

            dynamicGraphicPositions.get(i).update();
            dynamicGraphicVectors.get(i).update();
            i++;
         }
      }

      for (int j = i; j < dynamicGraphicPositions.size(); j++)
      {
         contactPointsWorld.get(j).setToNaN();
         normalVectors.get(j).set(Double.NaN, Double.NaN, Double.NaN);
         dynamicGraphicPositions.get(j).hideGraphicObject();
         dynamicGraphicVectors.get(j).hideGraphicObject();
      }

   }
}
