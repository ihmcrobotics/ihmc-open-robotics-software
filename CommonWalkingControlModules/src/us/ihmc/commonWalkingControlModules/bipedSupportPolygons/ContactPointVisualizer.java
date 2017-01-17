package us.ihmc.commonWalkingControlModules.bipedSupportPolygons;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.commonWalkingControlModules.controllers.Updatable;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicVector;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

/**
 * @author twan
 *         Date: 5/28/13
 */
public class ContactPointVisualizer implements Updatable
{
   private final static ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final List<YoFramePoint> contactPointsWorld = new ArrayList<YoFramePoint>();
   private final List<YoGraphicPosition> dynamicGraphicPositions = new ArrayList<YoGraphicPosition>();
   private final List<YoGraphicVector> dynamicGraphicVectors = new ArrayList<YoGraphicVector>();
   private final List<YoFrameVector> normalVectors = new ArrayList<YoFrameVector>();
   private final double normalVectorScale = 0.1;
   private final int maxNumberOfDynamicGraphicPositions;
   private final ArrayList<? extends PlaneContactState> contactStates;

   public ContactPointVisualizer(ArrayList<? extends PlaneContactState> contactStates, YoGraphicsListRegistry yoGraphicsListRegistry,
         YoVariableRegistry parentRegistry)
   {
      this.contactStates = contactStates;
      int totalNumberOfContactPoints = 0;
      for (int i = 0; i < contactStates.size(); i++)
         totalNumberOfContactPoints += contactStates.get(i).getTotalNumberOfContactPoints();

      maxNumberOfDynamicGraphicPositions = totalNumberOfContactPoints;

      for (int i = 0; i < maxNumberOfDynamicGraphicPositions; i++)
      {
         YoFramePoint contactPointWorld = new YoFramePoint("contactPoint" + i, worldFrame, this.registry);
         contactPointsWorld.add(contactPointWorld);
         YoGraphicPosition dynamicGraphicPosition = new YoGraphicPosition("contactViz" + i, contactPointWorld, 0.01, YoAppearance.Crimson());
         dynamicGraphicPositions.add(dynamicGraphicPosition);
         yoGraphicsListRegistry.registerYoGraphic("contactPoints", dynamicGraphicPosition);

         YoFrameVector normalVector = new YoFrameVector("contactNormal" + i, worldFrame, registry);
         normalVectors.add(normalVector);
         YoGraphicVector dynamicGraphicVector = new YoGraphicVector("contactNormalViz" + i, contactPointWorld, normalVector, YoAppearance.Crimson());
         dynamicGraphicVectors.add(dynamicGraphicVector);
         yoGraphicsListRegistry.registerYoGraphic("contactPoints", dynamicGraphicVector);
      }
      parentRegistry.addChild(registry);
   }

   private final FramePoint tempFramePoint = new FramePoint(worldFrame);
   private final FrameVector tempFrameVector = new FrameVector(worldFrame);

   @Override
   public void update(double time)
   {
      int i = 0;
      for (int j = 0; j < contactStates.size(); j++)
      {
         PlaneContactState contactState = contactStates.get(j);
         contactState.getContactNormalFrameVector(tempFrameVector);
         tempFrameVector.changeFrame(worldFrame);
         tempFrameVector.scale(normalVectorScale);

         List<? extends ContactPointInterface> contactPoints = contactState.getContactPoints();
         for (int k = 0; k < contactPoints.size(); k++)
         {
            updateContactPointDynamicGraphicObjects(i++, contactPoints.get(k));
         }
      }
   }

   private void updateContactPointDynamicGraphicObjects(int i, ContactPointInterface contactPoint)
   {
      if (contactPoint.isInContact())
      {
         contactPoint.getPosition(tempFramePoint);
         tempFramePoint.changeFrame(worldFrame);
         contactPointsWorld.get(i).set(tempFramePoint);
         normalVectors.get(i).set(tempFrameVector);

         dynamicGraphicPositions.get(i).showGraphicObject();
         dynamicGraphicVectors.get(i).showGraphicObject();
      }
      else
      {
         contactPointsWorld.get(i).setToNaN();
         normalVectors.get(i).set(Double.NaN, Double.NaN, Double.NaN);
         dynamicGraphicPositions.get(i).hideGraphicObject();
         dynamicGraphicVectors.get(i).hideGraphicObject();
      }

      dynamicGraphicPositions.get(i).update();
      dynamicGraphicVectors.get(i).update();
   }
}
