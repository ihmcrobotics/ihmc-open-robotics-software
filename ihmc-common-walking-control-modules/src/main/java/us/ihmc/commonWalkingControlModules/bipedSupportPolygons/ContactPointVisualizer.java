package us.ihmc.commonWalkingControlModules.bipedSupportPolygons;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.commonWalkingControlModules.controllers.Updatable;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicVector;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.SCS2YoGraphicHolder;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinitionFactory;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicGroupDefinition;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;

/**
 * @author twan Date: 5/28/13
 */
public class ContactPointVisualizer implements Updatable, SCS2YoGraphicHolder
{
   private final static ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final List<YoFramePoint3D> contactPointsWorld = new ArrayList<>();
   private final List<YoGraphicPosition> yoGraphicPositions;
   private final List<YoGraphicVector> yoGraphicVectors;
   private final List<YoFrameVector3D> normalVectors = new ArrayList<>();
   private final double normalVectorScale = 0.1;
   private final int maxNumberOfYoGraphicPositions;
   private final List<? extends PlaneContactState> contactStates;

   public ContactPointVisualizer(List<? extends PlaneContactState> contactStates, YoGraphicsListRegistry yoGraphicsListRegistry, YoRegistry parentRegistry)
   {
      this.contactStates = contactStates;
      int totalNumberOfContactPoints = 0;
      for (int i = 0; i < contactStates.size(); i++)
         totalNumberOfContactPoints += contactStates.get(i).getTotalNumberOfContactPoints();

      maxNumberOfYoGraphicPositions = totalNumberOfContactPoints;

      for (int i = 0; i < maxNumberOfYoGraphicPositions; i++)
      {
         YoFramePoint3D contactPointWorld = new YoFramePoint3D("contactPoint" + i, worldFrame, registry);
         contactPointsWorld.add(contactPointWorld);

         YoFrameVector3D normalVector = new YoFrameVector3D("contactNormal" + i, worldFrame, registry);
         normalVectors.add(normalVector);
      }

      if (yoGraphicsListRegistry != null)
      {
         yoGraphicPositions = new ArrayList<>();
         yoGraphicVectors = new ArrayList<>();
         for (int i = 0; i < maxNumberOfYoGraphicPositions; i++)
         {
            YoGraphicPosition yoGraphicPosition = new YoGraphicPosition("contactViz" + i, contactPointsWorld.get(i), 0.01, YoAppearance.Crimson());
            yoGraphicPositions.add(yoGraphicPosition);
            yoGraphicsListRegistry.registerYoGraphic("contactPoints", yoGraphicPosition);
            YoGraphicVector yoGraphicVector = new YoGraphicVector("contactNormalViz" + i, contactPointsWorld.get(i), normalVectors.get(i), YoAppearance.Crimson());
            yoGraphicVectors.add(yoGraphicVector);
            yoGraphicsListRegistry.registerYoGraphic("contactPoints", yoGraphicVector);
         }
      }
      else
      {
         yoGraphicPositions = null;
         yoGraphicVectors = null;
      }
      parentRegistry.addChild(registry);
   }

   private final FrameVector3D tempFrameVector = new FrameVector3D(worldFrame);

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

         List<? extends ContactPointBasics> contactPoints = contactState.getContactPoints();
         for (int k = 0; k < contactPoints.size(); k++)
         {
            updateContactPointYoGraphics(i++, contactPoints.get(k));
         }
      }
   }

   private void updateContactPointYoGraphics(int i, ContactPointBasics contactPoint)
   {
      if (contactPoint.isInContact())
      {
         contactPointsWorld.get(i).setMatchingFrame(contactPoint);
         normalVectors.get(i).set(tempFrameVector);

         if (yoGraphicPositions != null)
         {
            yoGraphicPositions.get(i).showGraphicObject();
            yoGraphicVectors.get(i).showGraphicObject();
         }
      }
      else
      {
         contactPointsWorld.get(i).setToNaN();
         normalVectors.get(i).set(Double.NaN, Double.NaN, Double.NaN);
         if (yoGraphicPositions != null)
         {
            yoGraphicPositions.get(i).hideGraphicObject();
            yoGraphicVectors.get(i).hideGraphicObject();
         }
      }

      if (yoGraphicPositions != null)
      {
         yoGraphicPositions.get(i).update();
         yoGraphicVectors.get(i).update();
      }
   }

   @Override
   public YoGraphicDefinition getSCS2YoGraphics()
   {
      YoGraphicGroupDefinition group = new YoGraphicGroupDefinition(getClass().getSimpleName());

      for (int i = 0; i < maxNumberOfYoGraphicPositions; i++)
      {
         group.addChild(YoGraphicDefinitionFactory.newYoGraphicPoint3D("contactPoint" + i, contactPointsWorld.get(i), 0.01, ColorDefinitions.Crimson()));
         group.addChild(YoGraphicDefinitionFactory.newYoGraphicArrow3D("contactNormal"
               + i, contactPointsWorld.get(i), normalVectors.get(i), 1.0, ColorDefinitions.Crimson()));
      }
      return group;
   }
}
