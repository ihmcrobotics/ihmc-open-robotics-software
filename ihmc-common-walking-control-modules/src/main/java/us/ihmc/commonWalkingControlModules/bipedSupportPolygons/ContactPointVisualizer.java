package us.ihmc.commonWalkingControlModules.bipedSupportPolygons;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.commonWalkingControlModules.controllers.Updatable;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicVector;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoFramePoint3D;
import us.ihmc.yoVariables.variable.YoFrameVector3D;

/**
 * @author twan
 *         Date: 5/28/13
 */
public class ContactPointVisualizer implements Updatable
{
   private final static ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final List<YoFramePoint3D> contactPointsWorld = new ArrayList<YoFramePoint3D>();
   private final List<YoGraphicPosition> yoGraphicPositions = new ArrayList<YoGraphicPosition>();
   private final List<YoGraphicVector> yoGraphicVectors = new ArrayList<YoGraphicVector>();
   private final List<YoFrameVector3D> normalVectors = new ArrayList<YoFrameVector3D>();
   private final double normalVectorScale = 0.1;
   private final int maxNumberOfYoGraphicPositions;
   private final ArrayList<? extends PlaneContactState> contactStates;

   public ContactPointVisualizer(ArrayList<? extends PlaneContactState> contactStates, YoGraphicsListRegistry yoGraphicsListRegistry,
         YoVariableRegistry parentRegistry)
   {
      this.contactStates = contactStates;
      int totalNumberOfContactPoints = 0;
      for (int i = 0; i < contactStates.size(); i++)
         totalNumberOfContactPoints += contactStates.get(i).getTotalNumberOfContactPoints();

      maxNumberOfYoGraphicPositions = totalNumberOfContactPoints;

      for (int i = 0; i < maxNumberOfYoGraphicPositions; i++)
      {
         YoFramePoint3D contactPointWorld = new YoFramePoint3D("contactPoint" + i, worldFrame, this.registry);
         contactPointsWorld.add(contactPointWorld);
         YoGraphicPosition yoGraphicPosition = new YoGraphicPosition("contactViz" + i, contactPointWorld, 0.01, YoAppearance.Crimson());
         yoGraphicPositions.add(yoGraphicPosition);
         yoGraphicsListRegistry.registerYoGraphic("contactPoints", yoGraphicPosition);

         YoFrameVector3D normalVector = new YoFrameVector3D("contactNormal" + i, worldFrame, registry);
         normalVectors.add(normalVector);
         YoGraphicVector yoGraphicVector = new YoGraphicVector("contactNormalViz" + i, contactPointWorld, normalVector, YoAppearance.Crimson());
         yoGraphicVectors.add(yoGraphicVector);
         yoGraphicsListRegistry.registerYoGraphic("contactPoints", yoGraphicVector);
      }
      parentRegistry.addChild(registry);
   }

   private final FramePoint3D tempFramePoint = new FramePoint3D(worldFrame);
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

         List<? extends ContactPointInterface> contactPoints = contactState.getContactPoints();
         for (int k = 0; k < contactPoints.size(); k++)
         {
            updateContactPointYoGraphics(i++, contactPoints.get(k));
         }
      }
   }

   private void updateContactPointYoGraphics(int i, ContactPointInterface contactPoint)
   {
      if (contactPoint.isInContact())
      {
         contactPoint.getPosition(tempFramePoint);
         tempFramePoint.changeFrame(worldFrame);
         contactPointsWorld.get(i).set(tempFramePoint);
         normalVectors.get(i).set(tempFrameVector);

         yoGraphicPositions.get(i).showGraphicObject();
         yoGraphicVectors.get(i).showGraphicObject();
      }
      else
      {
         contactPointsWorld.get(i).setToNaN();
         normalVectors.get(i).set(Double.NaN, Double.NaN, Double.NaN);
         yoGraphicPositions.get(i).hideGraphicObject();
         yoGraphicVectors.get(i).hideGraphicObject();
      }

      yoGraphicPositions.get(i).update();
      yoGraphicVectors.get(i).update();
   }
}
