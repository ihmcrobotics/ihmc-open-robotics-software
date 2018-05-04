package us.ihmc.simulationconstructionset;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicVector;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoFramePoint3D;
import us.ihmc.yoVariables.variable.YoFrameVector3D;

public class ContactingExternalForcePointsVisualizer
{
   private final static ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final List<YoFramePoint3D> contactPointsWorld = new ArrayList<YoFramePoint3D>();
   private final List<YoGraphicPosition> contactPointsViz = new ArrayList<YoGraphicPosition>();
   
   private final List<YoFrameVector3D> contactNormals = new ArrayList<YoFrameVector3D>();
   private final List<YoGraphicVector> contactNormalsViz = new ArrayList<YoGraphicVector>();
   
   private final List<YoFrameVector3D> forceVectors = new ArrayList<YoFrameVector3D>();
   private final List<YoGraphicVector> forceVectorsViz = new ArrayList<YoGraphicVector>();

   private double normalVectorScale = 0.03;
   private double forceVectorScale = 0.01;

   private final ArrayList<ContactingExternalForcePoint> contactPoints = new ArrayList<>();

   public ContactingExternalForcePointsVisualizer(int maxNumberOfYoGraphicPositions, YoGraphicsListRegistry yoGraphicsListRegistry,
                                                  YoVariableRegistry parentRegistry)
   {
      for (int i = 0; i < maxNumberOfYoGraphicPositions; i++)
      {
         YoFramePoint3D contactPointWorld = new YoFramePoint3D("contactPoint" + i, worldFrame, this.registry);
         contactPointsWorld.add(contactPointWorld);
         YoGraphicPosition yoGraphicPosition = new YoGraphicPosition("contactViz" + i, contactPointWorld, 0.01, YoAppearance.Crimson());
         contactPointsViz.add(yoGraphicPosition);
         yoGraphicsListRegistry.registerYoGraphic("contactPoints", yoGraphicPosition);

         YoFrameVector3D normalVector = new YoFrameVector3D("contactNormal" + i, worldFrame, registry);
         contactNormals.add(normalVector);
         YoGraphicVector contactNormalViz = new YoGraphicVector("contactNormalViz" + i, contactPointWorld, normalVector, YoAppearance.Gold());
         contactNormalViz.setDrawArrowhead(false);
         contactNormalViz.setLineRadiusWhenOneMeterLong(0.005);
         contactNormalsViz.add(contactNormalViz);
         yoGraphicsListRegistry.registerYoGraphic("contactPoints", contactNormalViz);

         YoFrameVector3D forceVector = new YoFrameVector3D("forceVector" + i, worldFrame, registry);
         forceVectors.add(forceVector);
         YoGraphicVector forceVectorViz = new YoGraphicVector("forceVectorViz" + i, contactPointWorld, forceVector, YoAppearance.Crimson());
         forceVectorViz.setDrawArrowhead(false);
         forceVectorsViz.add(forceVectorViz);
         yoGraphicsListRegistry.registerYoGraphic("contactPoints", forceVectorViz);
      }
      parentRegistry.addChild(registry);
   }

   public void setNormalVectorScale(double normalVectorScale)
   {
      this.normalVectorScale = normalVectorScale;
   }

   public void setForceVectorScale(double forceVectorScale)
   {
      this.forceVectorScale = forceVectorScale;
   }

   public void addPoints(ArrayList<ContactingExternalForcePoint> contactingExternalForcePoints)
   {
      this.contactPoints.addAll(contactingExternalForcePoints);
   }

   private final Point3D tempPoint = new Point3D();
   private final Vector3D tempVector = new Vector3D();

   public void update()
   {
      int yoGraphicIndex = 0;
      for (int contactPointIndex = 0; contactPointIndex < Math.min(contactPoints.size(), contactPointsWorld.size()); contactPointIndex++)
      {
         ContactingExternalForcePoint contactPoint = contactPoints.get(contactPointIndex);

         if (contactPoint.isInContact())
         {
            contactPoint.getPosition(tempPoint);
            contactPointsWorld.get(yoGraphicIndex).set(tempPoint);

            contactPoint.getSurfaceNormalInWorld(tempVector);
            tempVector.scale(normalVectorScale);
            contactNormals.get(yoGraphicIndex).set(tempVector);

            contactPoint.getForce(tempVector);
            tempVector.scale(forceVectorScale);
            forceVectors.get(yoGraphicIndex).set(tempVector);

//            contactPointsViz.get(yoGraphicIndex).showGraphicObject();
//            contactNormalsViz.get(yoGraphicIndex).showGraphicObject();
//            forceVectorsViz.get(yoGraphicIndex).showGraphicObject();

            contactPointsViz.get(yoGraphicIndex).update();
            contactNormalsViz.get(yoGraphicIndex).update();
            forceVectorsViz.get(yoGraphicIndex).update();

            yoGraphicIndex++;
         }
      }

      for (int j = yoGraphicIndex; j < contactPointsViz.size(); j++)
      {
         contactPointsWorld.get(j).setToNaN();
         contactNormals.get(j).set(Double.NaN, Double.NaN, Double.NaN);
//         contactPointsViz.get(j).hideGraphicObject();
//         contactNormalsViz.get(j).hideGraphicObject();
      }

   }

}
