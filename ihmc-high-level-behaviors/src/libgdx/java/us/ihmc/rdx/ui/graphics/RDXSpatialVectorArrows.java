package us.ihmc.rdx.ui.graphics;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import us.ihmc.behaviors.tools.yo.YoDoubleClientHelper;
import us.ihmc.behaviors.tools.yo.YoVariableClientHelper;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.mecano.spatial.interfaces.SpatialVectorReadOnly;
import us.ihmc.rdx.tools.LibGDXTools;

public class RDXSpatialVectorArrows
{
   private final RDXArrowGraphic linearPartArrow = new RDXArrowGraphic(Color.RED);
   private final RDXArrowGraphic angularPartArrow = new RDXArrowGraphic(Color.PURPLE);

   private final RigidBodyTransform tempTransform = new RigidBodyTransform();
   private final FramePoint3D origin = new FramePoint3D();
   private final ReferenceFrame originFrame;
   private YoVariableClientHelper yoVariableClientHelper;
   private int indexOfSensor = -1;
   private YoDoubleClientHelper linearXYoVariable;
   private YoDoubleClientHelper linearYYoVariable;
   private YoDoubleClientHelper linearZYoVariable;
   private YoDoubleClientHelper angularXYoVariable;
   private YoDoubleClientHelper angularYYoVariable;
   private YoDoubleClientHelper angularZYoVariable;
   private final Vector3D tempVector = new Vector3D();

   private double linearPartScale = 0.005;
   private double angularPartScale = 0.02;
   private boolean show = true;

   public RDXSpatialVectorArrows(ReferenceFrame originFrame, YoVariableClientHelper yoVariableClientHelper, String variablePrefix)
   {
      this.originFrame = originFrame;
      this.yoVariableClientHelper = yoVariableClientHelper;
      linearXYoVariable = yoVariableClientHelper.subscribeToYoDouble(variablePrefix + "ForceX");
      linearYYoVariable = yoVariableClientHelper.subscribeToYoDouble(variablePrefix + "ForceY");
      linearZYoVariable = yoVariableClientHelper.subscribeToYoDouble(variablePrefix + "ForceZ");
      angularXYoVariable = yoVariableClientHelper.subscribeToYoDouble(variablePrefix + "TorqueX");
      angularYYoVariable = yoVariableClientHelper.subscribeToYoDouble(variablePrefix + "TorqueY");
      angularZYoVariable = yoVariableClientHelper.subscribeToYoDouble(variablePrefix + "TorqueZ");
   }

   public RDXSpatialVectorArrows(ReferenceFrame originFrame)
   {
      this.originFrame = originFrame;
   }

   public RDXSpatialVectorArrows(ReferenceFrame originFrame, int indexOfSensor)
   {
      this.originFrame = originFrame;
      this.indexOfSensor = indexOfSensor;
   }

   public void update(SpatialVectorReadOnly spatialVector)
   {
      update(spatialVector.getLinearPart(), spatialVector.getAngularPart());
   }

   public void update(Vector3DReadOnly linearPart, Vector3DReadOnly angularPart)
   {
      origin.setToZero(originFrame);
      origin.changeFrame(ReferenceFrame.getWorldFrame());

      transform(linearPartArrow, linearPart, linearPartScale);
      transform(angularPartArrow, angularPart, angularPartScale);
   }

   public void updateFromYoVariables()
   {
      if (yoVariableClientHelper.isConnected())
      {
         origin.setToZero(originFrame);
         origin.changeFrame(ReferenceFrame.getWorldFrame());

         tempVector.set(linearXYoVariable.get(),
                        linearYYoVariable.get(),
                        linearZYoVariable.get());
         transform(linearPartArrow, tempVector, 0.005);
         tempVector.set(angularXYoVariable.get(),
                        angularYYoVariable.get(),
                        angularZYoVariable.get());
         transform(angularPartArrow, tempVector, 0.005);
      }
   }

   private void transform(RDXArrowGraphic arrowGraphic, Vector3DReadOnly vector, double scalar)
   {
      double length = vector.length();
      arrowGraphic.update(length * scalar);

      tempTransform.setToZero();
      EuclidGeometryTools.orientation3DFromZUpToVector3D(vector, tempTransform.getRotation());
      tempTransform.getTranslation().set(origin);
      LibGDXTools.toLibGDX(tempTransform, arrowGraphic.getDynamicModel().getOrCreateModelInstance().transform);
   }

   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      if (show)
      {
         linearPartArrow.getRenderables(renderables, pool);
         angularPartArrow.getRenderables(renderables, pool);
      }
   }

   public int getIndexOfSensor()
   {
      return indexOfSensor;
   }

   public void setLinearPartScale(double linearPartScale)
   {
      this.linearPartScale = linearPartScale;
   }

   public void setAngularPartScale(double angularPartScale)
   {
      this.angularPartScale = angularPartScale;
   }

   public boolean getShow()
   {
      return show;
   }

   public void setShow(boolean show)
   {
      this.show = show;
   }
}
