package us.ihmc.rdx.perception;

import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.graphicsDescription.MeshDataHolder;
import us.ihmc.perception.sensorHead.BlackflyLensProperties;
import us.ihmc.perception.spinnaker.BlackflyModelProperties;
import us.ihmc.rdx.mesh.MeshDataGeneratorMissing;
import us.ihmc.rdx.perception.RDXProjectionHemisphere.FisheyeTextureCalculator;
import us.ihmc.rdx.perception.RDXProjectionHemisphere.FisheyeTextureCalculator.CameraOrientation;

public class RDXProjectionHemisphereTest
{
   @Disabled
   @Test
   public void testJavaTextureCalculator()
   {
      {
         float xRadius = 1.0f;
         float yRadius = 1.0f;
         float zRadius = 1.0f;
         int latitudeResolution = 12;
         int longitudeResolution = 12;

         MeshDataHolder meshDataHolder = MeshDataGeneratorMissing.InvertedHemiEllipsoidNoBottom(xRadius,
                                                                                                yRadius,
                                                                                                zRadius,
                                                                                                latitudeResolution,
                                                                                                longitudeResolution);
         Quaternion rotation = new Quaternion();
         rotation.setYawPitchRoll(0.0, Math.PI, 0.0);
         meshDataHolder = MeshDataHolder.rotate(meshDataHolder, rotation);

         BlackflyLensProperties blackflyLensProperties = BlackflyLensProperties.forModel(BlackflyModelProperties.BFLY_U3_23S6C);
         double fx = blackflyLensProperties.getFocalLengthXForUndistortion();
         double fy = blackflyLensProperties.getFocalLengthYForUndistortion();
         double cx = blackflyLensProperties.getPrincipalPointXForUndistortion();
         double cy = blackflyLensProperties.getPrincipalPointYForUndistortion();
         double k1 = blackflyLensProperties.getK1ForUndistortion();
         double k2 = blackflyLensProperties.getK2ForUndistortion();
         double k3 = blackflyLensProperties.getK3ForUndistortion();
         double k4 = blackflyLensProperties.getK4ForUndistortion();
         FisheyeTextureCalculator fisheyeTextureCalculator = new FisheyeTextureCalculator(CameraOrientation.Z_DEPTH_POSITIVE,
                                                                                          fx,
                                                                                          fy,
                                                                                          cx,
                                                                                          cy,
                                                                                          k1,
                                                                                          k2,
                                                                                          k3,
                                                                                          k4,
                                                                                          Math.toRadians(170.0));
         fisheyeTextureCalculator.accept(meshDataHolder.getVertices(), meshDataHolder.getTexturePoints());

         for (int i = 0; i < meshDataHolder.getTexturePoints().length; i++)
         {
            System.out.printf("p=(%.2f, %.2f, %.2f) -> t=(%.2f, %.2f)%n",
                              meshDataHolder.getVertices()[i].getX(),
                              meshDataHolder.getVertices()[i].getY(),
                              meshDataHolder.getVertices()[i].getZ(),
                              meshDataHolder.getTexturePoints()[i].getX(),
                              meshDataHolder.getTexturePoints()[i].getY());
         }
      }
   }
}
