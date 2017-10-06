package us.ihmc.jMonkeyEngineToolkit.jme;

import com.jme3.light.SpotLight;
import com.jme3.math.Vector3f;

import us.ihmc.graphicsDescription.Graphics3DSpotLight;
import us.ihmc.jMonkeyEngineToolkit.jme.util.JMEDataTypeUtils;
import us.ihmc.jMonkeyEngineToolkit.jme.util.JMEGeometryUtils;

public class JMESpotLight extends SpotLight
{
   private final Graphics3DSpotLight spotLight;
   
   private final Vector3f jmePosition = new Vector3f();
   private final Vector3f jmeDirection = new Vector3f();
   
   private double innerAngle = 0;
   private double outerAngle = 0;
   
   public JMESpotLight(Graphics3DSpotLight spotLight)
   {
      super();
      this.spotLight = spotLight;
   }
   
   public void update()
   {
      JMEDataTypeUtils.packVecMathTuple3dInJMEVector3f(spotLight.getPosition(), jmePosition);
      JMEDataTypeUtils.packVecMathTuple3dInJMEVector3f(spotLight.getDirection(), jmeDirection);
      
      JMEGeometryUtils.transformFromZupToJMECoordinates(jmePosition);
      JMEGeometryUtils.transformFromZupToJMECoordinates(jmeDirection);
      
      setColor(JMEDataTypeUtils.colorToColorRGBA(spotLight.getColor()));
      
      setDirection(jmeDirection);
      setPosition(jmePosition);
      
      // Avoid calculating angles every update
      if(innerAngle != spotLight.getSpotInnerAngle())
      {
         setSpotInnerAngle((float) spotLight.getSpotInnerAngle());
         innerAngle = spotLight.getSpotInnerAngle();
      }
      
      if(outerAngle != spotLight.getSpotOuterAngle())
      {
         setSpotOuterAngle((float) spotLight.getSpotOuterAngle());
         outerAngle = spotLight.getSpotOuterAngle();
      }
      
      setSpotRange((float) spotLight.getSpotRange());
   }
}
