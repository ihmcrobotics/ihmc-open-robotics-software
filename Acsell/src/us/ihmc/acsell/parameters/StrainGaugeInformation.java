package us.ihmc.acsell.parameters;

import us.ihmc.acsell.hardware.AcsellActuator;

public class StrainGaugeInformation
{
   private AcsellActuator strainSensorBoard;
   private int strainSensorConnectorId;
   private double strainSensorGain, strainSensorOffset;
   
   public StrainGaugeInformation(AcsellActuator rightHipZ, int strainSensorConnectorId, double strainSensorGain, double strainSensorOffset)
   {
      this.strainSensorBoard = rightHipZ;
      this.strainSensorConnectorId = strainSensorConnectorId;    
      this.strainSensorGain = strainSensorGain;
      this.strainSensorOffset = strainSensorOffset;
   }
   public AcsellActuator getStrainSensorBoard()
   {
      return strainSensorBoard;
   }
   
   public int getStrainSensorConnectorId()
   {
      return strainSensorConnectorId;
   }

   public double getStrainSensorOffset()
   {
      return strainSensorOffset;
   }

   public double getStrainSensorGain()
   {      
      return strainSensorGain;
   }
};