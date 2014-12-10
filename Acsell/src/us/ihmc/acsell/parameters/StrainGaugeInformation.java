package us.ihmc.acsell.parameters;

import us.ihmc.steppr.hardware.StepprActuator;

public class StrainGaugeInformation
{
   private StepprActuator strainSensorBoard;
   private int strainSensorConnectorId;
   private double strainSensorGain, strainSensorOffset;
   
   public StrainGaugeInformation(StepprActuator strainSensorBoard, int strainSensorConnectorId, double strainSensorGain, double strainSensorOffset)
   {
      this.strainSensorBoard = strainSensorBoard;
      this.strainSensorConnectorId = strainSensorConnectorId;    
      this.strainSensorGain = strainSensorGain;
      this.strainSensorOffset = strainSensorOffset;
   }
   public StepprActuator getStrainSensorBoard()
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