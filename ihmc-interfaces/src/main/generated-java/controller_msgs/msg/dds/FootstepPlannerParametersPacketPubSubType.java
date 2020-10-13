package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "FootstepPlannerParametersPacket" defined in "FootstepPlannerParametersPacket_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from FootstepPlannerParametersPacket_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit FootstepPlannerParametersPacket_.idl instead.
*
*/
public class FootstepPlannerParametersPacketPubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.FootstepPlannerParametersPacket>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::FootstepPlannerParametersPacket_";

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.FootstepPlannerParametersPacket data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.FootstepPlannerParametersPacket data) throws java.io.IOException
   {
      deserializeCDR.deserialize(serializedPayload);
      read(data, deserializeCDR);
      deserializeCDR.finishDeserialize();
   }

   public static int getMaxCdrSerializedSize()
   {
      return getMaxCdrSerializedSize(0);
   }

   public static int getMaxCdrSerializedSize(int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.FootstepPlannerParametersPacket data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.FootstepPlannerParametersPacket data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);



      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.FootstepPlannerParametersPacket data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      cdr.write_type_7(data.getCheckForBodyBoxCollisions());

      cdr.write_type_7(data.getCheckForPathCollisions());

      cdr.write_type_6(data.getIdealFootstepWidth());

      cdr.write_type_6(data.getIdealFootstepLength());

      cdr.write_type_6(data.getIdealSideStepWidth());

      cdr.write_type_6(data.getIdealBackStepLength());

      cdr.write_type_6(data.getIdealStepLengthAtMaxStepZ());

      cdr.write_type_6(data.getWiggleInsideDeltaTarget());

      cdr.write_type_6(data.getWiggleInsideDeltaMinimum());

      cdr.write_type_6(data.getMaximumStepReach());

      cdr.write_type_6(data.getMaximumStepYaw());

      cdr.write_type_6(data.getMinimumStepWidth());

      cdr.write_type_6(data.getMinimumStepLength());

      cdr.write_type_6(data.getMinimumStepYaw());

      cdr.write_type_6(data.getMaximumStepReachWhenSteppingUp());

      cdr.write_type_6(data.getMaximumStepWidthWhenSteppingUp());

      cdr.write_type_6(data.getMaximumStepZWhenSteppingUp());

      cdr.write_type_6(data.getMaximumStepXWhenForwardAndDown());

      cdr.write_type_6(data.getMaximumStepYWhenForwardAndDown());

      cdr.write_type_6(data.getMaximumStepZWhenForwardAndDown());

      cdr.write_type_6(data.getMaximumStepZ());

      cdr.write_type_6(data.getMaximumSwingZ());

      cdr.write_type_6(data.getMaximumSwingReach());

      cdr.write_type_6(data.getMinimumStepZWhenFullyPitched());

      cdr.write_type_6(data.getMaximumStepXWhenFullyPitched());

      cdr.write_type_6(data.getStepYawReductionFactorAtMaxReach());

      cdr.write_type_6(data.getMinimumFootholdPercent());

      cdr.write_type_6(data.getMinimumSurfaceInclineRadians());

      cdr.write_type_7(data.getWiggleWhilePlanning());

      cdr.write_type_7(data.getRejectIfWiggleNotSatisfied());

      cdr.write_type_7(data.getEnableConcaveHullWiggler());

      cdr.write_type_6(data.getMaximumXyWiggleDistance());

      cdr.write_type_6(data.getMaximumYawWiggle());

      cdr.write_type_6(data.getMaximumZPenetrationOnValleyRegions());

      cdr.write_type_6(data.getMaximumStepWidth());

      cdr.write_type_6(data.getCliffBaseHeightToAvoid());

      cdr.write_type_6(data.getMinimumDistanceFromCliffBottoms());

      cdr.write_type_6(data.getCliffTopHeightToAvoid());

      cdr.write_type_6(data.getMinimumDistanceFromCliffTops());

      cdr.write_type_6(data.getBodyBoxHeight());

      cdr.write_type_6(data.getBodyBoxDepth());

      cdr.write_type_6(data.getBodyBoxWidth());

      cdr.write_type_6(data.getBodyBoxBaseX());

      cdr.write_type_6(data.getBodyBoxBaseY());

      cdr.write_type_6(data.getBodyBoxBaseZ());

      cdr.write_type_6(data.getMaximumSnapHeight());

      cdr.write_type_6(data.getMinClearanceFromStance());

      cdr.write_type_6(data.getFinalTurnProximity());

      cdr.write_type_6(data.getYawWeight());

      cdr.write_type_6(data.getPitchWeight());

      cdr.write_type_6(data.getRollWeight());

      cdr.write_type_6(data.getForwardWeight());

      cdr.write_type_6(data.getLateralWeight());

      cdr.write_type_6(data.getStepUpWeight());

      cdr.write_type_6(data.getStepDownWeight());

      cdr.write_type_6(data.getLongStepWeight());

      cdr.write_type_6(data.getFootholdAreaWeight());

      cdr.write_type_6(data.getCostPerStep());

      cdr.write_type_6(data.getAStarHeuristicsWeight());

      cdr.write_type_4(data.getNumberOfBoundingBoxChecks());

      cdr.write_type_6(data.getMaximum2dDistanceFromBoundingBoxToPenalize());

      cdr.write_type_6(data.getDistanceFromPathTolerance());

      cdr.write_type_6(data.getDeltaYawFromReferenceTolerance());

      cdr.write_type_2(data.getMaximumBranchFactor());

      cdr.write_type_7(data.getEnableExpansionMask());

      cdr.write_type_7(data.getEnableShinCollisionCheck());

      cdr.write_type_6(data.getShinToeClearance());

      cdr.write_type_6(data.getShinHeelClearance());

      cdr.write_type_6(data.getShinLength());

      cdr.write_type_6(data.getShinHeightOffet());

   }

   public static void read(controller_msgs.msg.dds.FootstepPlannerParametersPacket data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      data.setCheckForBodyBoxCollisions(cdr.read_type_7());
      	
      data.setCheckForPathCollisions(cdr.read_type_7());
      	
      data.setIdealFootstepWidth(cdr.read_type_6());
      	
      data.setIdealFootstepLength(cdr.read_type_6());
      	
      data.setIdealSideStepWidth(cdr.read_type_6());
      	
      data.setIdealBackStepLength(cdr.read_type_6());
      	
      data.setIdealStepLengthAtMaxStepZ(cdr.read_type_6());
      	
      data.setWiggleInsideDeltaTarget(cdr.read_type_6());
      	
      data.setWiggleInsideDeltaMinimum(cdr.read_type_6());
      	
      data.setMaximumStepReach(cdr.read_type_6());
      	
      data.setMaximumStepYaw(cdr.read_type_6());
      	
      data.setMinimumStepWidth(cdr.read_type_6());
      	
      data.setMinimumStepLength(cdr.read_type_6());
      	
      data.setMinimumStepYaw(cdr.read_type_6());
      	
      data.setMaximumStepReachWhenSteppingUp(cdr.read_type_6());
      	
      data.setMaximumStepWidthWhenSteppingUp(cdr.read_type_6());
      	
      data.setMaximumStepZWhenSteppingUp(cdr.read_type_6());
      	
      data.setMaximumStepXWhenForwardAndDown(cdr.read_type_6());
      	
      data.setMaximumStepYWhenForwardAndDown(cdr.read_type_6());
      	
      data.setMaximumStepZWhenForwardAndDown(cdr.read_type_6());
      	
      data.setMaximumStepZ(cdr.read_type_6());
      	
      data.setMaximumSwingZ(cdr.read_type_6());
      	
      data.setMaximumSwingReach(cdr.read_type_6());
      	
      data.setMinimumStepZWhenFullyPitched(cdr.read_type_6());
      	
      data.setMaximumStepXWhenFullyPitched(cdr.read_type_6());
      	
      data.setStepYawReductionFactorAtMaxReach(cdr.read_type_6());
      	
      data.setMinimumFootholdPercent(cdr.read_type_6());
      	
      data.setMinimumSurfaceInclineRadians(cdr.read_type_6());
      	
      data.setWiggleWhilePlanning(cdr.read_type_7());
      	
      data.setRejectIfWiggleNotSatisfied(cdr.read_type_7());
      	
      data.setEnableConcaveHullWiggler(cdr.read_type_7());
      	
      data.setMaximumXyWiggleDistance(cdr.read_type_6());
      	
      data.setMaximumYawWiggle(cdr.read_type_6());
      	
      data.setMaximumZPenetrationOnValleyRegions(cdr.read_type_6());
      	
      data.setMaximumStepWidth(cdr.read_type_6());
      	
      data.setCliffBaseHeightToAvoid(cdr.read_type_6());
      	
      data.setMinimumDistanceFromCliffBottoms(cdr.read_type_6());
      	
      data.setCliffTopHeightToAvoid(cdr.read_type_6());
      	
      data.setMinimumDistanceFromCliffTops(cdr.read_type_6());
      	
      data.setBodyBoxHeight(cdr.read_type_6());
      	
      data.setBodyBoxDepth(cdr.read_type_6());
      	
      data.setBodyBoxWidth(cdr.read_type_6());
      	
      data.setBodyBoxBaseX(cdr.read_type_6());
      	
      data.setBodyBoxBaseY(cdr.read_type_6());
      	
      data.setBodyBoxBaseZ(cdr.read_type_6());
      	
      data.setMaximumSnapHeight(cdr.read_type_6());
      	
      data.setMinClearanceFromStance(cdr.read_type_6());
      	
      data.setFinalTurnProximity(cdr.read_type_6());
      	
      data.setYawWeight(cdr.read_type_6());
      	
      data.setPitchWeight(cdr.read_type_6());
      	
      data.setRollWeight(cdr.read_type_6());
      	
      data.setForwardWeight(cdr.read_type_6());
      	
      data.setLateralWeight(cdr.read_type_6());
      	
      data.setStepUpWeight(cdr.read_type_6());
      	
      data.setStepDownWeight(cdr.read_type_6());
      	
      data.setLongStepWeight(cdr.read_type_6());
      	
      data.setFootholdAreaWeight(cdr.read_type_6());
      	
      data.setCostPerStep(cdr.read_type_6());
      	
      data.setAStarHeuristicsWeight(cdr.read_type_6());
      	
      data.setNumberOfBoundingBoxChecks(cdr.read_type_4());
      	
      data.setMaximum2dDistanceFromBoundingBoxToPenalize(cdr.read_type_6());
      	
      data.setDistanceFromPathTolerance(cdr.read_type_6());
      	
      data.setDeltaYawFromReferenceTolerance(cdr.read_type_6());
      	
      data.setMaximumBranchFactor(cdr.read_type_2());
      	
      data.setEnableExpansionMask(cdr.read_type_7());
      	
      data.setEnableShinCollisionCheck(cdr.read_type_7());
      	
      data.setShinToeClearance(cdr.read_type_6());
      	
      data.setShinHeelClearance(cdr.read_type_6());
      	
      data.setShinLength(cdr.read_type_6());
      	
      data.setShinHeightOffet(cdr.read_type_6());
      	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.FootstepPlannerParametersPacket data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_7("check_for_body_box_collisions", data.getCheckForBodyBoxCollisions());
      ser.write_type_7("check_for_path_collisions", data.getCheckForPathCollisions());
      ser.write_type_6("ideal_footstep_width", data.getIdealFootstepWidth());
      ser.write_type_6("ideal_footstep_length", data.getIdealFootstepLength());
      ser.write_type_6("ideal_side_step_width", data.getIdealSideStepWidth());
      ser.write_type_6("ideal_back_step_length", data.getIdealBackStepLength());
      ser.write_type_6("ideal_step_length_at_max_step_z", data.getIdealStepLengthAtMaxStepZ());
      ser.write_type_6("wiggle_inside_delta_target", data.getWiggleInsideDeltaTarget());
      ser.write_type_6("wiggle_inside_delta_minimum", data.getWiggleInsideDeltaMinimum());
      ser.write_type_6("maximum_step_reach", data.getMaximumStepReach());
      ser.write_type_6("maximum_step_yaw", data.getMaximumStepYaw());
      ser.write_type_6("minimum_step_width", data.getMinimumStepWidth());
      ser.write_type_6("minimum_step_length", data.getMinimumStepLength());
      ser.write_type_6("minimum_step_yaw", data.getMinimumStepYaw());
      ser.write_type_6("maximum_step_reach_when_stepping_up", data.getMaximumStepReachWhenSteppingUp());
      ser.write_type_6("maximum_step_width_when_stepping_up", data.getMaximumStepWidthWhenSteppingUp());
      ser.write_type_6("maximum_step_z_when_stepping_up", data.getMaximumStepZWhenSteppingUp());
      ser.write_type_6("maximum_step_x_when_forward_and_down", data.getMaximumStepXWhenForwardAndDown());
      ser.write_type_6("maximum_step_y_when_forward_and_down", data.getMaximumStepYWhenForwardAndDown());
      ser.write_type_6("maximum_step_z_when_forward_and_down", data.getMaximumStepZWhenForwardAndDown());
      ser.write_type_6("maximum_step_z", data.getMaximumStepZ());
      ser.write_type_6("maximum_swing_z", data.getMaximumSwingZ());
      ser.write_type_6("maximum_swing_reach", data.getMaximumSwingReach());
      ser.write_type_6("minimum_step_z_when_fully_pitched", data.getMinimumStepZWhenFullyPitched());
      ser.write_type_6("maximum_step_x_when_fully_pitched", data.getMaximumStepXWhenFullyPitched());
      ser.write_type_6("step_yaw_reduction_factor_at_max_reach", data.getStepYawReductionFactorAtMaxReach());
      ser.write_type_6("minimum_foothold_percent", data.getMinimumFootholdPercent());
      ser.write_type_6("minimum_surface_incline_radians", data.getMinimumSurfaceInclineRadians());
      ser.write_type_7("wiggle_while_planning", data.getWiggleWhilePlanning());
      ser.write_type_7("reject_if_wiggle_not_satisfied", data.getRejectIfWiggleNotSatisfied());
      ser.write_type_7("enable_concave_hull_wiggler", data.getEnableConcaveHullWiggler());
      ser.write_type_6("maximum_xy_wiggle_distance", data.getMaximumXyWiggleDistance());
      ser.write_type_6("maximum_yaw_wiggle", data.getMaximumYawWiggle());
      ser.write_type_6("maximum_z_penetration_on_valley_regions", data.getMaximumZPenetrationOnValleyRegions());
      ser.write_type_6("maximum_step_width", data.getMaximumStepWidth());
      ser.write_type_6("cliff_base_height_to_avoid", data.getCliffBaseHeightToAvoid());
      ser.write_type_6("minimum_distance_from_cliff_bottoms", data.getMinimumDistanceFromCliffBottoms());
      ser.write_type_6("cliff_top_height_to_avoid", data.getCliffTopHeightToAvoid());
      ser.write_type_6("minimum_distance_from_cliff_tops", data.getMinimumDistanceFromCliffTops());
      ser.write_type_6("body_box_height", data.getBodyBoxHeight());
      ser.write_type_6("body_box_depth", data.getBodyBoxDepth());
      ser.write_type_6("body_box_width", data.getBodyBoxWidth());
      ser.write_type_6("body_box_base_x", data.getBodyBoxBaseX());
      ser.write_type_6("body_box_base_y", data.getBodyBoxBaseY());
      ser.write_type_6("body_box_base_z", data.getBodyBoxBaseZ());
      ser.write_type_6("maximum_snap_height", data.getMaximumSnapHeight());
      ser.write_type_6("min_clearance_from_stance", data.getMinClearanceFromStance());
      ser.write_type_6("final_turn_proximity", data.getFinalTurnProximity());
      ser.write_type_6("yaw_weight", data.getYawWeight());
      ser.write_type_6("pitch_weight", data.getPitchWeight());
      ser.write_type_6("roll_weight", data.getRollWeight());
      ser.write_type_6("forward_weight", data.getForwardWeight());
      ser.write_type_6("lateral_weight", data.getLateralWeight());
      ser.write_type_6("step_up_weight", data.getStepUpWeight());
      ser.write_type_6("step_down_weight", data.getStepDownWeight());
      ser.write_type_6("long_step_weight", data.getLongStepWeight());
      ser.write_type_6("foothold_area_weight", data.getFootholdAreaWeight());
      ser.write_type_6("cost_per_step", data.getCostPerStep());
      ser.write_type_6("a_star_heuristics_weight", data.getAStarHeuristicsWeight());
      ser.write_type_4("number_of_bounding_box_checks", data.getNumberOfBoundingBoxChecks());
      ser.write_type_6("maximum_2d_distance_from_bounding_box_to_penalize", data.getMaximum2dDistanceFromBoundingBoxToPenalize());
      ser.write_type_6("distance_from_path_tolerance", data.getDistanceFromPathTolerance());
      ser.write_type_6("delta_yaw_from_reference_tolerance", data.getDeltaYawFromReferenceTolerance());
      ser.write_type_2("maximum_branch_factor", data.getMaximumBranchFactor());
      ser.write_type_7("enable_expansion_mask", data.getEnableExpansionMask());
      ser.write_type_7("enable_shin_collision_check", data.getEnableShinCollisionCheck());
      ser.write_type_6("shin_toe_clearance", data.getShinToeClearance());
      ser.write_type_6("shin_heel_clearance", data.getShinHeelClearance());
      ser.write_type_6("shin_length", data.getShinLength());
      ser.write_type_6("shin_height_offet", data.getShinHeightOffet());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.FootstepPlannerParametersPacket data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      data.setCheckForBodyBoxCollisions(ser.read_type_7("check_for_body_box_collisions"));
      data.setCheckForPathCollisions(ser.read_type_7("check_for_path_collisions"));
      data.setIdealFootstepWidth(ser.read_type_6("ideal_footstep_width"));
      data.setIdealFootstepLength(ser.read_type_6("ideal_footstep_length"));
      data.setIdealSideStepWidth(ser.read_type_6("ideal_side_step_width"));
      data.setIdealBackStepLength(ser.read_type_6("ideal_back_step_length"));
      data.setIdealStepLengthAtMaxStepZ(ser.read_type_6("ideal_step_length_at_max_step_z"));
      data.setWiggleInsideDeltaTarget(ser.read_type_6("wiggle_inside_delta_target"));
      data.setWiggleInsideDeltaMinimum(ser.read_type_6("wiggle_inside_delta_minimum"));
      data.setMaximumStepReach(ser.read_type_6("maximum_step_reach"));
      data.setMaximumStepYaw(ser.read_type_6("maximum_step_yaw"));
      data.setMinimumStepWidth(ser.read_type_6("minimum_step_width"));
      data.setMinimumStepLength(ser.read_type_6("minimum_step_length"));
      data.setMinimumStepYaw(ser.read_type_6("minimum_step_yaw"));
      data.setMaximumStepReachWhenSteppingUp(ser.read_type_6("maximum_step_reach_when_stepping_up"));
      data.setMaximumStepWidthWhenSteppingUp(ser.read_type_6("maximum_step_width_when_stepping_up"));
      data.setMaximumStepZWhenSteppingUp(ser.read_type_6("maximum_step_z_when_stepping_up"));
      data.setMaximumStepXWhenForwardAndDown(ser.read_type_6("maximum_step_x_when_forward_and_down"));
      data.setMaximumStepYWhenForwardAndDown(ser.read_type_6("maximum_step_y_when_forward_and_down"));
      data.setMaximumStepZWhenForwardAndDown(ser.read_type_6("maximum_step_z_when_forward_and_down"));
      data.setMaximumStepZ(ser.read_type_6("maximum_step_z"));
      data.setMaximumSwingZ(ser.read_type_6("maximum_swing_z"));
      data.setMaximumSwingReach(ser.read_type_6("maximum_swing_reach"));
      data.setMinimumStepZWhenFullyPitched(ser.read_type_6("minimum_step_z_when_fully_pitched"));
      data.setMaximumStepXWhenFullyPitched(ser.read_type_6("maximum_step_x_when_fully_pitched"));
      data.setStepYawReductionFactorAtMaxReach(ser.read_type_6("step_yaw_reduction_factor_at_max_reach"));
      data.setMinimumFootholdPercent(ser.read_type_6("minimum_foothold_percent"));
      data.setMinimumSurfaceInclineRadians(ser.read_type_6("minimum_surface_incline_radians"));
      data.setWiggleWhilePlanning(ser.read_type_7("wiggle_while_planning"));
      data.setRejectIfWiggleNotSatisfied(ser.read_type_7("reject_if_wiggle_not_satisfied"));
      data.setEnableConcaveHullWiggler(ser.read_type_7("enable_concave_hull_wiggler"));
      data.setMaximumXyWiggleDistance(ser.read_type_6("maximum_xy_wiggle_distance"));
      data.setMaximumYawWiggle(ser.read_type_6("maximum_yaw_wiggle"));
      data.setMaximumZPenetrationOnValleyRegions(ser.read_type_6("maximum_z_penetration_on_valley_regions"));
      data.setMaximumStepWidth(ser.read_type_6("maximum_step_width"));
      data.setCliffBaseHeightToAvoid(ser.read_type_6("cliff_base_height_to_avoid"));
      data.setMinimumDistanceFromCliffBottoms(ser.read_type_6("minimum_distance_from_cliff_bottoms"));
      data.setCliffTopHeightToAvoid(ser.read_type_6("cliff_top_height_to_avoid"));
      data.setMinimumDistanceFromCliffTops(ser.read_type_6("minimum_distance_from_cliff_tops"));
      data.setBodyBoxHeight(ser.read_type_6("body_box_height"));
      data.setBodyBoxDepth(ser.read_type_6("body_box_depth"));
      data.setBodyBoxWidth(ser.read_type_6("body_box_width"));
      data.setBodyBoxBaseX(ser.read_type_6("body_box_base_x"));
      data.setBodyBoxBaseY(ser.read_type_6("body_box_base_y"));
      data.setBodyBoxBaseZ(ser.read_type_6("body_box_base_z"));
      data.setMaximumSnapHeight(ser.read_type_6("maximum_snap_height"));
      data.setMinClearanceFromStance(ser.read_type_6("min_clearance_from_stance"));
      data.setFinalTurnProximity(ser.read_type_6("final_turn_proximity"));
      data.setYawWeight(ser.read_type_6("yaw_weight"));
      data.setPitchWeight(ser.read_type_6("pitch_weight"));
      data.setRollWeight(ser.read_type_6("roll_weight"));
      data.setForwardWeight(ser.read_type_6("forward_weight"));
      data.setLateralWeight(ser.read_type_6("lateral_weight"));
      data.setStepUpWeight(ser.read_type_6("step_up_weight"));
      data.setStepDownWeight(ser.read_type_6("step_down_weight"));
      data.setLongStepWeight(ser.read_type_6("long_step_weight"));
      data.setFootholdAreaWeight(ser.read_type_6("foothold_area_weight"));
      data.setCostPerStep(ser.read_type_6("cost_per_step"));
      data.setAStarHeuristicsWeight(ser.read_type_6("a_star_heuristics_weight"));
      data.setNumberOfBoundingBoxChecks(ser.read_type_4("number_of_bounding_box_checks"));
      data.setMaximum2dDistanceFromBoundingBoxToPenalize(ser.read_type_6("maximum_2d_distance_from_bounding_box_to_penalize"));
      data.setDistanceFromPathTolerance(ser.read_type_6("distance_from_path_tolerance"));
      data.setDeltaYawFromReferenceTolerance(ser.read_type_6("delta_yaw_from_reference_tolerance"));
      data.setMaximumBranchFactor(ser.read_type_2("maximum_branch_factor"));
      data.setEnableExpansionMask(ser.read_type_7("enable_expansion_mask"));
      data.setEnableShinCollisionCheck(ser.read_type_7("enable_shin_collision_check"));
      data.setShinToeClearance(ser.read_type_6("shin_toe_clearance"));
      data.setShinHeelClearance(ser.read_type_6("shin_heel_clearance"));
      data.setShinLength(ser.read_type_6("shin_length"));
      data.setShinHeightOffet(ser.read_type_6("shin_height_offet"));
   }

   public static void staticCopy(controller_msgs.msg.dds.FootstepPlannerParametersPacket src, controller_msgs.msg.dds.FootstepPlannerParametersPacket dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.FootstepPlannerParametersPacket createData()
   {
      return new controller_msgs.msg.dds.FootstepPlannerParametersPacket();
   }
   @Override
   public int getTypeSize()
   {
      return us.ihmc.idl.CDR.getTypeSize(getMaxCdrSerializedSize());
   }

   @Override
   public java.lang.String getName()
   {
      return name;
   }
   
   public void serialize(controller_msgs.msg.dds.FootstepPlannerParametersPacket data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.FootstepPlannerParametersPacket data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.FootstepPlannerParametersPacket src, controller_msgs.msg.dds.FootstepPlannerParametersPacket dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public FootstepPlannerParametersPacketPubSubType newInstance()
   {
      return new FootstepPlannerParametersPacketPubSubType();
   }
}
