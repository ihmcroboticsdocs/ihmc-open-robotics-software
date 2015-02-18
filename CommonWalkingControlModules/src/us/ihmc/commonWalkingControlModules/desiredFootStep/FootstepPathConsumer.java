package us.ihmc.commonWalkingControlModules.desiredFootStep;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import javax.vecmath.Point2d;

import us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator.FootstepTools;
import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.communication.packets.walking.FootstepData;
import us.ihmc.communication.packets.walking.FootstepDataList;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.PoseReferenceFrame;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.math.trajectories.providers.TrajectoryParameters;
import us.ihmc.utilities.robotSide.SideDependentList;
import us.ihmc.utilities.humanoidRobot.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.utilities.humanoidRobot.footstep.Footstep;

/**
 * User: Matt
 * Date: 1/18/13
 */
public class FootstepPathConsumer implements PacketConsumer<FootstepDataList>
{
   private boolean DEBUG = false;
   private FootstepPathCoordinator footstepPathCoordinator;
   private final SideDependentList<? extends ContactablePlaneBody> bipedFeet;

   public FootstepPathConsumer(SideDependentList<? extends ContactablePlaneBody> bipedFeet, FootstepPathCoordinator footstepPathCoordinator,
                               HashMap<Footstep, TrajectoryParameters> mapFromFootstepsToTrajectoryParameters)
   {
      this.footstepPathCoordinator = footstepPathCoordinator;
      this.bipedFeet = bipedFeet;
   }

   public void receivedPacket(FootstepDataList footstepList)
   {
      ArrayList<Footstep> footsteps = new ArrayList<Footstep>();

      for (int i = 0; i < footstepList.size(); i++)
      {
         FootstepData footstepData = footstepList.get(i);
         ContactablePlaneBody contactableBody = bipedFeet.get(footstepData.getRobotSide());
         Footstep footstep = FootstepTools.generateFootstepFromFootstepData(footstepData, contactableBody, i);
         footsteps.add(footstep);

         if (DEBUG)
         {
            System.out.println("FootstepPathConsumer received " + footstep);
         }
      }

      footstepPathCoordinator.setSwingTime(footstepList.swingTime);
      footstepPathCoordinator.setTransferTime(footstepList.transferTime);
      footstepPathCoordinator.updatePath(footsteps);
   }

}
